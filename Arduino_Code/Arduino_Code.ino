#include <OneWire.h>

int DS18S20_Pin = 2; //DS18S20 Signal pin on digital 2
int l=1;
double result;
int trig = 12;
int echo = 11;



//Temperature chip i/o
OneWire ds(DS18S20_Pin);  // on digital pin 2, seems to be a way of defining OneWire device object, coz saw classes in the OneWire header file.

//conductivity
const float ArduinoVoltage = 5.00; // CHANGE THIS FOR 3.3v Arduinos
const float ArduinoResolution = ArduinoVoltage / 1024;
const float resistorValue = 10000.0;
int threshold = 3;
int inputPin = A0;
int ouputPin = A5;
int inputPinTurbidity=A1;
double conductivity = -1;
double temperature = -1;
double turbidity = -1;
double waterLevel = -1;
double tds =-1;

void setup(void) {
 
  pinMode(ouputPin, OUTPUT);
  pinMode(inputPin, INPUT);
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);
  Serial.begin(9600);
 
  delay(5000);

}

void loop(void) {
  temperature = temperatureFunction();
  //Serial.print("Temperature = ");
  //Serial.println(temperature);
 
  turbidity = turbidityFunction();
  //Serial.print("Turbidity = ");
  //Serial.println(turbidity);
 
  conductivity = conductivityFunction();
  //Serial.print("Conductivity = ");
  //Serial.println(conductivity);

  waterLevel = ultrasonicFunction();
  //Serial.print("water level = ");
  //Serial.println(waterLevel);


  tds = conductivity*0.67;



char str_temp[10];
dtostrf(temperature,3,2,str_temp);
 //itoa((int)temperature,str_temp,10);
 //str has temp string.
//char str2[10];

char str_conductivity[10];
dtostrf(conductivity,3,2,str_conductivity);

char str_turbidity[10];
dtostrf(turbidity,3,2,str_turbidity);

char str_waterLevel[10];
dtostrf(waterLevel,3,2,str_waterLevel);

char str_tds[10];
dtostrf(tds,3,2,str_tds);


String str_final = "";

str_final += "Temp: ";
str_final += str_temp;
str_final += " Celcius\nCond: ";
str_final += str_conductivity;
str_final += " mS/meter\nTurb: ";
str_final += str_turbidity;
str_final += " NTU\nLevel: ";
str_final += str_waterLevel;
str_final += " cm\nTDS: ";
str_final += str_tds;
str_final += " mg/L\n";


Serial.println("AT");
delay(1000);
Serial.println("AT+CMGF=1");
delay(1000);
Serial.println("AT+CMGS=\"7600453121\"");
delay(1000);
Serial.print(str_final); //used for gsm
Serial.write(26);
delay(20000);      //after 100 sec message is sent   
}


double turbidityFunction() {
  int i=0;
  float voltage=0;
  for(i=0;i<10;i++)
  {
  // read the input on analog pin 0:
  int sensorValue = analogRead(inputPinTurbidity);
  // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V):
  voltage = sensorValue * (5.0 / 1023.0);
  voltage = 7.5 -voltage;
  // print out the value you read:
  //Serial.println(voltage);
  }

  return voltage;
}

double temperatureFunction() {
   for(l=1;l<=10;l++)
      {
         result=getTemp();
         //Serial.println(result); //not needed in the final code
         delay(100); //just here to slow down the output so it is easier to read
      }
      
      //erial.println(result);
  // now send to gsm
 
 
//Serial.write(26);
delay(1000); // delay in between reads for stability
   
delay(10*60); //compute and send result after 10 mins.
   
 return result;
}

 double getTemp(){
  //returns the temperature from one DS18S20 in DEG Celsius
//Serial.print("hey there1\n");
  byte data[12];
  byte addr[8];

ds.reset_search();
//Serial.print("hey there4\n");
 if ( !ds.search(addr)) {
      //no more sensors on chain, reset search
      ds.reset_search();
      return -1000;
  }
//Serial.print(" there4\n");
  if ( OneWire::crc8( addr, 7) != addr[7]) {
      Serial.print("CRC is not valid!\n");
      return -1000;
  }

  if ( addr[0] != 0x10 && addr[0] != 0x28) {
      Serial.print("Device is not recognized\n");
      return -1000;
  }

//read step 1 : conversion process
  ds.reset(); // reset every time temperature is to be found
  ds.select(addr); //addr contains the address of the temp sensor.
  ds.write(0x44,1); // start conversion, with parasite power on at the end

//MAYBE WE WILL TI WWAIT FOR MIN 100ms
   delay(100);     // maybe 750ms is enough, maybe not
 
//read step 2 :
  byte present = ds.reset();
  ds.select(addr);    
  ds.write(0xBE); // Read Scratchpad

 
  for (int i = 0; i < 9; i++) { // we need 9 bytes
    data[i] = ds.read();
  }
 
 // ds.reset_search();
 
  int MSB = data[1];
  int LSB = data[0];

  int TReading = ((MSB << 8)+LSB); //2's compliment, we say: upcasting hence the two bytes coalescese.
 
  int SignBit = TReading & 0x8000;  // test most sig bit
  if (SignBit) // negative
  {
    TReading = (TReading ^ 0xffff) + 1; // 2's comp
  }
int Tc_100 = (6 * TReading) + TReading / 4;    // multiply by (100 * 0.0625) or 6.25
 //Serial.print("hello!\n");
 

return (Tc_100/100.0);
 
}

/*this function takes n readings and sends the last one. The initial readings are not taken because it takes time to settle.*/
double conductivityFunction(){

  int n = 7,i;
 
  int analogValue=0;
  int oldAnalogValue=1000;
  float returnVoltage=0.0;
  float returnVoltageAcrossProbe;
  float resistance=0.0;
  double Siemens , conductivity;
  float l = 0.02;
  float A = 0.0001;
  float TDS=0.0;
 
 
  for(i=0 ; i<n ; i++){
    analogValue=0;
    oldAnalogValue=1000;
    returnVoltage=0.0;
    returnVoltageAcrossProbe;
    resistance=0.0;
    Siemens , conductivity;
    l = 0.02;
    A = 0.0001;
    TDS=0.0;
    
    while(((oldAnalogValue-analogValue)>threshold) || (oldAnalogValue<50))
    {
      oldAnalogValue = analogValue;
      digitalWrite( ouputPin, HIGH );
      delay(10); // allow ringing to stop
      analogValue = analogRead( inputPin );
      digitalWrite( ouputPin, LOW );
    }
  //  Serial.print("Return voltage across 10k= ");
    returnVoltage = analogValue *ArduinoResolution;
   // Serial.print(returnVoltage);                //across 10k resistor.
    //Serial.println(" volts");
 
    //Serial.print("Return voltage across probe ");
    returnVoltageAcrossProbe = 5 - analogValue *ArduinoResolution;
    //Serial.print(returnVoltageAcrossProbe);                //across probe.
    //Serial.println(" volts");
 
 
 
    //Serial.print("That works out to a resistance of ");
    resistance = ((5.00 * resistorValue) / returnVoltage) - resistorValue;        //
    //Serial.print(resistance);
    //Serial.println(" Ohms.");
 
    //Serial.print("Which works out to a conductance of ");
    Siemens = 1.0/(resistance/1000000);
    //Serial.print(Siemens);
    //Serial.println(" microSiemens.");
 
    //Serial.print("Which works out to a conductivity of ");
    conductivity = (Siemens*l*0.001)/A;
    //Serial.print(conductivity);
    //Serial.println(" milliSiemens/m");
 
    //Serial.print("Total Dissolved Solids are on the order of ");
    //TDS = 500 * (Siemens/1000);
    TDS = 0.67 * conductivity * 10;
    //Serial.print(TDS);
    //Serial.println(" PPM.");
 
    //if (returnVoltage>4.9) Serial.println("Are you sure this isn't metal?");
    delay(2000);
  }
 
  return conductivity;
}


float ultrasonicFunction(){

 
float t = 0, h = 0, hp = 0, th = 8; //th = tank height

// Transmitting pulse
digitalWrite(trig, LOW);
delayMicroseconds(2);
digitalWrite(trig, HIGH);
delayMicroseconds(10);
digitalWrite(trig, LOW);

// Waiting for pulse
t = pulseIn(echo, HIGH);

h = t / 57;
//Serial.print("h=");
//Serial.print(h);
//Serial.print("\n");

h = h - 6; // offset correction (Distance from Max Level)
//Uncomment if you want how much is full in %
h = 12 - h; // water height, 0 - 50 cm (Tank Height)

//hp = 100 * ((th-h)/th)
//hp = 100 * (h/th); // How much is filled in %, 0-100 %

// Sending to computer
//Serial.print(hp+"%");
// Serial.print(" cm\n");

//delay(1000);
//}
 
 return h;
}
