#include <LiquidCrystal.h>
LiquidCrystal lcd(12,11,5,4,3,2);

 

 const int switchPin = 6; 
 int switchState = 0;
 int condVal;
 
 void setup() {
   Serial.begin(9600); //establishes a connection to the arduino
   lcd.begin(16,2); //instructs the lcd to begin displaying
   pinMode(switchPin, INPUT); //sets analog 6 as an input pin
   
 }
 
 void loop() {  //everything between these curly braces will loop
   condVal = analogRead(A0);
   float voltage = condVal*(5.0/1023.0); //calculation of relative conductivity
   lcd.setCursor(0,0); //instructs LCD to go to the first line, first space
   lcd.print("Rel Conductance"); //instructs LCD to display "Rel Conductance" beginning on the first line, first space of the LCD display.
   lcd.setCursor(0,1); //instructs LCD display to go to the second line, first space.
   lcd.print(voltage); //display the relative conductivity from the probe on the second line, first space
   delay(50); //delay before looping
 }
