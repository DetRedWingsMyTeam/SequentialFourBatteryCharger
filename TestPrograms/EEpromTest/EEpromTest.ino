// New pgm based on previous otherTest of writing/reading float values to EErprom
// results at bottom of code

#include "Arduino.h"
#include <EEPROM.h>

int eLocation = 0;
float ePutValue = 01.45;
float eGetValue;
float BattV;
float SolarV;
float RelayV;
float CalFactor;
String CalFactorString;
float RetrvdCalFactor;

void setup() {
 Serial.begin(57600);
   
  while (!Serial) {
      // will pause Zero, Leonardo, etc until serial console opens
      delay(1);}

   CalFactor = 0.00448;
    BattV = 21.287;

    //float to string to float test - 
    
    CalFactorString=String(CalFactor, 5); 
    RetrvdCalFactor = CalFactorString.toFloat();
    Serial.print("CalFactorString:");
    Serial.print(CalFactorString);
    Serial.print("  RetrvdCalFactor:");
    Serial.println(RetrvdCalFactor,5);

    
  //to do the get method--->EEPROM.get(address, data)
  
  // get 6 float values in a for loop
  for (int i = 1; i < 7; i++) {
    Serial.print("get value from addr:");
    Serial.print(eLocation);
    Serial.print(" value=");
    EEPROM.get(eLocation, eGetValue);
    Serial.println(eGetValue, 5);
    eLocation = eLocation + sizeof(eGetValue);
    delay(200);
    }
  Serial.println("***********Writing*************");
  //write 4 float values in a for loop
  for (int i = 0; i < 4; i++) {
    Serial.print("Write Value:");
    delay(100);              //pause at least 3.3ms to write data.
    //One simple call for put, with the address first and the object second.
    EEPROM.put(i * sizeof(ePutValue), ePutValue);
    Serial.print(ePutValue);
    Serial.print(" To address:");
    Serial.println(i * sizeof(ePutValue));
    ePutValue = ePutValue + 1.12;
    eLocation = i * sizeof(ePutValue);
  }
  //write cal factor for A/D 1 
    Serial.print("Write A/D 1 Cal :");
    Serial.print(CalFactor, 5);
    //Serial.print(" adj to fit:");
    //ePutValue=(CalFactor * 1000); //adj to fit in eeprom
    ePutValue=CalFactor;
    delay(100);              //pause at least 3.3ms to write data.
    //One simple call for put, with the address first and the object second.
    eLocation += sizeof(ePutValue);
    EEPROM.put(eLocation, ePutValue);
    Serial.print("  writing ");
    Serial.print(ePutValue, 5);
    Serial.print(" To address:");
    Serial.println(eLocation);
    
  //write batt v  
    Serial.print("Write Batt V :");
    ePutValue=BattV;
    delay(100);              //pause at least 3.3ms to write data.
    //One simple call for put, with the address first and the object second.
    eLocation += sizeof(ePutValue);
    EEPROM.put(eLocation, ePutValue);
    Serial.print(ePutValue, 5);
    Serial.print(" To address:");
    Serial.println(eLocation);
    
  Serial.println("**************Final Readout***************:");
  for (int i = 0; i < 6; i++) {
    Serial.print("get method from Addr:");
    Serial.print(i * sizeof(eGetValue));
    Serial.print(" value=");
    EEPROM.get(i * sizeof(eGetValue), eGetValue);
    Serial.println(eGetValue,5);
    
  }
} //end setup()

void loop() {
  // put your main code here, to run repeatedly:
}

/* example results

CalFactorString:0.00448  RetrvdCalFactor:0.00448
get value from addr:0 value=1.45000
get value from addr:4 value=2.57000
get value from addr:8 value=3.69000
get value from addr:12 value=4.81000
get value from addr:16 value=0.00448
get value from addr:20 value=21.28700
***********Writing*************
Write Value:1.45 To address:0
Write Value:2.57 To address:4
Write Value:3.69 To address:8
Write Value:4.81 To address:12
Write A/D 1 Cal :0.00448  writing 0.00448 To address:16
Write Batt V :21.28700 To address:20
**************Final Readout***************:
get method from Addr:0 value=1.45000
get method from Addr:4 value=2.57000
get method from Addr:8 value=3.69000
get method from Addr:12 value=4.81000
get method from Addr:16 value=0.00448
get method from Addr:20 value=21.28700
*/
