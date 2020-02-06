// Test program for HM-10 Bluetooth Module Echo BT to Serial and Serial to BT

// include the library code
#include <Wire.h> 
#include <SoftwareSerial.h> //for the bluetooth communications
SoftwareSerial BTSerial(8,9);

byte comdata;

void setup()
 {
     Serial.begin(115200);
     Serial.println("BLE Test");
    
    BTSerial.begin(115200);
  	delay(200);
    
  pinMode(8, INPUT);
  pinMode(9, OUTPUT);

 }

void loop()
 {
CkSerialCmds();
 
 }

 void CkSerialCmds(){
  char incoming_char="";
        
       		if (Serial.available() > 0 || BTSerial.available() > 0)
       	 {   if (Serial.available()==0) {incoming_char = BTSerial.read();}
       	   	 else {incoming_char = Serial.read();}
             Serial.print ("Incoming Char=");Serial.println(incoming_char);
      }
 }



 
