/**
 * Author:A.E.Neumann
 * version: 1.0
 * date: 9-23-2018
 * * Programming Arduino Nano
    Arduino Nano
    Older Nano Boards - ATMega 328 (Old Bootloader)
    Newer Nano Boards - ATMega 328
    Programmer AVRISP MKII
 */

/*********************************************************/
// variables
int TimeCounter;
int ncnt;
int LEDStateCounter;
int LEDBluState;
int const LedOnTime = 50;
//int const interruptPin = 2;
int SerialOn = 0;

int LED=13;  			//portb.5 'for testing 'Digital pin 13 - On board LED

/*********************************************************/

void setup() {

	
	pinMode(LED,OUTPUT);

  	pinMode(LED_BUILTIN,OUTPUT);//We use the led on pin 13 to indicate when Arduino is A sleep
   digitalWrite(LED_BUILTIN,HIGH);//turning LED on
   LEDStateCounter=1; 
    SetSerial115();

}

void loop() {
 
 //Sequence the On Board LED
 digitalWrite(LED_BUILTIN,HIGH);
 delay(250);
 digitalWrite(LED_BUILTIN,LOW);
 delay(250);
 Serial.println ("250 Off");
 digitalWrite(LED_BUILTIN,HIGH);
 delay(500);
 digitalWrite(LED_BUILTIN,LOW);
 delay(500);
 Serial.println ("500 Off");
 digitalWrite(LED_BUILTIN,HIGH);
 delay(1000);
 digitalWrite(LED_BUILTIN,LOW);
 delay(1000);
 Serial.println ("1000 Off");
 digitalWrite(LED_BUILTIN,HIGH);
 delay(1500);
 digitalWrite(LED_BUILTIN,LOW);
 delay(1500);
 Serial.println ("1500 Off");
 digitalWrite(LED_BUILTIN,HIGH);
 delay(2000);
 digitalWrite(LED_BUILTIN,LOW);
 delay(2000);
 Serial.println ("2000 Off");
}
/*********************************************************/
void SetSerial115(){
  if (SerialOn != 115)
    {
      Serial.begin(115200);
        while (!Serial) {
          // will pause until serial console opens
          delay(10);}
     }
  SerialOn = 115;
}
