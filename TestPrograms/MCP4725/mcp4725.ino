/**************************************************************************/
/*! Basic test of MCP4725 DAC - 12 bit
  
   
*/
/**************************************************************************/
#include <Wire.h>
#include <Adafruit_MCP4725.h>
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27,20,4);

#define BUFFER_SIZE 20
#define VERSION     "1.0"

// Buffer for incoming data
char serial_buffer[BUFFER_SIZE];
int buffer_position;
bool stopSerial=false;
int SerialOn = 0;

uint32_t voltage;
uint32_t vptone;
float VDisplay;
float DACVoltage;

Adafruit_MCP4725 dac; // 0 to 4095

/********************** SETUP *************************/
void setup(void) {
  buffer_position = 0;	// initialize serial comm input buffer for setting time/cals

  lcd.init();  							//initialize the lcd
  lcd.backlight();  					//open the backlight 
  lcd.setCursor ( 0, 0 );            	// go to the top left corner
  lcd.print ("Initialization"); 		// write this string on the top row
  
  pinMode(LED_BUILTIN,OUTPUT);			//We use the led on pin 13 to indecate when Arduino is asleep
  digitalWrite(LED_BUILTIN,HIGH);		//turn LED on

  // For MCP4725A1 the address is 0x62 (default) or 0x63 (ADDR pin tied to VCC)
  // For MCP4725A0 the address is 0x60 or 0x61
  // For MCP4725A2 the address is 0x64 or 0x65
  dac.begin(0x62);
  
 SetSerial115();

  DACVoltage = 0.00122; //for 5 volt output

 // dac is 0 to 5 volts or 0.00122 volts/bit, 0.1 v = 82 bits
 vptone = 82;
 voltage = 2047; 
 dac.setVoltage (voltage, false); // init dac to midpoint 
}

/********************** MAIN *************************/
void loop(void) {
   /* uint32_t counter;
    // Run through the full 12-bit scale for a triangle wave
    for (counter = 0; counter < 4095; counter++)
    {
      dac.setVoltage(counter, false); // write to dac but not to eeprom in the dac
      delay(5);
    }
    for (counter = 4095; counter > 0; counter--)
    {
      dac.setVoltage(counter, false);
      delay(5);
    }
    */
    Serial.println ("Ready for DAC Command");
    SetDACSerial();
}

void SetDACSerial() // 
{
	char incoming_char="";
  	String time_string="";
  	
	do{ // Loop here Setting DAC Voltage
   
		// Wait for incoming data on serial port
	  	if (Serial.available() > 0) 
	    {   // Read the incoming character
	        incoming_char = Serial.read();
   
         // do the command
            if(incoming_char == 'u' )  // Raise voltage 0.1v
              { 
                voltage=voltage+vptone;
              if (voltage >= 4096) 
                 {voltage=4095;}
                dac.setVoltage(voltage, false);
                PrintDACVoltage();
              }
            else if(incoming_char== 'd') // Lower voltage 0.1v
              { 
              voltage=voltage-vptone;
              if (voltage >= 4096) 
                  {voltage=0;}
              dac.setVoltage(voltage, false);
              PrintDACVoltage();
          	  }
      	    else if(incoming_char == 'x') // Exit
             { stopSerial=true;
            
             }	
	    }
	}while (stopSerial==false);
} 

void PrintDACVoltage (){

  Serial.print ("DAC V =");
  VDisplay = voltage * DACVoltage; //0-5 volts at 4095 points Nominal is 0.00122 v/b
  Serial.println (VDisplay, 5); 
}
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
