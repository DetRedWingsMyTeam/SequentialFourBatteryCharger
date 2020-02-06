
/* Test program for the INA219 Current Sensing module along with LCD display 
 *  10-17-2018 by A.E.Nuemann
 */

#include <Wire.h>
#include <Adafruit_INA219.h>
Adafruit_INA219 ina219;
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27,20,4);  // set the LCD address to 0x27 
/*********************************************************/

void setup(void) 
{
 // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);

 Serial.begin(115200);
  while (!Serial) {
      // will pause Zero, Leonardo, etc until serial console opens
      delay(1);}

  delay (100);
  
  lcd.init();  //initialize the lcd
  lcd.backlight();  //open the backlight 
     
  lcd.setCursor ( 0, 0 );            // go to the top left corner
  lcd.print ("Main Setup"); // write this string on the top row
  
  Serial.println("Hello!");
  
  // Initialize the INA219.
  // By default the initialization will use the largest range (32V, 2A).  However
  // you can call a setCalibration function to change this range (see comments).
  ina219.begin();
  // To use a slightly lower 32V, 1A range (higher precision on amps):
  //ina219.setCalibration_32V_1A();
  // Or to use a lower 16V, 400mA range (higher precision on volts and amps):
  //ina219.setCalibration_16V_400mA();

  //Serial.println("Measuring voltage and current with INA219 ...");
}

void loop(void) 
{
	digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)

  float shuntvoltage = 0;
  float busvoltage = 0;
  float current_mA = 0;
  float loadvoltage = 0;
  float power_mW = 0;

  shuntvoltage = ina219.getShuntVoltage_mV();
  busvoltage = ina219.getBusVoltage_V();
  current_mA = ina219.getCurrent_mA();
  power_mW = ina219.getPower_mW();
  loadvoltage = busvoltage + (shuntvoltage / 1000);

 lcd.setCursor ( 0, 0 );            // go to the top left corner
 lcd.print("Shunt mV:           ");
 lcd.setCursor ( 0,1 );  
 lcd.print("Load V:             ");  
 lcd.setCursor ( 0,2 );           
 lcd.print("Current mA:         ");
 lcd.setCursor ( 0, 3 );  
 lcd.print("Power:              "); 
 
 lcd.setCursor ( 12,0 );  
 lcd.print(shuntvoltage);
 lcd.setCursor ( 12,1 ); 
 lcd.print (loadvoltage); // write this string on the top row
 
  lcd.setCursor ( 12, 2 ); 
  lcd.print(current_mA);
  lcd.setCursor ( 12,3 );
  lcd.print (power_mW);   

 
  Serial.print("Bus Voltage:   "); Serial.print(busvoltage); Serial.println(" V");
  Serial.print("Shunt Voltage: "); Serial.print(shuntvoltage); Serial.println(" mV");
  Serial.print("Load Voltage:  "); Serial.print(loadvoltage); Serial.println(" V");
  Serial.print("Current:       "); Serial.print(current_mA); Serial.println(" mA");
  Serial.print("Power:         "); Serial.print(power_mW); Serial.println(" mW");
  Serial.println("");

  delay(1000);
  digitalWrite(LED_BUILTIN, LOW);   // turn the LED on (HIGH is the voltage level)
  delay(1000);
}
