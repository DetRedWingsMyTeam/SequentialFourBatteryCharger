// Date and time functions using a DS3231 RTC connected via I2C and Wire lib
#include <Wire.h>
#include <DS3232RTC.h> // used for Alarm functions and time/date
#define interruptPin 2 //Pin we are going to use to wake up the Arduino

#define BUFFER_SIZE 20
#define VERSION     "1.0"
tmElements_t clock;

// pin definitions
byte alarmInput=2;
byte SQW_PIN=2;   // connect this pin to DS3231 INT/SQW pin

char daysOfTheWeek[8][12] = {"null", "Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
char daysOfTheWeekShrt[8][5] = {"nul", "Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"};

int SerialOn = 0;


/****************** SETUP ******************************/
void setup () {

	// initialize digital pin LED_BUILTIN as an output.
	pinMode(LED_BUILTIN, OUTPUT);
	
	SetSerial115();	// init serial port to 115,200 baud
	
	
	delay (100);
	
	// initialize the alarms to known values, clear the alarm flags, clear the alarm interrupt flags
	// RTC.setAlarm(alarmType, seconds, minutes, hours, dayOrDate);
	RTC.setAlarm(ALM1_MATCH_DATE, 0, 0, 0, 1);
	RTC.setAlarm(ALM2_MATCH_DATE, 0, 0, 0, 1);
	RTC.alarm(ALARM_1);
	RTC.alarm(ALARM_2);
	RTC.alarmInterrupt(ALARM_1, false);
	RTC.alarmInterrupt(ALARM_2, false);
	 
	RTC.squareWave(SQWAVE_NONE);			// config INT/SQW pin for "interrupt" (disable square wave output)
	RTC.alarmInterrupt(ALARM_1, true);		// enable interrupt output for Alarm 1
	
	pinMode(SQW_PIN, INPUT_PULLUP);		// config interrupt on the falling edge from the SQW pin
	pinMode(interruptPin,INPUT_PULLUP);	//Set pin d2 to input using the built-in pullup resistor
	
}

/********************** MAIN *************************/
void loop () {
    	RTC.read(clock);
	
	SerialPrintTime();
        
    	delay(1000);
}


void printDigitsserial(int digits){
	// utility function for clock display: prints preceding colon and leading 0
	Serial.print(':');
	if(digits < 10){
	    Serial.print('0');}
	Serial.print(digits);
}

void SerialPrintTime(){
	// Send date over serial connection
	//Serial.print("Date: ");
	Serial.print(clock.Month, DEC);
	Serial.print("/");
	Serial.print(clock.Day, DEC);
	Serial.print("/");
	Serial.print(clock.Year+1970, DEC);

	// Send Day-of-Week and time
	Serial.print("  ");
  Serial.print(daysOfTheWeek[clock.Wday]);
  Serial.print("  ");
	Serial.print(clock.Hour, DEC);
	printDigitsserial(clock.Minute);	// adj to 00 if only 0 add :
	printDigitsserial(clock.Second);
	
	// print temperature
	Serial.print("   Temp: ");
	//Serial.print(RTC.getTemp());
	Serial.print(RTC.temperature()/4);
	Serial.println(" C");
	Serial.println("--------------------------------");
}

void SerialPrintHrsMinsSecs(){
  Serial.print(clock.Hour, DEC);
  printDigitsserial(clock.Minute);  // adj to 00 if only 0 add :
  printDigitsserial(clock.Second);
  Serial.println();
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
