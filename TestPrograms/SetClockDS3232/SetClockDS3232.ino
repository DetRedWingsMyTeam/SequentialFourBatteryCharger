// Set the clock on a DS3232 Module - 

// Set the date and time by entering the following on the Arduino
// serial monitor:
//  year,month,day,hour,minute,second,
//
// Where
//  year can be two or four digits,
//  month is 1-12,
//  day is 1-31,
//  hour is 0-23, and
//  minute and second are 0-59.
//
// Entering the final comma delimiter (after "second") will avoid a
// one-second timeout and will allow the RTC to be set more accurately.

#include <Wire.h>
#include <DS3232RTC.h> // used for Alarm functions and time/date
#define interruptPin 2 //Pin we are going to use to wake up the Arduino
//for bluetooth
#include <SoftwareSerial.h>
SoftwareSerial BTSerial(8,9); //pin 8 is TX from BT module

#define BUFFER_SIZE 20
#define VERSION     "1.0"
tmElements_t clock;
int lastseconds;

// pin definitions
byte alarmInput=2;
byte SQW_PIN=2;   // connect this pin to DS3231 INT/SQW pin

char daysOfTheWeek[8][12] = {"null", "Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
char daysOfTheWeekShrt[8][5] = {"nul", "Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"};

// Buffer for incoming data
char serial_buffer[BUFFER_SIZE];
int buffer_position;
bool stopSerialInput=false;
char incoming_char="";
int SerialOn = 0;

const char St1 = "  aReady to Date/Time Set";
const char St2 = "  bReady to Date/Time Set";
const char St3 = "Var Usage Reduction";
/****************** SETUP ******************************/
void setup () {

	// initialize digital pin LED_BUILTIN as an output.
	pinMode(LED_BUILTIN, OUTPUT);
	
	pinMode(8, INPUT);
     pinMode(9, OUTPUT);

	SetSerial115();	// init serial port to 115,200 baud
	BTSerial.begin(115200);
	
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

void loop()
{
      CheckForSerialInput();
    
      RTC.read(clock); 	// read the DS3232 and return all values in structure clock
      if(lastseconds != clock.Second)
      {
        lastseconds = clock.Second;
        SerialPrintTime();
      }
}

void CheckForSerialInput(){
  if (Serial.available() > 0 || BTSerial.available() > 0)  
  
    {   if (Serial.available()==0) 
      {incoming_char = BTSerial.read();}
       else {incoming_char = Serial.read();}
      Serial.print(incoming_char);
      if(incoming_char=='r' )  //set time
        Serial.println ("  Ready to Date/Time Set");
       Serial.println (St1);
       Serial.println (St1);
      Serial.println (St2);
       Serial.println (St2);
       Serial.println (St3);
       Serial.println (St3);
       stopSerialInput=false; //set so we loop waiting for date/time chars
        GetandSetTime();
    }   
}

void GetandSetTime(){
   
    time_t t; // time structure used to set DS3232 clock

do {
    // check for input to set the RTC, minimum length is 12, i.e. yy,m,d,h,m,s
    if (Serial.available() >= 12) {
        // note that the clockElements_t Year member is an offset from 1970,
        // but the RTC wants the last two digits of the calendar year.
        // use the convenience macros from the Time Library to do the conversions.
        int y = Serial.parseInt(); //returns the first valid (long) integer number from the serial buffer. Comma causes skip to next
        if (y >= 100 && y < 1000){
            Serial.println("Error: Year must be two digits or four digits!");
            stopSerialInput=true;}
        else {
            if (y >= 1000)
                clock.Year = CalendarYrToTm(y);
            else    // (y < 100)
                clock.Year = y2kYearToTm(y);
            clock.Month = Serial.parseInt();
            clock.Day = Serial.parseInt();
            clock.Hour = Serial.parseInt();
            clock.Minute = Serial.parseInt();
            clock.Second = Serial.parseInt();
            t = makeTime(clock);   // fill structure t with date/time from vars in clock structure
            RTC.set(t);         // set the DS3232 date/time 
            stopSerialInput=true;
            }
      }
  }while (stopSerialInput==false);

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
