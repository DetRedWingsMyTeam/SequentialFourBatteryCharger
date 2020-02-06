/** Author:C.S.Robbins
 * version: 1.18.0 Arduino Nano
 * date: 2/12/2019 to 5/25/2019, 10/23/2019, 11/17/2019, 12/11/2019, 12/23/2019, 1/25/2020
 * 7/18/19 - added ext bluetooth module, changed ext data acq to 1 min interval, 8/25/19 chged to 2 amps
 * Programming Arduino Nano
    Arduino Nano
    ATMega 328 for Nano w/Bluetooth else ATMega 328 (Old Bootloader)
    Programmer AVRISP MKII
    Data files from SerialBluetooth Terminal are stored at Android/Data/de.kai_morich.serial_bluetooth_terminal/files
 * Description: 
 * 4 Port Intelligent 4+ Stage Lead Acid Battery Charger
 Battery Types - Flooded = Refillable, SLA = Maintenance Free, AGM = Absorbent Glass Mat, GEL = Gel Cell Type
 Stage 1 - Bulk Constant CurrentCharge = Constant Current while the Voltage Increases
 	       Max amps = 0.25C or 1/4 of the rated AH or max output of power supply typ 2-3A
            even for a 20 AH battery, .25C is 5 amps. Approx conversion from CCA to AH is CCA/7.25, ex; 180 CCA = 25 AH, .25C = 6.4A
 	       Setpt is 14.4v (2.40/cell) then Stage 2
 	       or if 240 minutes elapsed move to Stage 2
 Stage 2 - Absorption Charge = Two Stage Constant Voltage Stages 2, 3
           High Absorption Charge 14.4v (14.2v) until current =< 0.05C (ah), 70-80% or 0.25a
           		Max 8 hours in High Absorption Stage 2A
           		SLA=14.4, AGM=14.6, Gel=14.1
 Stage 3 - Low Absorption Charge 13.65v (2.275/cell) until < 0.1a
 Stage 4 - Float Charge = Pulsed or Constant 13.4 (SLA/VRLA), 13.5-13.8 (AGM Nom 13.6), 13.8 (Gel)
 Stage 5 - Battery Capacity Check - Open Circuit V =12.73 or greater = 100%
 Stage 6 - Ramp to Float Charge
 Stage 7 - Storage Mode 13.0 to 13.2V, Battery Volts => 12.7 (20321)
  
 SLA State of Charge 100% 12.70+, 75% 12.40, 50% 12.20
 AGM State of Charge 100% 12.80+, 75% 12.60, 50% 12.30
 
SOC	12V - Open Circuit for 4 hours prior to measuring or until V stabilizes
100	12.73-12.78  90 12.62-12.66  80	12.5-12.54  70 12.37-12.42  60 12.24-12.30
50	12.1-12.18  40 11.96-12.06  30 11.81-11.94  20 11.66  1011.51
 */

// include the library code
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27,20,4);
#include <DS3232RTC.h> // used for Alarm functions and now time/date
#include <Adafruit_MCP4725.h>	//12 bit DAC
#include <Adafruit_ADS1015.h> //4 ch 16 bit A/D
#include <Adafruit_INA219.h> //current sensor module
#include <AT24CX.h>  //4k eeprom in clock module
AT24CX mem;

#include <SoftwareSerial.h> //for the bluetooth communications
SoftwareSerial BTSerial(8,9);

Adafruit_INA219 ina219;			//Current Sensor Module 1 - Internal
Adafruit_INA219 ina219_Ext(0x41); //Current Sensor Module 2 - For External 
Adafruit_ADS1115 ads1115; //A/D module 16 bit 4 chan Range: +/- 6.144V 1 bit = 0.1875mV/ADS1115
Adafruit_MCP4725 dac; // Digital to Analog DAC 0 to 4095, 0 to 5 volts

/*************** VARIABLES ***************************/
const float Version=1.17;

// Arrays are index 0 but we use 1 as start so need array dim +1
#define BUFFER_SIZE 10  //serial port
// Buffer for incoming data
char serial_buffer[BUFFER_SIZE];
int buffer_position;
bool pauseflag=false;

tmElements_t clock; //Real Time Clock for timing

//Front Panel Battery Select Pushbuttons - PB0-PB3  dig 8,9,10,11
const byte BattSelectSw1=8;  
const byte BattSelectSw2=9; 
const byte BattSelectSw3=10;  
const byte BattSelectSw4=11;
byte BattSelectSw[5]={0,8,9,10,11}; //keep track of which sw was pushed 

//Front Panel Switches
const byte MonitorChargeSelectSwitch=7;  //portd.7 'Digital pin 7 as Input
const byte PwrSupplyChargerSwitch=6;     //portd.6 'Digital pin 6 as Input
const byte BatteryTypeSwitch=12;        //portb.4 'Digital pin 12 as Input
const byte LED=13;                 //portb.5 'for testing 'Digital pin 13 - On board LED

// battery relays port pins
const byte BattRly1=5;  //dig pins PD2-PD5, Dig 2 to Dig5
const byte BattRly2=4;
const byte BattRly3=3;
const byte BattRly4=2;
byte LastBattRly=0;

//Front panel LEDs port pins are A0-A3 which are 14 to 17
byte LEDs[9]={0,A0,A1,A2,A3,0,0,0,0}; //index 6-9 used for 1sec flash on/off flag

//Charging Variables
int BattVolts[5]; // A0 - A3
bool BattSelectActive[5]={false,false,false,false,false};
int ChargeStageTimeElapsedMins=0;
int Stage1ChargeTime=0;
byte LastBatteryActive = 0;   //set by battery selecte pb sws
bool ActiveCharging=false;
byte ActiveBatt=0;           //which batt being charged
byte BatteryChargeStage=0;       //stages 1 to 5
bool Stage3TransitionFlag=false;
bool BatteryChargeMode=false;     //enabled if Charge/Monitor Sw is in Charge position
byte BatteryType[5]={0,0,0,0,0}; //0=SLA, 1=AGM, 2=GEL, 3=Flooded
bool EnableBattTypeSet=false; //allow setting batt type after batt select sw pushed
bool BatteryTypeSwClosed=false; //enable inc'g to next batt type
bool BatteryCharge4Stages=false; // normal only 3 stages, CC, CV, Float, else CC, CV1, CV2 13.65v, Float
float BattCalFactor = 0.000625;

//AGM Absorption Stage2 14.3 volts +/- 0.1 volts 
//AGM Float Stage4 13.3 +/- 0.1 volts 
//AGM Conditioning Charge at 70F = 15.58 volts
// battery charging constants for SLA - Sealed Lead Acid
const int AGM100PercentChargedVolts=20480; //12.80/.000625 = 21840 //0-20.48 volts at 32768 points Nominal is 0.000625/bit
const int SLA100PercentChargedVolts=20320; //12.700 100% SOC
const int SLA90PercentChargedVolts=20160; //12.600 90% SOC
const int Stage1TimeThreshold = 240;
const int Stage1SLAVoltageThreshold = 23040;  
const int Stage2SLAVoltageThreshold = 23040;  //nom 14.40
const int Stage3SLAVoltageThreshold = 21840;	//nom 13.65  
const int Stage4SLAVoltageThreshold = 21440;  //nom 13.40 
int Stage1VThreshold= Stage1SLAVoltageThreshold;   //init to SLA
int Stage2VThreshold= Stage2SLAVoltageThreshold;   //init to SLA
int Stage3VThreshold= Stage3SLAVoltageThreshold;   //init to SLA
int Stage4VThreshold= Stage4SLAVoltageThreshold;   //init to SLA

// Current Sensor data variables
float shuntvoltageINA219 = 0;
float busvoltageINA219 = 0;
float current_mAINA219 = 0;
float current_AmpsINA219 = 0;
float loadvoltageINA219 = 0;
float power_mWINA219 = 0;

// Modes Variables 
bool PowerSupplyMode=false;
bool DataAcqMode=false;
bool DataAcqExternal=false;
bool FastDataAcqExternal=false;
bool FastDataAcqInternal=false;
bool DataAcq10SecMode=false;

// Digital to Analog Converter Variables
int DACvoltage = 2047; //Midpoint of output, DA converter, dac is 0 to 5 volts/4095 bits or 0.00122 volts/bit, 0.1 v = 82 bits
//int DACVstep=42; // 0.05 volts
int DACVstep=21;   // 0.025 volts
int DACSmallStep=5; // 0.00611 volts

//Serial/LCD Communications Variables
char daysOfTheWeek[8][12] = {"null", "Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
char daysOfTheWeekShrt[8][5] = {"nul", "Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"};
byte SerialOn = 0;
bool stopSerialInput=false;

// Timing Variables
bool OneSecElapsed=false;
bool TenSecElapsed=false;
bool OneMinElapsed=false;
byte CurrentSeconds = 0;
byte CurrentMinutes = 0;
byte TenSecondTimer = 10;
bool TenSecDataFlag = false;
bool OneMinDataFlag = false;

//EEProm 
const byte EEPromDataStartAddr = 4;
unsigned int LastEEPromAddr=EEPromDataStartAddr;

//Memory check
uint8_t * heapptr, * stackptr;
uint8_t dheapptr, dstackptr;

/****************** SETUP ******************************/
void setup() {
  pinMode(BattRly1,OUTPUT);		//init relays
	pinMode(BattRly2,OUTPUT);
	pinMode(BattRly3,OUTPUT);
	pinMode(BattRly4,OUTPUT);
	digitalWrite(BattRly1, HIGH);  //active low
  digitalWrite(BattRly2, HIGH);
  digitalWrite(BattRly3, HIGH);
  digitalWrite(BattRly4, HIGH);
  
	pinMode(A0,OUTPUT);	//set analog pins as outputs for LEDs
	pinMode(A1,OUTPUT);
	pinMode(A2,OUTPUT);
	pinMode(A3,OUTPUT);
  digitalWrite(A0,LOW);	// active high so turn off
  digitalWrite(A1,LOW);	
  digitalWrite(A2,LOW);
  digitalWrite(A3,LOW);

	pinMode (BattSelectSw1, INPUT_PULLUP); //setup Batt Select pushbutton switches
  pinMode (BattSelectSw2, INPUT_PULLUP);
  pinMode (BattSelectSw3, INPUT_PULLUP);
  pinMode (BattSelectSw4, INPUT_PULLUP);
  
    //need to do it this way
	pinMode(PwrSupplyChargerSwitch, INPUT_PULLUP);
	pinMode(MonitorChargeSelectSwitch,INPUT_PULLUP);
  pinMode(BatteryTypeSwitch,INPUT_PULLUP);
	pinMode(LED,OUTPUT);
  
  pinMode(8, INPUT); //BT Serial Rx from BT module
  pinMode(9, OUTPUT);

  buffer_position = 0;	// initialize serial comm input buffer for setting time/cals

  lcd.init();  							//initialize the lcd
  lcd.backlight();  					//turn on the backlight 
  lcd.setCursor ( 0, 0 );            	// go to the top left corner
  lcd.print ("Init"); 		// write this string on the top row
  
  pinMode(LED_BUILTIN,OUTPUT);			//We use the led on pin 13 for diagnostics
  digitalWrite(LED_BUILTIN,HIGH);		//turn LED on
  
  // ads1115 16bit A/D init
 // set 16 bit A/D to GAIN_ONE (for an input range of +/-4.096V) range=20.48/32768=0.000625 v/bit
  ads1115.setGain(GAIN_ONE);     // 1x gain   +/- 4.096V, 8.192v range, 65536, 4.096/32768= 1 bit = .000125
  ads1115.begin();
  
  // For MCP4725A1 the address is 0x62 (default) or 0x63 (ADDR pin tied to VCC)
  dac.begin(0x62);
  dac.setVoltage (DACvoltage, false); // init dac to midpoint (, false = do not write to DAC's internal eeprom)

 // Initialize the INA219. By default the initialization will use the largest range (32V, 2A).  
  ina219.begin();		// Initialize first board (default address 0x40)
  ina219_Ext.begin();  // Initialize second board with the address 0x41
  
   SetSerial115();
   BTSerial.begin(115200);
  	delay(200);
  //SetSerial9600();

  Serial.print(F("freeMemory()="));Serial.println(freeMemory()); 

}

/********************** MAIN *************************/
void loop() {
    
    checkPwrSupplyChargerSwitch(); // check Charger/Power Supply sw
    
    checkChargeOrMonitorSelectSwitch(); // check Charge/Monitor sw
    
    checkBatterySelectSwitches(); // check if wants to select a batt to charge or monitor
    
    checkBattTypeSwitch(); // if selected a batt to add, see if wants to set/change batt type
 	
    read16bitAD(); readINA219CurrentSensor(); //Read for Charge or Power Supply
    
  if (BatteryChargeMode==true && LastBatteryActive !=0) // charge battery 
 	   { ChargeBattery(); }

	if (PowerSupplyMode==true)	// update display for power supply mode
      {	lcdPrintPowerSupplyVoltsAmps();
        Serial.print (F("Power Supply Volts="));
        SerialPrintPowerSupplyVoltsAmps();
      }
      
  else  // display system/batt volts or status every second to ser port if not doing data acq and not pwr supply
      { if (DataAcqMode==false && pauseflag==false)
        {  lcdPrintChargeMonitorStatus();
           if (TenSecDataFlag== false && OneMinDataFlag==false){SerialPrintChargeMonitorStatus();}
             }
        else
        {	//if not charging or power supply mode and no leds on and ext DA true then display ext v/i on lcd
				    if (PowerSupplyMode==false && BatteryChargeMode==false && LastBatteryActive==0 && DataAcqExternal==true)
					    {lcdPrintExtVI();} //if ext data acq then leave lcd as is so displays ext V and I
            else
              { if (DataAcqExternal!=true)  //must be charging so show charge info
					        {lcdPrintChargeMonitorStatus();}
			        }
         }
      }
    CkforSerialCommands();

    // Now Turn On/Off/Flash LEDs
      SetLEDSOnOffFlash(); 
    
    do {CheckIfOneSecondElapsed();    //wait one second before continuing
    }while (OneSecElapsed==false);

   //check_mem();
   //Serial.print ("Stack =");Serial.print(dstackptr);Serial.print ("  Heap=");Serial.println(dheapptr);
	    
     //1 sec int/ext data acq
    if (DataAcqMode==true && DataAcq10SecMode==false && (FastDataAcqExternal==true || FastDataAcqInternal==true))  // get ext/int data every sec
              {SerialSendAcqData();}
              
   //10 sec int/ext data acq
    if (TenSecElapsed==true) 
    {  TenSecElapsed=false;
       if (TenSecDataFlag== true & DataAcqMode==false){SerialPrintChargeMonitorStatus();}
       if (DataAcq10SecMode==true && DataAcqMode==true )
       {   SerialSendAcqData();
           if (PowerSupplyMode==false && BatteryChargeMode==false && LastBatteryActive==0 && DataAcqExternal==true)
           {   lcdPrintExtVI();} //if ext data acq then leave lcd as is so displays ext V and I
       }
    }
             
    //30 sec int/ext data acq

    // do one minute stuff
   	CheckIfOneMinuteElapsed();	
  	if (OneMinElapsed==true)
      {   OneMinElapsed=false; //now reset it
          if (OneMinDataFlag== true & DataAcqMode==false){SerialPrintChargeMonitorStatus();}
          // One Min Int/Ext Data Acq
          if (DataAcqMode==true && DataAcq10SecMode==false && FastDataAcqExternal==false && FastDataAcqInternal==false)	// if one min then send data 
			        {SerialandEEPromSendAcqData();}
	        if (BatteryChargeMode==true) 	//inc minutes counter for charging
		          {ChargeStageTimeElapsedMins=ChargeStageTimeElapsedMins+1;
		           if (BatteryChargeStage==1){Stage1ChargeTime=Stage1ChargeTime+1;}
		          }
	    }	

}
/********************** END OF MAIN *************************/

void ChargeBattery(){
    //ActiveBatt=1; //temp for now
     
     if (ActiveCharging==false) //start charging if not already
        { StartCharging();}
     
    if (BatteryChargeStage==1) //Stage1 then maintain constant current at 2.0 amps until V=14.4 volts then next stage
      {  
        if (BattVolts[ActiveBatt] < Stage1VThreshold && ChargeStageTimeElapsedMins < Stage1TimeThreshold) // continue in Stage1
              {   
                if (current_mAINA219 > 2000) // >2.0 amps lower v to get back to 2.0 amps, nom .014v/.086a
                    {AdjustDACforCurrent(2000);}
                else if (current_mAINA219 < 2000) // <2.0 amps raise v to get up to 2.0 amps
                    {AdjustDACforCurrent(2000);}
              }
         else  // was Stage1 now entering Stage2 V=>14.4 or 240 mins elapsed
              { ChargeStageTimeElapsedMins=0; BatteryChargeStage=2; 
                //min 30 mins in stage 2 but no more than 5 times Stage 1 time
                Stage1ChargeTime=5*Stage1ChargeTime; if(Stage1ChargeTime < 30){Stage1ChargeTime=30;} 
                SetSpecificDACVoltage(Stage2VThreshold);}
      }
    else if (BatteryChargeStage==2) //if Stage2 then maintain 14.4 volts until current < 250ma or 8 hours or 5xStage1 Time
      {   	if (current_mAINA219 <= 250 || ChargeStageTimeElapsedMins >= 480 || ChargeStageTimeElapsedMins >= Stage1ChargeTime) //if BattCurrent =< 250ma then exit and go to Stage3 if 4 stage charging enabled
					    {   ChargeStageTimeElapsedMins=0;
                  if (DataAcqMode==false) {Serial.println ("Stage 2 exit to Next Stage");} 
					        if (BatteryCharge4Stages==true) {	
						          BatteryChargeStage=3;
			              	//SetSpecificDACVoltage(Stage3VThreshold); 
						          Stage3TransitionFlag = true;}
          				else
          						{BatteryChargeStage=4; //going from 14.4 to 13.4 Stage 4
          						SetSpecificDACVoltage(Stage4VThreshold);}
					    }
	          else if (BattVolts[ActiveBatt] < Stage2VThreshold) //if Voltage < 14.4 volts then raise voltage
		            { DACVoltageSetToTarget(true, Stage2VThreshold);}
	          else if (BattVolts[ActiveBatt] > Stage2VThreshold) //if Voltage > 14.4 volts then lower voltage
		            { DACVoltageSetToTarget(false, Stage2VThreshold);}
       }      
    else if (BatteryChargeStage==3) //maintain 13.65 volts until current < 100ma but move to 13.65 slowly
      // 13.65 volts = 13.65/.000625 = 21840 //0-20.48 volts at 32768 points Nominal is 0.000625/bit
       {  if (Stage3TransitionFlag==true) //starts here
              { SetDACVoltageStep(false, DACSmallStep); //transition to 13.65v slowly to allow Batt V to stabilize 
      				  if (BattVolts[ActiveBatt] <= Stage3VThreshold) {Stage3TransitionFlag=false;}
              }
		       else if (current_mAINA219 <= 100 || ChargeStageTimeElapsedMins > 480) //need time for the battery V to stabilize at lower V
				      {if (ChargeStageTimeElapsedMins > 30 || Stage3TransitionFlag==false) //min 30 mins at 13.65 to see what happens
				          { if (DataAcqMode==false) {Serial.println ("Stage 3 exit to Next Stage");}
				            BatteryChargeStage=6; 
						        ChargeStageTimeElapsedMins=0;
                    SetSpecificDACVoltage(BattVolts[ActiveBatt]);
                    //DACVoltageSetToTarget(true,BattVolts[ActiveBatt]);
						        //SetSpecificDACVoltage(Stage4VThreshold);
						       }
				       }
		       else if (BattVolts[ActiveBatt] < Stage3VThreshold) //if Voltage < 13.65 volts then raise voltage
				       {  DACVoltageSetToTarget(true,Stage3VThreshold);}
		       else if (BattVolts[ActiveBatt] > Stage3VThreshold) //if Voltage > 13.65 volts then lower voltage
				       {  DACVoltageSetToTarget(false,Stage3VThreshold);}
       }
     //if Stage 4 float mode 13.4v for 60 mins then Stage 4 - check V for 30 mins if falling, notify SOC
     // 13.4 volts = 13.4/.000625 = 21840 //0-20.48 volts at 32768 points Nominal is 0.000625/bit
    else if (BatteryChargeStage==4) //float mode for 30 mins at 13.4v then see if another batt to charge
       {  if (ChargeStageTimeElapsedMins > 30)
	            //see if should charge another battery
	             { CheckNextActiveBattery(); ChargeStageTimeElapsedMins=0; Stage1ChargeTime=0;}
          else if (BattVolts[ActiveBatt] < Stage4VThreshold) //if Voltage < 13.4 volts then raise voltage
	             { DACVoltageSetToTarget(true,Stage4VThreshold);}
          else if (BattVolts[ActiveBatt] > Stage4VThreshold) //if Voltage > 13.4 volts then lower voltage
	             { DACVoltageSetToTarget(false,Stage4VThreshold);}
       }
  else if (BatteryChargeStage==5) //Monitor batt mode to check SOC 
  		{   if (ChargeStageTimeElapsedMins > 30) 
				      { CheckNextActiveBattery(); ChargeStageTimeElapsedMins=0; Stage1ChargeTime=0;}
				  else if (BattVolts[ActiveBatt] < SLA100PercentChargedVolts) 
              { BatteryChargeStage=6; //set to ramp to float mode to bring up to 13.4 to recharge to 100%
                TurnOnRelay(ActiveBatt);
                SetSpecificDACVoltage(BattVolts[ActiveBatt]);   
				      }
  		}
  else if (BatteryChargeStage==6) //Ramp up from Batt V to Stage 4 V then go to Stage 4 
      { if (BattVolts[ActiveBatt] >= Stage4VThreshold)
           { BatteryChargeStage=4; ChargeStageTimeElapsedMins=0;}
        else if (ChargeStageTimeElapsedMins > 30) //see if should charge another battery
           { CheckNextActiveBattery(); ChargeStageTimeElapsedMins=0; Stage1ChargeTime=0;}
        else if (current_mAINA219 <= 400)   //limit current while raising V to 13.4
           { SetDACVoltageStep(true, DACSmallStep);}
      }
}

void StartCharging() {
    if (LastBattRly==0) //no relays on so turn on 
        {     //SetSpecificDACVoltage(22000); //TEMP FOR DEBUGGING
              // set ActiveBatt to 1st battery selected 
              ActiveBatt = LastBatteryActive; 
              SetSpecificDACVoltage(BattVolts[ActiveBatt]); //start Batt Charge V = Batt V to minimize inrush/outrush
              TurnOnRelay(LastBatteryActive);
              ActiveCharging=true;
              Stage1ChargeTime=0;
              // if Batt Type then Set Thresholds for Charging Stages
      
              // see if Battery is already charged, yes if Voltage is => 12.70 volts SLA, 12.80 AGM
              if (BattVolts[ActiveBatt] >= SLA100PercentChargedVolts) 
                  {BatteryChargeStage=5;TurnOffRelay();} //just monitor as fully charged
              else if (BattVolts[ActiveBatt] >= SLA90PercentChargedVolts) 
                  {BatteryChargeStage=6;
                  SetSpecificDACVoltage(BattVolts[ActiveBatt]);} //set to ramp to float mode as already 90% charged
              else
                  {BatteryChargeStage=1;} //less than 90% charged so start charge process
              Serial.print (F("Battery Charging Started, Relay "));Serial.println(LastBatteryActive);
       }
}

void checkChargeOrMonitorSelectSwitch() {
	// chkswitch to see if wants Monitor or Charge Modes
	if(digitalRead(MonitorChargeSelectSwitch)==HIGH && PowerSupplyMode==false) // Auto Charge Mode
	{	if (BatteryChargeMode==false)	//mode change to charge mode 
			{ Serial.println (F("Battery Charging Mode On"));
			  BatteryChargeMode=true;
        lcd.clear();
        EnableBattTypeSet=false;  
	      DataAcqMode=false; //cancel any data acq when changing modes!
	    }
	} 
  else
  {	if(digitalRead(MonitorChargeSelectSwitch)==LOW && PowerSupplyMode==false) // Monitor Mode
    {if (BatteryChargeMode==true)	// Sw closed means mode change to monitor
			{ BatteryChargeMode=false;
        ActiveCharging=false; //so if go back to charge then restart charging
			  //LastBatteryActive=0;
        TurnOffRelay();
        lcd.clear();  //clear in case amps was displayed, cannot show amps when only monitoring
        EnableBattTypeSet=false;  
			  DataAcqMode=false; //cancel any data acq when changing modes! 
	    }
    }
	}
}
	
void checkPwrSupplyChargerSwitch() {  // will test the charger/pwr supply switch 
   if(digitalRead(PwrSupplyChargerSwitch)==LOW) // Sw closed means pwr supply mode
	  {	if (PowerSupplyMode==false)	//mode change to pwr supply, we check so we don't repeat
    		{ PowerSupplyMode=true;
          lcd.clear();
    			BatteryChargeMode=false; // do not charge in this mode!
          ActiveCharging=false; // so if go back to charge then restart charging
    			lcdPrintLocatedString(0,1,"Mode is Power Supply");
    			TurnOnRelay(1); 
    			EnableBattTypeSet=false;   
    	    DataAcqMode=false; //cancel any data acq when changing modes! 
    	  }
	  } 
  else
   {	if (PowerSupplyMode==true)	// Sw open means mode change to charge or monitor
  		{ PowerSupplyMode=false;
        TurnOffRelay();
        lcd.clear();
        EnableBattTypeSet=false;  
        DataAcqMode=false;
  	  }
	 }
}

void checkBatterySelectSwitches() {  // if sw pushed, include that battery in charging/monitoring
  for (int n=1;n<5;n++){
      if(digitalRead(BattSelectSw[n])==LOW) // Sw open means want that battery too unless was
        { if (BattSelectActive[n]==false)    // activate this batt
              {  BattSelectActive[n]=true;
                 digitalWrite(LEDs[n],HIGH); //turn on appropriate LED
                 if (LastBatteryActive==0) 
                    {LastBatteryActive=n; EnableBattTypeSet=true;}
              }
           else if (BattSelectActive[n]==true)   // was active, now not
              {  BattSelectActive[n]=false;
                 digitalWrite(LEDs[n],LOW); //turn off appropriate LED
                 //if this chan was charging, turn it off
                 if (LastBatteryActive==n)
                      {TurnOffRelay();
                       LastBatteryActive=0;
                       ActiveCharging=false;}
              } 
        }
  }
}

void checkBattTypeSwitch(){	
   bool TypeSwNewClosureFlag=false;
  //when user pushes Batt sw, then check after for type set
  //each push inc's type,  BatteryType[5]={0,0,0,0,0}; //0=SLA(default), 1=AGM, 2=GEL, 3=Flooded 
  //set batt type enabled after batt select sw pushed but only in Monitor Mode
  // but first see if this is a new push of the switch (closed = high)
  if(digitalRead(BatteryTypeSwitch)==HIGH && BatteryTypeSwClosed==false) // Sw closed and was open, new closure 
      {TypeSwNewClosureFlag = true; BatteryTypeSwClosed=true;}
  else if(digitalRead(BatteryTypeSwitch)==LOW) //sw open
      {BatteryTypeSwClosed=false;}

  if (TypeSwNewClosureFlag==true && EnableBattTypeSet==true && digitalRead(MonitorChargeSelectSwitch)==LOW && PowerSupplyMode==false)  
     {  BatteryType[LastBatteryActive]=BatteryType[LastBatteryActive]+1; //types 0=SLA, 1=AGM, 2=GEL, 3=Flooded
       	if (BatteryType[LastBatteryActive]==4)
			  	{BatteryType[LastBatteryActive]=0;} 
     }
  else if(TypeSwNewClosureFlag==true && digitalRead(MonitorChargeSelectSwitch)==HIGH) // use type sw in charging mode to move to next stage
        {BatteryChargeStage=BatteryChargeStage+1; 
            if (BatteryChargeStage==6) {TurnOnRelay(LastBatteryActive);BatteryChargeStage=1;}
            ChargeStageTimeElapsedMins=0;
        } 
}

void CheckNextActiveBattery(){ //see if should charge nxt batt
	byte BatteryThatWasActive=LastBatteryActive; //remember so can tell if changed

	if (LastBatteryActive==1) 
		{if (BattSelectActive[2]==true)
			{LastBatteryActive=2;}
		else if (BattSelectActive[3]==true)
			{LastBatteryActive=3;}
		else if (BattSelectActive[4]==true)
			{LastBatteryActive=4;}
		}
	else if (LastBatteryActive==2) 
		{if (BattSelectActive[3]==true)
			{LastBatteryActive=3;}
		else if (BattSelectActive[4]==true)
			{LastBatteryActive=4;}
		else if (BattSelectActive[1]==true)
			{LastBatteryActive=1;}
		}
	else if (LastBatteryActive==3) 
		{if (BattSelectActive[4]==true)
			{LastBatteryActive=4;}
		else if (BattSelectActive[1]==true)
			{LastBatteryActive=1;}
		else if (BattSelectActive[2]==true)
			{LastBatteryActive=2;}
		}
	else if (LastBatteryActive==4) 
		{if (BattSelectActive[1]==true)
			{LastBatteryActive=1;}
		else if (BattSelectActive[2]==true)
			{LastBatteryActive=2;}
		else if (BattSelectActive[3]==true)
			{LastBatteryActive=3;}
		}
	if (LastBatteryActive!=BatteryThatWasActive)
		//reset charging stage because we changed which batt to charge
		{   ActiveBatt=LastBatteryActive;
		    Serial.print("Nxt Active Batt=");Serial.println(ActiveBatt);
		    ChargeStageTimeElapsedMins=0;
        // see if Battery is already charged, yes if Voltage is => 12.70 volts SLA, 12.80 AGM
        if (BattVolts[ActiveBatt] >= SLA100PercentChargedVolts) 
            {BatteryChargeStage=5; TurnOffRelay();} //just monitor as fully charged
        else if (BattVolts[ActiveBatt] >= SLA90PercentChargedVolts) 
            {BatteryChargeStage=6; //set to ramp to float mode as already 90% charged
             TurnOnRelay(ActiveBatt);
             SetSpecificDACVoltage(BattVolts[ActiveBatt]);}
        else
            {BatteryChargeStage=1;TurnOnRelay(ActiveBatt);} //less than 90% charged so start charge process
 		}
	else
		{ if (BatteryChargeStage!=5) //same batt so monitor for SOC
			{BatteryChargeStage=5;TurnOffRelay();}	
		}
}

void CheckIfOneSecondElapsed(){ // check for one second passed
	delay(100);
    RTC.read(clock);
	if (CurrentSeconds!=clock.Second){ 
		  OneSecElapsed = true;
		  CurrentSeconds=clock.Second;
		  TenSecondTimer--;
		  if (TenSecondTimer==0){
		      TenSecElapsed=true; TenSecondTimer=10;}
  }
	else {OneSecElapsed = false;}
}

void CheckIfOneMinuteElapsed(){ // check for one minute passed
	delay(100);
    RTC.read(clock);
	if (CurrentMinutes!=clock.Minute){ 
		  OneMinElapsed = true;
		  CurrentMinutes=clock.Minute;}
}

void read16bitAD(){
    int Vdrop = 0; byte n=3; //need because num for a/d and array are reverse of each other
    for (int i=1;i<5;i++){
        BattVolts[i] =ads1115.readADC_SingleEnded(n); //B1=A3, B2=A2, B3=A1, B4=A0
        //compensate for approx .05 volts (80 bits) drop in cables at 0.8 amp which is .0625 ohms 
        // approx 10 bits for every 0.1 amps
        if (current_mAINA219>10){Vdrop = current_mAINA219/10;}
        if (i==ActiveBatt) {BattVolts[i]=BattVolts[i]-Vdrop;}
    		if (BattVolts[i]<0) BattVolts[i]=0;
        n=n-1;
	  }
}

void readINA219CurrentSensor(){
  shuntvoltageINA219 = ina219.getShuntVoltage_mV();
  busvoltageINA219 = ina219.getBusVoltage_V();
  current_mAINA219 = ina219.getCurrent_mA();
  current_AmpsINA219 = current_mAINA219/1000;
  if (current_AmpsINA219 < 0) {current_AmpsINA219=0;}
  //Serial.print ("Current Amps=");Serial.println(current_AmpsINA219);
  power_mWINA219 = ina219.getPower_mW();
  loadvoltageINA219 = busvoltageINA219 + (shuntvoltageINA219 / 1000);
}

void readExtINA219CurrentSensor(){
  shuntvoltageINA219 = ina219_Ext.getShuntVoltage_mV();
  busvoltageINA219 = ina219_Ext.getBusVoltage_V();
  current_mAINA219 = ina219_Ext.getCurrent_mA();
  current_AmpsINA219 = current_mAINA219/1000;
  if (current_AmpsINA219 < 0) {current_AmpsINA219=0;}
  power_mWINA219 = ina219_Ext.getPower_mW();
  loadvoltageINA219 = busvoltageINA219 + (shuntvoltageINA219 / 1000);
}

void SetSpecificDACVoltage(int Volts){
  int ComputedDACVolts=0;
    //convert from Batt Volts (11.248-16.152) to DAC Volts (0-5.00)
    if (Volts < 17997) {Volts = 17997;} // min DAC volts
    ComputedDACVolts = Volts - 17997; //subtract 11.248 base V
    ComputedDACVolts = ComputedDACVolts * 0.521855; //convert Batt Volts range 7847 to DAC range 4095
    dac.setVoltage (ComputedDACVolts, false);
    DACvoltage=ComputedDACVolts;
}

void DACVoltageSetToTarget(bool updown,int VoltsTarget) {//compute how far away we are from target V
int OffsetToTargetV = 0;
if (updown==true)	//up means we are low
	   { OffsetToTargetV = (VoltsTarget - BattVolts[ActiveBatt]) * 0.521855; //convert Batt Range to DAC Range
 		 DACvoltage=DACvoltage + OffsetToTargetV; 
		 if (DACvoltage>=4096) {DACvoltage=4095;}
	   }  
	else	//dwn means we are high
	   { OffsetToTargetV = (BattVolts[ActiveBatt] - VoltsTarget) * 0.521855;
		 DACvoltage=DACvoltage - OffsetToTargetV; 
		 if (DACvoltage<0) {DACvoltage=0;}
	   }  
	dac.setVoltage (DACvoltage, false);
}
 
void SetDACVoltage(bool DACDir){  //DAC output step up or step down 
	if (DACDir==true)	//up
	   {  DACvoltage=DACvoltage+DACVstep;
          if (DACvoltage>=4096) {DACvoltage=4095;}
	   }  
	else	//dwn
	   {  DACvoltage=DACvoltage-DACVstep;
       	  if (DACvoltage<0) {DACvoltage=0;}
	   }  
	dac.setVoltage (DACvoltage, false);
}

void SetDACVoltageStep(bool DACDir, int DACStep){  //DAC output step up or step down 
  if (DACDir==true) //up
     {  DACvoltage=DACvoltage+DACStep;
          if (DACvoltage>=4096) {DACvoltage=4095;}
     }  
  else  //dwn
     {  DACvoltage=DACvoltage-DACStep;
          if (DACvoltage<0) {DACvoltage=0;}
     }  
  dac.setVoltage (DACvoltage, false);
}

void AdjustDACforCurrent(int unsigned CurrentDesired) {
  int unsigned DACAdjustStep = 0;
//try to get closer with desired current by setting V more accurately, approx 0.00122 volts/bit
// in 2 Amp CC mode, approx .014v/.086a, so if .086a from desired, adj v by .014v or 11 bits, .008a/bit
// if .043a 43ma from desired, adj v by .007v or 6 bits, current is approx 8ma/bit
if (current_mAINA219 > CurrentDesired) //lower v
   { DACAdjustStep = (current_mAINA219 - CurrentDesired)/8; 
		DACvoltage=DACvoltage-(DACAdjustStep);
       if (DACvoltage<0) {DACvoltage=0;}
   }
else	//raise v
	   {  DACAdjustStep = (CurrentDesired - current_mAINA219)/8; 
	      DACvoltage=DACvoltage+(DACAdjustStep);
       	  if (DACvoltage>=4096) {DACvoltage=4095;}
	   }  
	dac.setVoltage (DACvoltage, false);
}

void TurnOnRelay(byte WhichRelay){    
    if ((WhichRelay==1) && (LastBattRly != BattRly1))
    { // turn off last relay and and turn on relay 1 if not already on
      digitalWrite(LastBattRly, HIGH);  //turn off last relay
      digitalWrite(BattRly1, LOW);  //active low turns on
      LastBattRly=BattRly1;
    }
    else if ((WhichRelay==2) && (LastBattRly != BattRly2))
    { // turn off last relay and and turn on relay 2
      digitalWrite(LastBattRly, HIGH);  //turn off last relay
      digitalWrite(BattRly2, LOW);  //active low turns on
      LastBattRly=BattRly2;
    }
    else if ((WhichRelay==3) && (LastBattRly != BattRly3))
    { // turn off last relay and and turn on relay 3
      digitalWrite(LastBattRly, HIGH);  //turn off last relay
      digitalWrite(BattRly3, LOW);  //active low turns on
      LastBattRly=BattRly3;
    }
    else if ((WhichRelay==4) && (LastBattRly != BattRly4))
    { // turn off last relay and and turn on relay 4
      digitalWrite(LastBattRly, HIGH);  //turn off last relay
      digitalWrite(BattRly4, LOW);  //active low turns on
      LastBattRly=BattRly4;
    }
}

void TurnOffRelay(){    
     // turn off last relay
      digitalWrite(LastBattRly, HIGH);  //active low, high = off
      LastBattRly=0; // no relays on
      if (DataAcqMode==false)
	      {Serial.println (F("Relay is Off ")); 
      	  SerialPrintVolts();}
}

 
void TurnOffLEDs(byte whichLEDs){ //LEDs are pins 14 to 17 (A0-A3)
	if (whichLEDs==0)
		{   for (int n=1;n<5;n++){
			     digitalWrite(LEDs[n],LOW);LEDs[whichLEDs+4]=0;}	// active high so turn off
		}
	else
		{digitalWrite(LEDs[whichLEDs],LOW);LEDs[whichLEDs+4]=0;}	// active high so turn off
}

void SetLEDSOnOffFlash() {	//LEDs are A0-A3
  //Charging Stage 1 = Pulse Flash 100ms on, Stage 2 =Pulse Flash 250ms on , 
// Stage 3= 1s Flash, Stage 4 = On or Monitor = On
  for (int n=1;n<5;n++){
     	if (BattSelectActive[n]==true && ActiveBatt==n && BatteryChargeMode==true)  //this batt is currently active but is it charging
      	{	if (BatteryChargeStage==1)	//double pulse flash 75ms
      		{	digitalWrite(LEDs[n],HIGH);	//active high
      			delay(75); digitalWrite(LEDs[n],LOW);
            digitalWrite(LEDs[n],HIGH); //active high
            delay(75); digitalWrite(LEDs[n],LOW);LEDs[n+4]=0;
      		}
      		else if (BatteryChargeStage==2)	//pulse flash 250ms
      		{	digitalWrite(LEDs[n],HIGH);	//active high
      			delay(250); digitalWrite(LEDs[n],LOW);LEDs[n+4]=0;
      		}
      		else if (BatteryChargeStage==3)	//1s flash
      		{	//Serial.print ("LED S3 for Batt" ); Serial.println (BattSelectActive[n]);
      		  if (LEDs[n+4]==0)
      				{digitalWrite(LEDs[n],HIGH);LEDs[n+4]=1;}	//was off, turn on
      			else
      				{digitalWrite(LEDs[n],LOW);LEDs[n+4]=0;}	//was on, turn off
      		}
      		else if (BatteryChargeStage==4 || BatteryChargeStage==0)	
      				{digitalWrite(LEDs[n],HIGH);LEDs[n+4]=1;}	//turn on
      	}
       else if (BattSelectActive[n]==true && BatteryChargeMode==false && PowerSupplyMode==false ) // batt selected but not active and not charging
          {digitalWrite(LEDs[n],HIGH);LEDs[n+4]=1;} //turn on
    }	
}

void DisplaySwStates(){
  //print states of all switches
  Serial.print (F("BSel1="));Serial.print (digitalRead(BattSelectSw1));
  Serial.print (F(" BSel2="));Serial.print (digitalRead(BattSelectSw2));
  Serial.print (F(" BSel3="));Serial.print (digitalRead(BattSelectSw3));
  Serial.print (F(" BSel4="));Serial.println (digitalRead(BattSelectSw4));
  Serial.print (F(" MonChrg="));Serial.print (digitalRead(MonitorChargeSelectSwitch));
  Serial.print (F(" ChrgPwrS="));Serial.print (digitalRead(PwrSupplyChargerSwitch));
  Serial.print (F(" Type="));Serial.println (digitalRead(BatteryTypeSwitch));
 }

void SerialandEEPromSendAcqData(){
  // also record to eeprom in clock module AT24C32 4K but only if doing data acq every minute 
  WriteChargeDataToEEProm();
  SerialSendAcqData();
}

void SerialSendAcqData(){		//send V and I for B1 or External
  float VDisplay;
    if (DataAcqExternal==false)
    		{read16bitAD();
    		VDisplay = BattVolts[ActiveBatt] * BattCalFactor; //0-20.48 volts at 32768 points Nominal is 0.000625/bit
    		Serial.print(F("S")); Serial.print (BatteryChargeStage);Serial.print(F("  "));
    		//Serial.print (ChargeStageTimeElapsedMins);Serial.print(F("  "));
    		Serial.print (VDisplay,3); Serial.print (F(", "));
    		readINA219CurrentSensor();
        Serial.println(current_AmpsINA219,3);}
    		//Serial.println (current_mAINA219,3);}
	// else read the external input current sensor for both voltage and current
	else
    		{readExtINA219CurrentSensor();
    		VDisplay = loadvoltageINA219;
    		//Serial.print(F("ExtB, S")); Serial.print (BatteryChargeStage);Serial.print(F("  "));
    		//Serial.print (ChargeStageTimeElapsedMins);Serial.print(F("  "));
    		Serial.print (VDisplay,3); Serial.print (F(", "));
        Serial.println(current_AmpsINA219,3);}
}

void SerialPrintChargeMonitorStatus(){  //display b1-b4 stats
	for (int n=1;n<5;n++){ 
		if (BattSelectActive[n]==true)
			{ SerialPrintChargeInfo(n); Serial.print (F(", "));} //BatteryType[]
		else
			{ Serial.print ("B"); Serial.print(n); SerialPrintBattType(n); Serial.print(F(" OFF,  "));}
		}
	Serial.println();
}

void SerialPrintChargeInfo(byte WB) {
  float VDisplay;
  VDisplay = BattVolts[WB] * BattCalFactor;
  Serial.print(F("B")); Serial.print(WB); 
  if (PowerSupplyMode==false && BatteryChargeMode==false) // Monitor Mode
    {SerialPrintBattType(WB);Serial.print(F(" M ")); Serial.print(VDisplay,3);}
  else //charging mode
    {SerialPrintBattType(WB);
     if (ActiveBatt==WB) //only show current for current active batt charging
         {Serial.print(F(" S"));Serial.print(BatteryChargeStage);Serial.print(F("  ")); Serial.print(VDisplay,3);
          Serial.print(F("  ")); Serial.print(current_AmpsINA219, 3);Serial.print(F("  "));Serial.print (ChargeStageTimeElapsedMins);Serial.print(F(" "));}
      else
          {Serial.print(F(" S0 "));Serial.print(VDisplay,3);}
    } 
}

void SerialPrintBattType(byte WchB) {	//byte BatteryType[5]={0,0,0,0,0}; //0=SLA, 1=AGM, 2=GEL, 3=Flooded
  	Serial.print(" ");
  	if (BatteryType[WchB]==0) {Serial.print(F("SL"));}
  	else if (BatteryType[WchB]==1) {Serial.print(F("AG"));}
	  else if (BatteryType[WchB]==2) {Serial.print(F("GL"));}
	  else if (BatteryType[WchB]==3) {Serial.print(F("FL"));}
}

void SerialprintDACVoltage(){
  float VDisplay;
  Serial.print (F("DAC V ="));
  //VDisplay = DACvoltage * 0.00122; //0-5 volts at 4095 points Nominal is 0.00122 v/b
  VDisplay = DACvoltage * 0.001198; //0-5 volts at 4095 points Corrected is 0.0011198 v/b
  Serial.print (DACvoltage); Serial.print (F("  "));
  Serial.print (VDisplay, 5); 
  read16bitAD();
  VDisplay = BattVolts[1] * BattCalFactor; 
  Serial.print(F("  Batt1 Volts:")); Serial.println (VDisplay,3);
}

void SerialPrintVolts(){
  float VDisplay;
    read16bitAD();
	Serial.print(F("Batt1 Volts:")); Serial.print (BattVolts[1]); 
	Serial.print(F("  Batt2 Volts:")); Serial.print (BattVolts[2]);  
	Serial.print(F("  Batt3 Volts:")); Serial.print (BattVolts[3]); 
	Serial.print(F("  Batt4 Volts:")); Serial.println (BattVolts[4]); 
    
	VDisplay = BattVolts[1] * BattCalFactor; //0-20.48 volts at 32768 points Nominal is 0.000625/bit
	Serial.print(F("Batt1 Volts:")); Serial.print (VDisplay,3);
	VDisplay = BattVolts[2] * BattCalFactor; 
	Serial.print(F("  Batt2 Volts:")); Serial.print (VDisplay,3); 
	VDisplay = BattVolts[3] * BattCalFactor; 
	Serial.print(F("  Batt3 Volts:")); Serial.print (VDisplay,3);
	VDisplay = BattVolts[4] * BattCalFactor; 
	Serial.print(F("  Batt4 Volts:")); Serial.println (VDisplay,3); 
}

void SerialPrintCurrent(){
  Serial.print(F("Bus Voltage:   ")); Serial.print((ina219.getBusVoltage_V()),3); Serial.println(F(" V"));
  Serial.print(F("Shunt Voltage: ")); Serial.print(ina219.getShuntVoltage_mV()); Serial.println(F(" mV"));
  Serial.print(F("Load Voltage:  ")); Serial.print(loadvoltageINA219,3); Serial.println(F(" V"));
  Serial.print(F("CurrentMA:       ")); Serial.print(current_mAINA219,3); Serial.println(F(" MA"));
  Serial.print(F("CurrentA:       ")); Serial.print(current_AmpsINA219,3); Serial.println(F(" A"));
  Serial.print(F("Power:         ")); Serial.print(power_mWINA219); Serial.println(F(" mW"));
  Serial.println("");
}

void SerialPrintTime(){
	// Send date over serial connection
	Serial.print(F("Date: "));
	Serial.print(clock.Month, DEC);
	Serial.print(F("/"));
	Serial.print(clock.Day, DEC);
	Serial.print(F("/"));
	Serial.print(clock.Year+1970, DEC);

	// Send Day-of-Week and time
	Serial.print(F("  "));
    //Serial.print(RTC.getDOWStr());  //get actual day string
    Serial.print(daysOfTheWeek[clock.Wday]);
    Serial.print(F("  "));
	Serial.print(F("Time: "));
	Serial.print(clock.Hour, DEC);
	printDigitsserial(clock.Minute);	// adj to 00 if only 0 add :
	printDigitsserial(clock.Second);
	
	// print current temperature
	Serial.print(F("   Temp: "));
	//Serial.print(RTC.getTemp());
	Serial.print(RTC.temperature()/4);
	Serial.println(F(" C"));
	Serial.println(F("--------------------------------"));
}

void SerialPrintPowerSupplyVoltsAmps(){
  float VDisplay;
  VDisplay = BattVolts[1] * BattCalFactor;
  Serial.print (VDisplay, 3);
  Serial.print (F(" Amps="));
  Serial.println (current_AmpsINA219,3);
}

void lcdPrintPowerSupplyVoltsAmps(){
  float VDisplay;
  VDisplay = BattVolts[1] * BattCalFactor;
  if (VDisplay < 10)
    {lcdPrintLocatedString(0,2,"Volts = ");  
     lcd.setCursor (8,2);}
  else
    {lcdPrintLocatedString(0,2,"Volts ="); 
    lcd.setCursor (7,2);}
  lcd.print (VDisplay,3);
  lcdPrintLocatedString(0,3,"Amps ="); 
    lcd.setCursor (7,3); 
    lcd.print (current_AmpsINA219, 3);
}

void lcdPrintExtVI(){
  float VDisplay;
	lcdPrintLocatedString(0,0,"ExtB             "); 
	lcd.setCursor (5,0);
	readExtINA219CurrentSensor();
	VDisplay = loadvoltageINA219;
  lcd.print (VDisplay,3);
	lcd.setCursor (12,0);
  if (current_AmpsINA219<0) { current_AmpsINA219=0;}
  lcd.print (current_AmpsINA219, 3);
}

void lcdPrintChargeMonitorStatus(){  //display b1-b4 stats
  for (int n=1;n<5;n++){
    if (BattSelectActive[n]==true)
      { lcdPrintChargeLine(n-1, n); } //which lcd line (0-3) and which batt#(1-4)
    else
      { String BState = "B";BState.concat(n);
        if (BatteryType[n]==0) {BState.concat(" S");}
        else if (BatteryType[n]==1) {BState.concat(" A");}
        else if (BatteryType[n]==2) {BState.concat(" G");}
        else if (BatteryType[n]==3) {BState.concat(" F");}
        
        BState.concat(" OFF            "); //B1 A OFF, B1 A S1 12.393 1.393
        lcdPrintLocatedString(0,n-1,BState);}
    }
}

void lcdPrintChargeLine(byte WL, byte WBatt) {
  float VDisplay;
  VDisplay = BattVolts[WBatt] * BattCalFactor;
  lcdPrintLocatedString(0,WL,"B"); 
  lcdPrintLocatedInt(1,WL,WL+1); 
  if (PowerSupplyMode==false && BatteryChargeMode==false) // Monitor Mode
    { lcdPrintBatteryType(WL, WBatt);
      lcdPrintLocatedString(4,WL," M    ");
      lcd.setCursor (8,WL);
      lcd.print (VDisplay,3);}
  else //charging mode
    { lcdPrintBatteryType(WL, WBatt);
      if (ActiveBatt==WBatt) 
          {lcdPrintLocatedString(5,WL,"S      ");lcdPrintLocatedInt(6,WL,BatteryChargeStage);}
      else
          {lcdPrintLocatedString(5,WL,"S0      ");}
      lcd.setCursor (8,WL);
      lcd.print (VDisplay,3);
      if (ActiveBatt==WBatt) //only show current for current active batt charging
        {lcd.setCursor (15,WL);lcd.print (current_AmpsINA219, 3);}
      else
        {lcd.setCursor (15,WL);lcd.print ("     ");}
    } 
} 

void lcdPrintBatteryType(byte WchLn, byte WBatt){
    if (BatteryType[WBatt]==0) {lcdPrintLocatedString(2,WchLn," S");}
    else if (BatteryType[WBatt]==1) {lcdPrintLocatedString(2,WchLn," A");}
    else if (BatteryType[WBatt]==2) {lcdPrintLocatedString(2,WchLn," G");}
    else if (BatteryType[WBatt]==3) {lcdPrintLocatedString(2,WchLn," F");}
}

void lcdPrintTime(bool statDsp){
    lcd.setCursor ( 0, 0 );            // go to the top left corner
    printDigitsDatelcd(clock.Month);
    printDigitsDatelcd(clock.Day);  // adj to 00 if only 0 add /
    lcd.print(clock.Year+1970);
    lcd.print("          ");  
    lcd.setCursor(11,0);
    if (statDsp==true)
    {
      lcd.print(daysOfTheWeekShrt[clock.Wday]);
      lcd.setCursor (15,0);
      printDigitslcd(clock.Hour, false); // don't print the :
      printDigitslcd(clock.Minute, true);}
    else 
    {
      //lcd.print(daysOfTheWeek[clock.dow]); //lcd.print(clock.dow);
      lcd.print(daysOfTheWeek[clock.Wday]); //lcd.print(clock.dow);
      lcd.setCursor (0,1);
      printDigitslcd(clock.Hour, false); // don't print the :
      printDigitslcd(clock.Minute, true); // adj to 00 if only 0 and add :
      printDigitslcd(clock.Second, true);
      lcd.print("          "); }
}
void lcdSetStringLen(String StrgToSet, int LenStrg, bool SpcBefore){ 
  String RightLen=StrgToSet;
  if (SpcBefore==true) //put the space before the string if string too short
  {
      if (StrgToSet.length() < LenStrg) 
      { RightLen =  String(" " + StrgToSet ); }
      else if (StrgToSet.length() > LenStrg) // too long, shorten it
      { RightLen = StrgToSet.substring(0, LenStrg);}
  }
  else // if too short put spc after
  {
    if (StrgToSet.length() < LenStrg) 
      { RightLen =  String(StrgToSet+" " ); }
      else if (StrgToSet.length() > LenStrg) // too long, shorten it
      { RightLen = StrgToSet.substring(0, LenStrg);}
  }
  lcd.print (RightLen);
}

void printDigitslcd(int digits, bool divider){
    // utility function for digital clock display: prints preceding colon and leading 0
    if (divider==true){
        lcd.print(":");}
    if(digits < 10){
        lcd.print("0");}
    lcd.print(digits);
}

void lcdPrintLocatedString (int Col, int Row, String ToPrint){    
  lcd.setCursor ( Col, Row);            
  lcd.print(ToPrint);
}

void lcdPrintLocatedInt( int Col, int Row, int IPrint){
  lcd.setCursor ( Col, Row);            
  lcd.print(IPrint);
 } 

void printDigitsDatelcd(int digits){
    // utility function for digital clock display: prints preceding colon and leading 0
    if(digits < 10){
        lcd.print("0");}
    lcd.print(digits);
    lcd.print("/");
}

void printDigitsserial(int digits){
  // utility function for clock display: prints preceding colon and leading 0
  Serial.print(':');
  if(digits < 10){
      Serial.print('0');}
  Serial.print(digits);
}

void CkforSerialCommands() {
	   char incoming_char="";
     bool multipleCharsflag=false;
   
	  	/*if (Serial.available() > 0) 
	    {   // Read the incoming character
	        incoming_char = Serial.read();
          delay(2);
         Serial.write(incoming_char);
		*/
		
		 if (Serial.available() > 0 || BTSerial.available() > 0)  
       	 {   if (Serial.available()==0) {incoming_char = BTSerial.read();}
       	   	 else {incoming_char = Serial.read();}

          //Serial.print ("Incoming Char=");Serial.println(incoming_char);
          	if (incoming_char=='a') {TurnOffRelay();} //turns off all relays
          	else if (incoming_char=='b') {ActivateRelay();} //turn on rly1,2,3 or 4
          	
            else if (incoming_char=='g') //battery type, Batt# then Type, ex; b1S = Batt 1 = Sla, b3A = Batt 3 = AGM
            	{SetBatteryType();}

            else if(incoming_char== 'u') { SetDACVoltage(true);SerialprintDACVoltage();} // Raise DAC voltage 0.1v
            else if(incoming_char== 'd') { SetDACVoltage(false);SerialprintDACVoltage();}// Lower DAC voltage 0.1v

            else if (incoming_char=='1') {TenSecDataFlag=false;OneMinDataFlag=false;} //
            else if (incoming_char=='2') {TenSecDataFlag=true;OneMinDataFlag=false;} 
            else if (incoming_char=='3') {OneMinDataFlag=true;TenSecDataFlag=false;} 
            else if (incoming_char=='5') {SetSpecificDACVoltage(17997);SerialprintDACVoltage();} //set DAC to min
            else if (incoming_char=='6') {SetSpecificDACVoltage(25844);SerialprintDACVoltage();} //set DAC to max
            else if (incoming_char=='7') {SetSpecificDACVoltage(21919);SerialprintDACVoltage();} //set DAC to mid
          	else if (incoming_char=='i') {readINA219CurrentSensor(); SerialPrintCurrent();} //display current amps
          	else if (incoming_char=='j') {readExtINA219CurrentSensor(); SerialPrintCurrent();} //display current amps
                                                                        
           	else if (incoming_char=='q') {DataAcqMode=true;DataAcqExternal=false;LastEEPromAddr=EEPromDataStartAddr;SerialSendAcqData();} //get data from B1,send to serial
           	else if (incoming_char=='t') {DataAcqMode=true;DataAcqExternal=true;LastEEPromAddr=EEPromDataStartAddr;SerialSendAcqData();}
            else if (incoming_char=='c') {DataAcqMode=true;DataAcq10SecMode=false;DataAcqExternal=true;LastEEPromAddr=EEPromDataStartAddr;FastDataAcqExternal=true; SerialSendAcqData();}
            else if (incoming_char=='m') {DataAcqMode=true;DataAcq10SecMode=false;DataAcqExternal=false;LastEEPromAddr=EEPromDataStartAddr;FastDataAcqInternal=true; SerialSendAcqData();}
          	else if (incoming_char=='r') {DataAcqMode=false;DataAcq10SecMode=false;DataAcqExternal=false;FastDataAcqExternal=false;FastDataAcqInternal=false;} //stop data acq mode
            else if (incoming_char=='s') {DataAcqMode=true;DataAcq10SecMode=true;DataAcqExternal=false;FastDataAcqExternal=false;FastDataAcqInternal=false;} //10s Int data acq mode
            else if (incoming_char=='v') {DataAcqMode=true;DataAcq10SecMode=true;DataAcqExternal=true;FastDataAcqExternal=false;FastDataAcqInternal=false;} //10s Int data acq mode
          	                              
          	
          	else if (incoming_char=='w') {DisplaySwStates();}
            // turn leds off then turn on specific selected LED A0-A3 High 
            else if (incoming_char=='e') {TurnOffLEDs(0);} 
            else if (incoming_char=='f') {TurnOnLED();} 
            else if (incoming_char=='h') {BatteryCharge4Stages=!BatteryCharge4Stages;} //enables 4 stage charging 
            else if (incoming_char=='k') {ReadDataFromEEProm();}

            else if (incoming_char=='n') { RTC.read(clock);SerialPrintTime();}
            else if (incoming_char=='l') { RTC.read(clock);SerialPrintTime();SerialPrintVolts();}
            else if (incoming_char=='o') { SerialPrintVolts();}
            else if (incoming_char=='z') { SetClockTime();}
            else if (incoming_char=='y') {SetBatteryVoltage();} //set battery voltage
            else if (incoming_char=='0') {Serial.print ("Version=");Serial.println(Version);} 
            else if (incoming_char=='p' || incoming_char==" ") 
              { pauseflag=!pauseflag;}
      }
}

void ActivateRelay(){
  WaitForChar();
  char WRelay = Serial.read(); //read 2nd char which Relay to turn on
  TurnOffRelay(); //turn off any on
  if (WRelay == '1') {TurnOnRelay(1);} 
  else if (WRelay=='2') {TurnOnRelay(2);}
  else if (WRelay=='3') {TurnOnRelay(3);}
  else if (WRelay=='4') {TurnOnRelay(4);}
} 

void TurnOnLED(){
  WaitForChar();
  char LEDNum = Serial.read(); //read 2nd char which led to turn on
  Serial.print ("LED On=");Serial.println(LEDNum);
	TurnOffLEDs(0);
	if (LEDNum == '1') {digitalWrite(A0,HIGH);} 
 	else if (LEDNum=='2') {digitalWrite(A1,HIGH);}
  else if (LEDNum=='3') {digitalWrite(A2,HIGH);}
  else if (LEDNum=='4') {digitalWrite(A3,HIGH);}
} 

void SetBatteryType(){ //0=SLA, 1=AGM, 2=GEL, 3=Flooded - cmd ex; b2g
  char BIndx ="";  char BType ="";
  byte TypeNum=0; byte BattIndex=0;
 //read two more chars
  BIndx = Serial.read(); BType= Serial.read();
 	if (BType=='s') {TypeNum=0;} 	//convert to type 
  else if (BType=='a') {TypeNum=1;}   
  else if (BType=='g') {TypeNum=2;} 
  else if (BType=='f') {TypeNum=3;} 

  BattIndex=byte(BIndx)-48; //cnv ascii char to number
  BatteryType[BattIndex]=TypeNum;
	//Serial.print ("index=");Serial.print(BIndx);Serial.print(" type=");Serial.print (BType);
	//Serial.print (" cnv index=");Serial.print(BattIndex);Serial.print (" cnv type=");Serial.println(TypeNum);
}

void SetBatteryVoltage(){ // setting to 11.250 to 16.250, read chars until cr
 float BVoltage=0; char VoltsChar=""; byte ChCount=0; String inString = ""; 
 int DacV = 0;
  Serial.println(F("SetBattV "));
	do {VoltsChar= Serial.read();    //read chars until cr or more than 7 chs
		if (VoltsChar != '\n')
        {ChCount++;
        inString += (char)VoltsChar;
        //Serial.print(" Ch="); Serial.print (VoltsChar);Serial.print (" ChCount=");Serial.print(ChCount);
        }
    else
			{;break;} //Serial.println ("Got CR  ")
    }while (ChCount<7);
  
     BVoltage=inString.toFloat();
     Serial.print(F("Float Convert = "));Serial.println(BVoltage,3);
     //now convert to bits and set dac voltage
     BVoltage=BVoltage-11.250;	//set to DAC range 0-5v
     DacV=BVoltage/.001221;	//convert to bits 0-4095
     dac.setVoltage (DacV, false);
     Serial.print (F("Setting Dac Voltage to "));Serial.println(DacV);
}

void SetClockTime() {
    time_t t;
    char schar="";
    
    // check for input to set the RTC, minimum length is 12, i.e. yy,m,d,h,m,s
    // best way to set clock is copy/paste into serial monitor ie; z19,5,22,7,37,30
    delay(200);
    if (Serial.available() >= 12) {
        // note that the tmElements_t Year member is an offset from 1970,
        // but the RTC wants the last two digits of the calendar year.
        // use the convenience macros from the Time Library to do the conversions.
        int y = Serial.parseInt();
        if (y >= 100 && y < 1000)
            {Serial.print(F("Err:"));}
        else {
            Serial.println (F("Setting Clock "));
            if (y >= 1000)
                clock.Year = CalendarYrToTm(y);
            else    // (y < 100)
                clock.Year = y2kYearToTm(y);
            clock.Month = Serial.parseInt();
            clock.Day = Serial.parseInt();
            clock.Hour = Serial.parseInt();
            clock.Minute = Serial.parseInt();
            clock.Second = Serial.parseInt();
            t = makeTime(clock);
            RTC.set(t);        // use the time_t value to ensure correct weekday is set
            //setTime(t); //not necessary
            Serial.print (F("RTC set to: "));
            RTC.read(clock);
            SerialPrintTime();
            // dump any extraneous input
            while (Serial.available() > 0) {Serial.read();}
        }
    }
    else
     {Serial.print(F(" Some Err Setting Clock:"));
      while (Serial.available() > 0) {schar = Serial.read();Serial.print (schar);}
      Serial.println();}
}

void WaitForChar(){
  do { CheckIfOneSecondElapsed();
     } while ((Serial.available() == 0 && BTSerial.available() == 0) && TenSecElapsed == false );
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

void SetSerial9600(){
  if (SerialOn != 9600)
    {
      Serial.begin(9600);
        while (!Serial) {
          // will pause until serial console opens
          delay(10);}
     }
  SerialOn = 9600;
}

void WriteChargeDataToEEProm(){ //write mins, BattV, BattI
   unsigned int CurrentAmps = current_AmpsINA219 * 1000; //convert to int from float
	if (LastEEPromAddr < 4001) { //max of 650 data points (6 bytes each) or 650 mins or 4000 bytes
		  mem.writeInt(LastEEPromAddr, ChargeStageTimeElapsedMins); //mins into charging
		  mem.writeInt(LastEEPromAddr+2, BattVolts[1]); //write two bytes of int BattVolts
		  mem.writeInt(LastEEPromAddr+4, CurrentAmps); //write two bytes of int BattCurrent
      mem.writeInt(EEPromDataStartAddr-2, LastEEPromAddr); //save the last data pts address
 	    LastEEPromAddr = LastEEPromAddr+6; //prep for next save
   }
}

void ReadDataFromEEProm(){
  float VDisplay;
  unsigned int VData; 
  unsigned int x=EEPromDataStartAddr; //set starting address for data
  LastEEPromAddr=mem.readInt(EEPromDataStartAddr-2); //last used address is store before starting address
  //read mins, int V data, int I data
     Serial.print ("Rd Chg Data ");
     do {  Serial.print (x); Serial.print (F(" = "));
               Serial.print (mem.readInt(x));Serial.print (F("  "));  //mins
               VData= mem.readInt(x+2); VDisplay = VData * BattCalFactor; Serial.print (VDisplay); Serial.print (F("  "));  //volts
               VData= mem.readInt(x+4);  VDisplay = VData * .001; Serial.println (VDisplay, 3); //current
               x=x+6;
               delay(50);
         } while (x <= LastEEPromAddr); 
}

