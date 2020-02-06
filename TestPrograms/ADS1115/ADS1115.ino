// 4 channel 16 bit A/D module test

#include <Wire.h>
#include <Adafruit_ADS1015.h>

Adafruit_ADS1115 ads;

float ADDecimal;

void setup(void)
{

  SetSerial115();
  Serial.println("Getting single-ended readings from AIN0..3");
  Serial.println("ADC Range: +/- 6.144V ((1 bit = 0.1875mV/ADS1115)");

  ads.begin();
  // set 16 bit A/D to GAIN_ONE (for an input range of +/-4.096V)
  //ads1015.setGain(GAIN_ONE);     // 1x gain   +/- 4.096V  1 bit = 2mV
}

/********************** MAIN *************************/
void loop(void)
{
  int16_t adc0, adc1, adc2, adc3;

  adc0 = ads.readADC_SingleEnded(0);
  adc1 = ads.readADC_SingleEnded(1);
  adc2 = ads.readADC_SingleEnded(2);
  adc3 = ads.readADC_SingleEnded(3);
  ADDecimal = .0001872*adc0; //cal value otherwise .0001875
  Serial.print("AIN0: "); Serial.print(ADDecimal, 3);
  ADDecimal = .0001872*adc1;
  Serial.print("  AIN1: "); Serial.print(ADDecimal, 3);
  ADDecimal = .0001872*adc2;
  Serial.print("  AIN2: "); Serial.print(ADDecimal, 3);
  ADDecimal = .0001872*adc3;
  Serial.print("  AIN3: "); Serial.print(ADDecimal, 3);
  Serial.println(" ");
  
  delay(1000);
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
