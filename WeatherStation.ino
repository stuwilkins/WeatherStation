#include <Adafruit_ATParser.h>
#include <Adafruit_BluefruitLE_SPI.h>
#include <Adafruit_BLEMIDI.h>
#include <Adafruit_BLEBattery.h>
#include <Adafruit_BLEGatt.h>
#include <Adafruit_BLEEddystone.h>
#include <Adafruit_BLE.h>
#include <Adafruit_BluefruitLE_UART.h>
#include <Adafruit_SI1145.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#define ANEMOMETER_IO 			0
#define RAIN_IO       			1
#define WIND_DIRECTION_IO		A0
#define I2C_ENABLE_IO			5

// Calibration constants

// For windspeed 1 click.s^-1 = 1.492 MPH
#define WINDSPEED_CALIBRATION 	1.492

// Global Variables	

volatile long wind_last = 0;
volatile byte wind_clicks = 0;
unsigned long last_wind_check = 0;

volatile unsigned long rain_time;
volatile unsigned long rain_last;
volatile unsigned long rain_interval;
volatile unsigned long rain;

volatile unsigned long rain_day_total;
volatile unsigned long rain_hour_total[60];

unsigned long last_second;
 
Adafruit_BME280 bme; // I2C
void wind_speed_ISR()
{
	if (millis() - wind_last > 10)
	{
		wind_last = millis();
		wind_clicks++;
	}
}

void rainfall_IRQ()
{
	rain_time = millis();
	rain_interval = rain_time - rain_last;

	if(rain_interval > 10)
	{
		rain_day_total++;
		rain_hour_total[minutes]++;
		rain_last = rain_time;
	}
}


void setup() {
	// Start serial and wait for host to connect

	Serial.begin(115200);
	delay(5000);

	pinMode(ANEMOMETER_IO, INPUT_PULLUP);
	pinMode(RAIN_IO, INPUT_PULLUP);
	pinMode(I2C_ENABLE, OUTPUT);

	attachInterrupt(digitalPinToInterrupt(ANEMOMETER_IO), wind_speed_ISR, FALLING);	
	attachInterrupt(digitalPinToInterrupt(RAIN_IO), rainfall_isr, FALLING);	

	interrupts();

	rain_last = millis();
	wind_last = millis();
	last_second = millis();
}

void loop() {
	if(millis() - lastSecond >= 1000)
	{
		last_second += 1000;
	}
	Serial.print(F("Wind speed = "));
	Serial.println(get_wind_speed());
	Serial.print(F("Wind direction = "));
	Serial.println(get_wind_direction());
	delay(1000);
}

bool reset_i2c(void)
{

}	

float get_wind_speed(void)
{
	unsigned long delta_time = millis() - last_wind_check;
	float wind_speed = (float)wind_clicks * 1000 / (float)delta_time;

	wind_clicks = 0;
	last_wind_check = millis();

	wind_speed *= WINDSPEED_CALIBRATION;

	return wind_speed;
}	

int get_wind_direction(void)
{
	unsigned int adc;

	adc = average_analog_read(WIND_DIRECTION_IO);
	Serial.println(adc);
	adc-=5;

	if (adc < 66) return (113);
	if (adc < 84) return (68);
	if (adc < 93) return (90);
	if (adc < 127) return (158);
	if (adc < 184) return (135);
	if (adc < 244) return (203);
	if (adc < 287) return (180);
	if (adc < 406) return (23);
	if (adc < 461) return (45);
	if (adc < 599) return (248);
	if (adc < 630) return (225);
	if (adc < 702) return (338);
	if (adc < 785) return (0);
	if (adc < 827) return (293);
	if (adc < 886) return (315);
	if (adc < 944) return (270);
		
	return (-1); // error, disconnected?
}

int average_analog_read(int pin)
{
	byte numberOfReadings = 32;
	unsigned int running_value = 0;

	for(int x = 0 ; x < numberOfReadings ; x++)
	{
		running_value += analogRead(pin);
	}	
	running_value /= numberOfReadings;

	return(running_value);
}
