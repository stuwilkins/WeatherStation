/*
 * =====================================================================================
 *
 *       Filename:  WeatherStation.ino
 *
 *    Description:  Weatherstation
 *
 *        Version:  1.0
 *        Created:  06/06/2018 05:38:34
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Stuart B. Wilkins (sbw), stuwilkins@mac.com
 *   Organization:  
 *
 * =====================================================================================
 */

#include <Wire.h>
#include <SPI.h>

#include <WiFi101.h>
#include <WiFi101OTA.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"

#include <Adafruit_SI1145.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_INA219.h>
#include <Adafruit_SleepyDog.h>

#include <MAX17043GU.h>
#include <RTClib.h>
#include <PWFusion_AS3935.h>
#include <math.h>

#include "WeatherStation.h"
#include "auth.h"

// Global Variables	

volatile unsigned long wind_last = 0;
volatile unsigned long wind_clicks = 0;
volatile bool lightning_irq = 0;
unsigned long last_wind_check = 0;

volatile uint32_t rain_time;
volatile uint32_t rain_last;
volatile uint32_t rain_interval;
volatile uint32_t rain_total = 0;
volatile uint32_t rain_day_total = 0;
volatile uint32_t rain_hour_total[60];

volatile int16_t this_minute = 0;
volatile int16_t this_2minute = 0;
volatile int16_t this_10minute = 0;

int update_counter;

// Global variables to keep track of accumulations.

uint8_t current_minute = 0;	
int16_t last_day = -1;
int16_t last_minute = -1;
int16_t last_second = -1;
int16_t last_hour = -1;
int32_t wind_direction_total = 0;
int32_t wind_direction_n = 0;

// Hardware definitions

Adafruit_BME280 bme;
Adafruit_SI1145 uv;
MAX17043GU battery;
Adafruit_INA219 ina219;
//PWF_AS3935 lightning0(AS3935_CS, AS3935_IRQ, AS3935_SI);

#ifndef SOFTWARE_RTC
	RTC_DS3231 rtc;
#else
	RTC_Millis rtc;
#endif

readings data;

// MQTT Setup
WiFiClient wifi_client;
Adafruit_MQTT_Client mqtt_client(&wifi_client, "192.168.1.2", 1883, "", "");

WiFiServer wifi_server(8000);

Adafruit_MQTT_Publish mqtt_battery = Adafruit_MQTT_Publish(&mqtt_client, "homeauto/weather/battery");
Adafruit_MQTT_Publish mqtt_temperature = Adafruit_MQTT_Publish(&mqtt_client, "homeauto/weather/weather");
Adafruit_MQTT_Publish mqtt_humidity = Adafruit_MQTT_Publish(&mqtt_client, "homeauto/weather/humidity");
Adafruit_MQTT_Publish mqtt_pressure = Adafruit_MQTT_Publish(&mqtt_client, "homeauto/weather/pressure");
Adafruit_MQTT_Publish mqtt_vis_light = Adafruit_MQTT_Publish(&mqtt_client, "homeauto/weather/vis_light");
Adafruit_MQTT_Publish mqtt_ir_light = Adafruit_MQTT_Publish(&mqtt_client, "homeauto/weather/ir_light");
Adafruit_MQTT_Publish mqtt_uv_index = Adafruit_MQTT_Publish(&mqtt_client, "homeauto/weather/uv_index");
Adafruit_MQTT_Publish mqtt_wind_speed = Adafruit_MQTT_Publish(&mqtt_client, "homeauto/weather/wind_speed");
Adafruit_MQTT_Publish mqtt_wind_direction = Adafruit_MQTT_Publish(&mqtt_client, "homeauto/weather/wind_direction");
Adafruit_MQTT_Publish mqtt_rain_hour = Adafruit_MQTT_Publish(&mqtt_client, "homeauto/weather/rain_hour");
Adafruit_MQTT_Publish mqtt_rain_hour_once = Adafruit_MQTT_Publish(&mqtt_client, "homeauto/weather/rain_hour_once");
Adafruit_MQTT_Publish mqtt_rain_day = Adafruit_MQTT_Publish(&mqtt_client, "homeauto/weather/rain_day");
Adafruit_MQTT_Publish mqtt_rain_day_once = Adafruit_MQTT_Publish(&mqtt_client, "homeauto/weather/rain_day_once");
Adafruit_MQTT_Publish mqtt_rain = Adafruit_MQTT_Publish(&mqtt_client, "homeauto/weather/rain");
Adafruit_MQTT_Publish mqtt_dew_point = Adafruit_MQTT_Publish(&mqtt_client, "homeauto/weather/dew_point");
Adafruit_MQTT_Publish mqtt_wind_speed_2m_ave = Adafruit_MQTT_Publish(&mqtt_client, "homeauto/weather/wind_speed_2m_ave");
Adafruit_MQTT_Publish mqtt_wind_direction_2m_ave = Adafruit_MQTT_Publish(&mqtt_client, "homeauto/weather/wind_direction_2m_ave");
Adafruit_MQTT_Publish mqtt_wind_speed_10m_gust = Adafruit_MQTT_Publish(&mqtt_client, "homeauto/weather/wind_speed_10m_gust");
Adafruit_MQTT_Publish mqtt_wind_direction_10m_gust = Adafruit_MQTT_Publish(&mqtt_client, "homeauto/weather/wind_direction_10m_gust");
Adafruit_MQTT_Publish mqtt_solar_voltage = Adafruit_MQTT_Publish(&mqtt_client, "homeauto/weather/solar_voltage");
Adafruit_MQTT_Publish mqtt_solar_current = Adafruit_MQTT_Publish(&mqtt_client, "homeauto/weather/solar_current");
Adafruit_MQTT_Publish mqtt_battery_soc = Adafruit_MQTT_Publish(&mqtt_client, "homeauto/weather/battery_soc");
Adafruit_MQTT_Publish mqtt_lightning_distance = Adafruit_MQTT_Publish(&mqtt_client, "homeauto/weather/lightning_distance");
Adafruit_MQTT_Publish mqtt_lightning_energy = Adafruit_MQTT_Publish(&mqtt_client, "homeauto/weather/lightning_energy");
Adafruit_MQTT_Publish mqtt_input_voltage = Adafruit_MQTT_Publish(&mqtt_client, "homeauto/weather/input_voltage");
Adafruit_MQTT_Publish mqtt_status = Adafruit_MQTT_Publish(&mqtt_client, "homeauto/weather/status");
Adafruit_MQTT_Publish mqtt_signal = Adafruit_MQTT_Publish(&mqtt_client, "homeauto/weather/signal");

// IP Address Configure

IPAddress wifi_ip(192,168,1,19);
IPAddress wifi_mask(255,255,255,0);
IPAddress wifi_gateway(192,168,1,1);

// Error handler 

void error(const __FlashStringHelper *err) {
	Serial.println(err);
	while(1)
	{
		digitalWrite(BUILTIN_LED_IO, !digitalRead(BUILTIN_LED_IO));
		delay(100);
	}
}

// ISR Routines for wind and rain sensors

void lightning_ISR()
{
	lightning_irq = true;
}

void wind_speed_ISR()
{
if (millis() - wind_last > 10)
{
	wind_last = millis();
	wind_clicks++;
	}
}

void rainfall_ISR()
{
	rain_time = millis();
	rain_interval = rain_time - rain_last;

	if(rain_interval > 10)
	{
		rain_total++;
		rain_day_total++;
		rain_hour_total[this_minute]++;
		rain_last = rain_time;
	}
}

// Setup lightning sensor

void setup_lightning(void)
{
	lightning_irq = false;
	//lightning0.AS3935_DefInit();
	//lightning0.AS3935_ManualCal(AS3935_CAPACITANCE, AS3935_OUTDOORS, AS3935_DIST_EN);
}

// Setup Routine

void setup() {

	Serial.begin(115200);

	pinMode(ANEMOMETER_IO, INPUT_PULLUP);
	pinMode(RAIN_IO, INPUT_PULLUP);
	pinMode(SOLAR_CHARGER_CHRG, INPUT_PULLUP);
	pinMode(SOLAR_CHARGER_DONE, INPUT_PULLUP);
	pinMode(BUILTIN_LED_IO, OUTPUT);

	for(int i=0;i<10;i++)
	{
		digitalWrite(BUILTIN_LED_IO, 1);
		delay(500);
		digitalWrite(BUILTIN_LED_IO, 0);
		delay(500);
	}

	digitalWrite(BUILTIN_LED_IO, 1);
	
	// Setup wind speed and rain interrupts

	rain_last = millis();
	wind_last = millis();
	for(int i=0;i<60;i++)
	{
		rain_hour_total[i] = 0;
	}

	// Setup wind accumulations
	
	for(int i=0;i<120;i++)
	{
		data.wind_speed_2m_t[i] = 0;
		data.wind_speed_2m_c[i] = 0;
		data.wind_direction_2m[i] = 0;
	}
	for(int i=0;i<600;i++)
	{
		data.wind_speed_10m_ave[i] = 0;
		data.wind_direction_10m_ave[i] = 0;
	}

	update_counter = 0;

	// Setup lightning
	setup_lightning();

	// Setup RTC
	Serial.println(F("Setup clock ...."));
	setup_clock();

	// Now setup sensors
	Serial.println(F("Setup sensors ...."));
	setup_i2c_sensors();


	Serial.println("Setting up interrupt handlers ...");

	attachInterrupt(digitalPinToInterrupt(ANEMOMETER_IO), wind_speed_ISR, FALLING);	
	attachInterrupt(digitalPinToInterrupt(RAIN_IO), rainfall_ISR, FALLING);	
	attachInterrupt(digitalPinToInterrupt(AS3935_IRQ), lightning_ISR, RISING);	

	int countdownMS = Watchdog.enable(WATCHDOG_TIME);
	Serial.print("Enabled the watchdog with max countdown of ");
	Serial.print(countdownMS, DEC);
	Serial.println(" milliseconds!");
	Serial.println();

	// Setup WIFI

	Serial.println(F("Setup WIFI ...."));
	setup_wifi();
	wifi_connect();

	Serial.println(F("Setup MQTT ...."));
	mqtt_connect();

	Serial.println("Finished setup .....");

	digitalWrite(BUILTIN_LED_IO, 0);
}

void loop() {
	bool new_hour = false;
	bool new_day = false;
	bool new_minute = false;
	bool new_second = false;

	unsigned long start_millis = millis();

	// pet the dog!
	Watchdog.reset();

	// Do WIFI and MQTT Connection
	wifi_connect();
	mqtt_connect();
	
	// Poll for over the air updates
	WiFiOTA.poll();

	// pet the dog!
	Watchdog.reset();

	DateTime now = rtc.now();

	this_minute = now.minute();
	this_2minute = (now.minute() % 2) * 60;
	this_2minute += now.second();
	this_10minute = (now.minute() % 6) * 60;
	this_10minute += now.second();

	if((last_hour == (UTC_TIME_OFFSET - 1)) && 
	   (now.hour() == UTC_TIME_OFFSET))
	{
		// New Day!
		rain_day_total = 0;
		new_day = true;
		Serial.println("New Day .... ");
	}

	if(last_minute < now.minute())
	{
		// New Minute
		new_minute = true;
		rain_hour_total[this_minute] = 0;
		Serial.println("New Minute .... ");
	}

	if(last_hour < now.hour())
	{
		// New Hour!
		new_hour = true;	
		Serial.println("New Hour .... ");
	}

	if(last_second < now.second())
	{
		// New Second !
		new_second = true;
		Serial.println("New Second .... ");
	}


	// Setup the time from last update

	// Check for lightning interrupt
	if(lightning_irq)
	{
		lightning_irq = false;
		//if(read_lightning(&data))
		//{
		//	write_lightning(&data, now);	
		//}
	}

	if(new_second || new_day || new_hour || new_minute)
	{
		print_clock(&now);
		
		// Lets take windspeed and direction
		Serial.print("New Second .... ");
		Serial.print(this_minute);
		Serial.print(" ");
		Serial.print(this_2minute);
		Serial.print(" ");
		Serial.print(this_10minute);
		Serial.println("");

		read_sensors(&data);
		read_wind(&data, this_2minute, 120, this_10minute, 600);
		calculate_sensors(&data);
		print_sensors(&data);
		if(((now.second() % 2) == 0) || new_hour || new_day)
		{
			write_sensors(&data, now, new_hour, new_day);
			digitalWrite(BUILTIN_LED_IO, !digitalRead(BUILTIN_LED_IO));
		}
	}
	
	last_minute = now.minute();
	last_day = now.day();
	last_hour = now.hour();
	last_second = now.second();

	if(new_second)
	{
		Serial.print(F("loop() ran in "));
		Serial.println(millis() - start_millis);
	}
}

//bool read_lightning(readings *data)
//{
//	uint8_t int_src = lightning0.AS3935_GetInterruptSrc();
//	if(0 == int_src)
//	{
//		Serial.println(F("read_lightning() : interrupt source result not expected"));
//		return false;
//	}
//	else if(1 == int_src)
//	{
//		uint8_t lightning_dist_km = lightning0.AS3935_GetLightningDistKm();
//		uint32_t lightning_energy = lightning0.AS3935_GetStrikeEnergyRaw();
//
//		data->lightning_distance = (uint32_t)lightning_dist_km;
//		data->lightning_energy = (uint32_t)lightning_energy;
//
//		Serial.print(F("read_lightning() : Lightning detected! Distance to strike: "));
//		Serial.print(lightning_dist_km);
//		Serial.println(F(" kilometers"));
//
//		return true;
//	}
//	else if(2 == int_src)
//	{
//		data->lightning_distance = 0;
//		data->lightning_energy = 0;
//		Serial.println(F("read_lightning() : Disturber detected"));
//		return false;
//	}
//	else if(3 == int_src)
//	{
//		Serial.println(F("read_lightning() : Noise level too high"));
//		return false;
//	}
//}

uint32_t compute_rain_hour(void)
{
	// Compute the hourly rainfall
	uint32_t rain = 0;
	for(int i=0;i<60;i++)
	{
		rain += rain_hour_total[i];
	}
	rain *= RAIN_BUCKETS_TO_UM;
	return rain;
}

uint32_t compute_rain_day(void)
{
	// Compute the hourly rainfall
	uint32_t rain = rain_day_total * RAIN_BUCKETS_TO_UM;
	return rain;
}

uint32_t compute_rain(void)
{
	// Compute instantanious rain
	uint32_t rain = rain_total * RAIN_BUCKETS_TO_UM;
	return rain;
}

bool write_lightning(readings *data, DateTime ts)
{
	Serial.println("write_lightning()");

	uint32_t unixtime = ts.unixtime();
	mqtt_publish_data(&mqtt_lightning_distance, unixtime, (int32_t)data->lightning_distance);
	mqtt_publish_data(&mqtt_lightning_energy, unixtime, (int32_t)data->lightning_energy);
		
	return true;
}

bool write_sensors(readings *data, DateTime ts, bool new_hour, bool new_day)
{
	Serial.println("write_sensors()");

	uint32_t unixtime = ts.unixtime();
	mqtt_publish_data(&mqtt_battery, unixtime, (int32_t)data->battery_voltage);
	mqtt_publish_data(&mqtt_battery_soc, unixtime, (int32_t)data->battery_soc);
	mqtt_publish_data(&mqtt_temperature, unixtime, (int32_t)(data->temperature * 1000));
	mqtt_publish_data(&mqtt_humidity, unixtime, (int32_t)(data->humidity * 1000));
	mqtt_publish_data(&mqtt_pressure, unixtime, (int32_t)(data->pressure * 10));
	mqtt_publish_data(&mqtt_uv_index, unixtime, (int32_t)(data->uv_index));
	mqtt_publish_data(&mqtt_vis_light, unixtime, (int32_t)data->vis_light);
	mqtt_publish_data(&mqtt_ir_light, unixtime, (int32_t)data->ir_light);
	mqtt_publish_data(&mqtt_wind_speed, unixtime, (int32_t)(data->wind_speed));
	mqtt_publish_data(&mqtt_wind_direction, unixtime, (int32_t)(data->wind_direction * 22500));
	mqtt_publish_data(&mqtt_rain_hour, unixtime, (int32_t)data->rain_hour);
	mqtt_publish_data(&mqtt_rain_day, unixtime, (int32_t)data->rain_day);
	mqtt_publish_data(&mqtt_dew_point, unixtime, (int32_t)(data->dew_point * 1000));
	mqtt_publish_data(&mqtt_wind_direction_2m_ave, unixtime, (int32_t)(data->wind_direction_2m_ave * 1000));
	mqtt_publish_data(&mqtt_wind_speed_2m_ave, unixtime, (int32_t)(data->wind_speed_2m_ave));
	mqtt_publish_data(&mqtt_wind_speed_10m_gust, unixtime, (int32_t)(data->wind_speed_gust_10m));
	mqtt_publish_data(&mqtt_wind_direction_10m_gust, unixtime, (int32_t)(data->wind_direction_gust_10m * 22500));
	mqtt_publish_data(&mqtt_solar_voltage, unixtime, (int32_t)(data->solar_voltage));
	mqtt_publish_data(&mqtt_solar_current, unixtime, (int32_t)(data->solar_current));
	mqtt_publish_data(&mqtt_input_voltage, unixtime, (int32_t)(data->input_voltage));
	mqtt_publish_data(&mqtt_status, unixtime, (int32_t)(data->status));
	mqtt_publish_data(&mqtt_signal, unixtime, (int32_t)(WiFi.RSSI()));

	if(new_hour)
	{
		mqtt_publish_data(&mqtt_rain_hour_once, unixtime, (int32_t)data->rain_hour);
	}

	if(new_day)
	{
		mqtt_publish_data(&mqtt_rain_day_once, unixtime, (int32_t)data->rain_day);
	}

	return true;
}

bool read_sensors(readings *data)
{
	data->input_voltage = read_battery();

	data->battery_voltage = (battery.voltageLevel() * 1000);
	data->battery_soc = (battery.fuelLevel() * 1000);

	float shuntvoltage = ina219.getShuntVoltage_mV();
	float busvoltage = ina219.getBusVoltage_V();
	data->solar_voltage = (int32_t)(busvoltage * 1000) + (int32_t)shuntvoltage;
	data->solar_current = (int32_t)(ina219.getCurrent_mA() * 1000);

	uint32_t status = 0;
	status |= (digitalRead(SOLAR_CHARGER_DONE) == 0);
	status |= ((digitalRead(SOLAR_CHARGER_CHRG) == 0)<< 1);

	data->status = status;

	bme.takeForcedMeasurement();
	data->temperature = bme.readTemperature();
	data->humidity = bme.readHumidity();
	data->pressure = bme.readPressure();

	//data->ir_light = uv.readIR();
	//data->vis_light = uv.readVisible();
	//data->uv_index = uv.readUV();
	
	data->ir_light = 0;
	data->vis_light = 0;
	data->uv_index = 0;

	data->rain = compute_rain();
	data->rain_hour = compute_rain_hour();
	data->rain_day = compute_rain_day();

	return true;
}

bool calculate_sensors(readings *data)
{
	data->dew_point = (float)calculate_dew_point((double)data->humidity, 
	                                             (double)data->temperature);

	return true;
}

bool read_wind(readings *data, int cur_2m, int n, int cur_10m, int m)
{
	unsigned long t, c;

	// Do bounds checking ...
	if(cur_2m >= n)
	{
		// Error
		return false;
	}
	if(cur_10m >= m)
	{
		// Error
		return false;
	}
	
	get_wind_speed(&c, &t);
	data->wind_speed = (WINDSPEED_CALIBRATION * c);
	data->wind_speed /= t;
	data->wind_speed_2m_t[cur_2m] = t;
	data->wind_speed_2m_c[cur_2m] = c;
	data->wind_speed_10m_ave[cur_10m] = data->wind_speed;

	t = 0;
	c = 0;
	for(int i=0;i<n;i++)
	{
		t += data->wind_speed_2m_t[i];
		c += data->wind_speed_2m_c[i];
	}

	data->wind_speed_2m_ave	= (WINDSPEED_CALIBRATION * c);
	data->wind_speed_2m_ave /= t;

	uint8_t dir = get_wind_direction();	
	data->wind_direction = dir;
	data->wind_direction_2m[cur_2m] = dir;
	data->wind_direction_10m_ave[cur_10m] = dir;

	data->wind_direction_2m_ave = mean_mitsuta(data->wind_direction_2m, 120);

	// Now calculate 10m gusts
	unsigned long gust = 0;
	uint8_t gust_dir;
	for(int i=0;i<m;i++)
	{
		// Search for highest
		if(data->wind_speed_10m_ave[i] > gust)
		{
			gust = data->wind_speed_10m_ave[i];
			gust_dir = data->wind_direction_10m_ave[i];
		}
	}

	data->wind_speed_gust_10m = gust;
	data->wind_direction_gust_10m = gust_dir;

	return true;
}

void print_sensors(readings *data)
{
	Serial.print(F("Status                  = 0x"));
	Serial.println(data->status, HEX);
	Serial.print(F("Battery voltage         = "));
	Serial.print(data->battery_voltage);
	Serial.println(F(" mV"));
	Serial.print(F("Battery SOC             = "));
	Serial.print(data->battery_soc);
	Serial.println(F(" %"));
	Serial.print(F("Solar Voltage           = "));
	Serial.print(data->solar_voltage);
	Serial.println(F(" mV"));
	Serial.print(F("Solar Current           = "));
	Serial.print(data->solar_current);
	Serial.println(F(" uA"));
	Serial.print(F("Input Voltage           = "));
	Serial.print(data->input_voltage);
	Serial.println(F(" mV"));
	Serial.print(F("Wind speed              = "));
	Serial.print(data->wind_speed);
	Serial.println(F(" mph"));
	Serial.print(F("Wind direction          = "));
	if((data->wind_direction >= 0) && (data->wind_direction < 16))
	{
		Serial.print(cardinal_points[data->wind_direction]);
		Serial.print(" ");
		Serial.print(data->wind_direction * 22.5);
	} else {
		Serial.print(F("ERROR"));
	}
	Serial.println(F("	"));
	Serial.print(F("Temperature             = "));
	Serial.print(data->temperature);
	Serial.println(F(" *C"));
	Serial.print(F("Humidity                = "));
	Serial.print(data->humidity);
	Serial.println(F(" %"));
	Serial.print(F("Dew point               = "));
	Serial.print(data->dew_point);
	Serial.println(F(" *C"));
	Serial.print(F("Pressure                = "));
	Serial.print(data->pressure);
	Serial.println(F(" "));
	Serial.print(F("UV Index                = "));
	Serial.print(data->uv_index);
	Serial.println(F(" "));
	Serial.print(F("Visible light           = "));
	Serial.print(data->vis_light);
	Serial.println(F(" "));
	Serial.print(F("IR Light                = "));
	Serial.print(data->ir_light);
	Serial.println(F(" "));
	Serial.print(F("Rain                    = "));
	Serial.print(data->rain);
	Serial.println(F(" "));
	Serial.print(F("Rain Hour               = "));
	Serial.print(data->rain_hour);
	Serial.println(F(" "));
	Serial.print(F("Rain Day                = "));
	Serial.print(data->rain_day);
	Serial.println(F(" "));
	Serial.print(F("Wind Speed 2m           = "));
	Serial.print(data->wind_speed_2m_ave);
	Serial.println(F(" "));
	Serial.print(F("Wind Direction 2m       = "));
	Serial.print(data->wind_direction_2m_ave);
	Serial.println(F(" "));
	Serial.print(F("Wind speed 10m gust     = "));
	Serial.print(data->wind_speed_gust_10m);
	Serial.println(F(" "));
	Serial.print(F("Wind Direction 10m gust = "));
	Serial.print(data->wind_direction_gust_10m);
	Serial.println(F(" "));
}

bool setup_clock(void)
{
#ifdef SOFTWARE_RTC
	Serial.println(F("**** Using software RTC ****"));
	rtc.begin(DateTime(F(__DATE__), F(__TIME__)));
#else
	Serial.println(F("Using hardware RTC"));
	if(!rtc.begin()) {
		Serial.println("Error!");
		error(F("Couldn't find RTC"));
	}

	if (rtc.lostPower()) {
		Serial.println(F("RTC lost power, setting time from compilation time of sketch."));
		// following line sets the RTC to the date & time this sketch was compiled
		DateTime compile_time = DateTime(F(__DATE__), F(__TIME__));
		rtc.adjust(compile_time + TimeSpan(0, UTC_TIME_OFFSET, 0, 0));
    }

#endif
	return true;
}

bool setup_i2c_sensors(void)
{
	bool status = true;

	// Setup battery fuel guage
	Wire.begin();
	delay(100);
	battery.restart();

	ina219.begin();
	ina219.setCalibration_32V_1A();

    status = bme.begin();  
    if(!status){
        Serial.println(F("Could not find a valid BME280 sensor, check wiring!"));
        return false;
    }

	// Set the bme sensor for weather sensing

	bme.setSampling(Adafruit_BME280::MODE_FORCED,
                    Adafruit_BME280::SAMPLING_X16, // temperature
                    Adafruit_BME280::SAMPLING_X16, // pressure
                    Adafruit_BME280::SAMPLING_X16, // humidity
                    Adafruit_BME280::FILTER_OFF   );

	//status = uv.begin();
	//if(!status){
	//	Serial.println(F("Cound not find a valid SI1145 sensor, check wiring!"));
	//}

	return true;
}	

void get_wind_speed(unsigned long *clicks, unsigned long *delta)
{
	*delta = millis() - last_wind_check;
	*clicks = wind_clicks;
	wind_clicks = 0;
	last_wind_check = millis();
}	

int16_t get_wind_direction(void)
{
	unsigned int adc;

	adc = average_analog_read(WIND_DIRECTION_IO);
	// Add a value which is the "error" on the wind pin

	adc-=WIND_ADC_DELTA;

	if(adc < WIND_ADC_5)
		return 5;
	if(adc < WIND_ADC_3)
		return 3;
	if(adc < WIND_ADC_4)
		return 4;
	if(adc < WIND_ADC_7)
		return 7;
	if(adc < WIND_ADC_6)
		return 6;
	if(adc < WIND_ADC_9)
		return 9;
	if(adc < WIND_ADC_8)
		return 8;
	if(adc < WIND_ADC_1)
		return 1;
	if(adc < WIND_ADC_2)
		return 2;
	if(adc < WIND_ADC_11)
		return 11;
	if(adc < WIND_ADC_10)
		return 10;
	if(adc < WIND_ADC_15)
		return 15;
	if(adc < WIND_ADC_0)
		return 0;
	if(adc < WIND_ADC_13)
		return 13;
	if(adc < WIND_ADC_14)
		return 14;
	if(adc < WIND_ADC_12)
		return 12;
	
	return -1;
}

uint16_t average_analog_read(int pin)
{
	byte nr = 16;
	uint16_t running_value = 0;

	for(int x=0;x<nr;x++)
	{
		running_value += analogRead(pin);
	}	

	return running_value / nr;
}

void print_clock(DateTime *t)
{
	Serial.print(t->year(), DEC);
    Serial.print('/');
    Serial.print(t->month(), DEC);
    Serial.print('/');
    Serial.print(t->day(), DEC);
    Serial.print(" (");
    Serial.print(days_of_the_week[t->dayOfTheWeek()]);
    Serial.print(") ");
    Serial.print(t->hour(), DEC);
    Serial.print(':');
    Serial.print(t->minute(), DEC);
    Serial.print(':');
    Serial.print(t->second(), DEC);
    Serial.println();
}

uint32_t read_battery(void)
{	
	uint32_t bat = average_analog_read(BATTERY_IO);
	bat *= (3300 * 2); // 3.3V ref, divide/2 
	bat /= 1023; // ADC full ragne
	return bat;
}


void setup_wifi(void)
{

	WiFi.setPins(8,7,4,2);
	if (WiFi.status() == WL_NO_SHIELD) {
		error(F("WiFi module not present"));
	}
	//WiFi.config(wifi_ip, wifi_gateway, wifi_gateway, wifi_mask);

	Watchdog.reset();
	Serial.println(F("Waiting 5s for descovery of networks"));
	delay(5000);
	
	Watchdog.reset();
	wifi_list_networks();

	Watchdog.reset();
}

void wifi_connect() {

	if(WiFi.status() == WL_CONNECTED)
	{
		return;
	}

	Watchdog.disable();

	int tries = 50;
	while(WiFi.status() != WL_CONNECTED)
	{
		Serial.print(F("Attempting to connect to WPA SSID: "));
		Serial.println(wifi_ssid);
		WiFi.begin(wifi_ssid, wifi_password);
		if(--tries == 0)
		{
			Serial.println(F("Enabling watchdog"));
			Watchdog.enable(WATCHDOG_TIME);
		}
	}
	
	Serial.println(F("Waiting 10s for WIFI to connect"));
	delay(10000);

	// start the WiFi OTA library with SD based storage
	Serial.println(F("Setup OTA Programming ....."));
	WiFiOTA.begin(OTA_CONNECTION_NAME, ota_password, InternalStorage);

	// Start telnet port server
	wifi_server.begin();

	// Re-enable the watchdog
	Watchdog.enable(WATCHDOG_TIME);
}

void mqtt_connect() {

	if(mqtt_client.connected())
	{
		return;
	} else {
		mqtt_client.disconnect();
	}

	Serial.println(F("Connecting to MQTT Server ....."));

	int ping_result;
	while(WiFi.ping(mqtt_server) < 0)
	{
		Serial.println("Waiting to ping server....");
		delay(1000);
	}

	Watchdog.disable();
	int8_t ret;
	int tries = 50;
	while((ret = mqtt_client.connect()) != 0) { // connect will return 0 for connected
		Serial.println(mqtt_client.connectErrorString(ret));
		Serial.print("Retrying MQTT connection ... tries = ");
		Serial.println(tries);
		mqtt_client.disconnect();

		// Check if WiFi Is connected.
		if(WiFi.status() != WL_CONNECTED)
		{
			// We need to start WiFi
			wifi_connect();
			Watchdog.disable();
		}
		if(--tries == 0)
		{
			Serial.println(F("Enabling watchdog"));
			Watchdog.enable(WATCHDOG_TIME);
		}
	}

	Serial.println("MQTT Connected!");
	// Re-enable the watchdog
	Watchdog.enable(WATCHDOG_TIME);
}

void mqtt_publish_data(Adafruit_MQTT_Publish *pub, uint32_t timestamp, int32_t val)
{
    uint8_t _val[8];

    _val[4] = (val >> 24) & 0xFF;
    _val[5] = (val >> 16) & 0xFF;
    _val[6] = (val >> 8) & 0xFF;
    _val[7] = val & 0xFF;

    _val[0] = (timestamp >> 24) & 0xFF;
    _val[1] = (timestamp >> 16) & 0xFF;
    _val[2] = (timestamp >> 8) & 0xFF;
    _val[3] = timestamp & 0xFF;

    pub->publish(_val, 8);
}

double calculate_dew_point(double RH, double T)
{
	double H, Dp;

	H = (log10(RH)-2)/0.4343 + (17.62*T)/(243.12+T);
	Dp = 243.12 * H / (17.62 - H);

	return Dp;
}

float mean_mitsuta(int8_t *bearing, int N)
{
	float sum, D, delta;
	D = ((float)bearing[0] * 22.5);
	sum = D;

	for(int i=1;i<N;i++)
	{
		delta = ((float)bearing[i] * 22.5) - D;
		if(delta < -180)
		{
			D += delta + 360;
		} else if( delta < 180) {
			D += delta;
		} else {
			D += delta - 360;
		}

		sum += D;
	}

	sum /= N;

	if(sum >= 360)
	{
		sum -= 360;
	}

	if(sum < 0)
	{
		sum += 360;
	}

	return sum;
}	

void wifi_list_networks() {
	// scan for nearby networks:
	Serial.println("** Scan Networks **");
	byte numSsid = WiFi.scanNetworks();

	// print the list of networks seen:
	Serial.print("number of available networks:");
	Serial.println(numSsid);

	// print the network number and name for each network found:
	for (int thisNet = 0; thisNet<numSsid; thisNet++) {
		Serial.print(thisNet);
		Serial.print(") ");
		Serial.print(WiFi.SSID(thisNet));
		Serial.print("\tSignal: ");
		Serial.print(WiFi.RSSI(thisNet));
		Serial.print(" dBm");
		Serial.print("\tEncryption: ");
		Serial.println(WiFi.encryptionType(thisNet));
	}
}
