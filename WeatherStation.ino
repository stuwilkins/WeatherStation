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
#include <PubSubClient.h>

#include <Adafruit_SI1145.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_INA219.h>
#include <Adafruit_SleepyDog.h>

#include <MAX17043GU.h>
#include <RTClib.h>
//#include <PWFusion_AS3935.h>
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
int16_t this_3hr = 0.0;

int update_counter;

// Global variables to keep track of accumulations.

uint8_t current_minute = 0;	
int32_t wind_direction_total = 0;
int32_t wind_direction_n = 0;
DateTime last;

// Hardware definitions

Adafruit_BME280 bme;
Adafruit_SI1145 uv;
MAX17043GU battery;
Adafruit_INA219 ina219;
//PWF_AS3935 lightning0(AS3935_CS, AS3935_IRQ, AS3935_SI);

RTC_DS3231 rtc;

readings data;

// MQTT Setup
WiFiClient wifi_client;
PubSubClient mqtt_client(wifi_client);

static const char* mqtt_battery                 = "homeauto/weather/battery";
static const char* mqtt_temperature             = "homeauto/weather/temperature";
static const char* mqtt_humidity                = "homeauto/weather/humidity";
static const char* mqtt_pressure                = "homeauto/weather/pressure";
static const char* mqtt_vis_light               = "homeauto/weather/vis_light";
static const char* mqtt_ir_light                = "homeauto/weather/ir_light";
static const char* mqtt_uv_index                = "homeauto/weather/uv_index";
static const char* mqtt_wind_speed              = "homeauto/weather/wind_speed";
static const char* mqtt_wind_direction          = "homeauto/weather/wind_direction";
static const char* mqtt_rain_hour               = "homeauto/weather/rain_hour";
static const char* mqtt_rain_hour_once          = "homeauto/weather/rain_hour_once";
static const char* mqtt_rain_day                = "homeauto/weather/rain_day";
static const char* mqtt_rain_day_once           = "homeauto/weather/rain_day_once";
static const char* mqtt_dew_point               = "homeauto/weather/dew_point";
static const char* mqtt_wind_speed_2m_ave       = "homeauto/weather/wind_speed_2m_ave";
static const char* mqtt_wind_direction_2m_ave   = "homeauto/weather/wind_direction_2m_ave";
static const char* mqtt_wind_speed_10m_gust     = "homeauto/weather/wind_speed_10m_gust";
static const char* mqtt_wind_direction_10m_gust = "homeauto/weather/wind_direction_10m_gust";
static const char* mqtt_solar_voltage           = "homeauto/weather/solar_voltage";
static const char* mqtt_solar_current           = "homeauto/weather/solar_current";
static const char* mqtt_battery_soc             = "homeauto/weather/battery_soc";
static const char* mqtt_lightning_distance      = "homeauto/weather/lightning_distance";
static const char* mqtt_lightning_energy        = "homeauto/weather/lightning_energy";
static const char* mqtt_input_voltage           = "homeauto/weather/input_voltage";
static const char* mqtt_status                  = "homeauto/weather/status";
static const char* mqtt_signal                  = "homeauto/weather/signal";
static const char* mqtt_loop_time               = "homeauto/weather/loop_time";
static const char* mqtt_pressure_trend          = "homeauto/weather/pressure_trend";

static const char* mqtt_wifi_power_sp           = "homeauto/weather/wifi_power_sp";
static const char* mqtt_subscribe[]             = {mqtt_wifi_power_sp, 0};

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

	for(int i=0;i<180;i++)
	{
		data.pressure_3hr[i] = 0.0;
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
	Watchdog.reset();

	Serial.print(F("Setup MQTT ...."));
	Serial.print(MQTT_SERVER);
	Serial.print(F(" "));
	Serial.println(MQTT_PORT);
	mqtt_client.setServer(MQTT_SERVER, MQTT_PORT);
	mqtt_client.setCallback(mqtt_callback);
	mqtt_connect();
	Watchdog.reset();

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
	this_10minute = (now.minute() % 10) * 60;
	this_10minute += now.second();
	this_3hr = (now.hour() % 3) * 60;
	this_3hr += now.minute();

	if((last.hour() == (UTC_TIME_OFFSET - 1)) && 
	   (now.hour() == UTC_TIME_OFFSET))
	{
		// New Day!
		rain_day_total = 0;
		new_day = true;
		Serial.println("New Day .... ");
	}

	if(last.minute() < now.minute())
	{
		// New Minute
		new_minute = true;
		rain_hour_total[this_minute] = 0;
		Serial.println("New Minute .... ");
	}

	if(last.hour() < now.hour())
	{
		// New Hour!
		new_hour = true;	
		Serial.println("New Hour .... ");
	}

	if(last.second() < now.second())
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

	if( new_second ||
		new_day || 
		new_hour || 
		new_minute)
	{
		read_sensors(&data);
		read_wind(&data, this_2minute, 120, this_10minute, 600);
	
		calculate_sensors(&data);
		//print_sensors(&data);

		if(new_minute)
		{
			// Do the pressure trend
			data.pressure_3hr[this_3hr] = data.pressure;

			DateTime start = now - TimeSpan(0, 3, 0, 0);
			int16_t last_3hr = (start.hour() % 3) * 60;
			last_3hr += start.minute();

			float pressure_diff = data.pressure_3hr[this_3hr] - data.pressure_3hr[last_3hr];
			if(pressure_diff >= 2)
			{
				data.pressure_trend = PRESSURE_RAPIDLY_RISING;
			}

			if((pressure_diff >= 1) && (pressure_diff < 2))
			{
				data.pressure_trend = PRESSURE_RISING;
			} 
			
			if(abs(pressure_diff) < 1)
			{
				data.pressure_trend = PRESSURE_STEADY;
			}

			if((pressure_diff <= 1) && (pressure_diff > -2))
			{
				data.pressure_trend = PRESSURE_FALLING;
			}

			if(pressure_diff <= -2) 
			{
				data.pressure_trend = PRESSURE_RAPIDLY_FALLING;
			}
			
		}

		if(((now.second() % 2) == 0) || new_minute || new_hour || new_day)
		{
			write_sensors(&data, now, new_minute, new_hour, new_day);
			mqtt_publish_data(mqtt_loop_time, now.unixtime(), (int32_t)(millis() - start_millis), 0);
		}

		digitalWrite(BUILTIN_LED_IO, !digitalRead(BUILTIN_LED_IO));
	}
	
	last = now;
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
	mqtt_publish_data(mqtt_lightning_distance, unixtime, (int32_t)data->lightning_distance, 0);
	mqtt_publish_data(mqtt_lightning_energy, unixtime, (int32_t)data->lightning_energy, 0);
		
	return true;
}

bool write_sensors(readings *data, DateTime ts, bool new_minute, bool new_hour, bool new_day)
{
	uint32_t unixtime = ts.unixtime();
	mqtt_publish_data(mqtt_battery, unixtime, (int32_t)data->battery_voltage, 0);
	mqtt_publish_data(mqtt_battery_soc, unixtime, (int32_t)data->battery_soc, 0);
	mqtt_publish_data(mqtt_temperature, unixtime, (int32_t)(data->temperature * 1000), 0);
	mqtt_publish_data(mqtt_humidity, unixtime, (int32_t)(data->humidity * 1000), 0);
	mqtt_publish_data(mqtt_pressure, unixtime, (int32_t)(data->pressure * 10), 0);
	mqtt_publish_data(mqtt_uv_index, unixtime, (int32_t)(data->uv_index), 0);
	mqtt_publish_data(mqtt_vis_light, unixtime, (int32_t)data->vis_light, 0);
	mqtt_publish_data(mqtt_ir_light, unixtime, (int32_t)data->ir_light, 0);
	mqtt_publish_data(mqtt_wind_speed, unixtime, (int32_t)(data->wind_speed), 0);
	mqtt_publish_data(mqtt_wind_direction, unixtime, (int32_t)(data->wind_direction * 22500), 0);
	mqtt_publish_data(mqtt_rain_hour, unixtime, (int32_t)data->rain_hour, 0);
	mqtt_publish_data(mqtt_rain_day, unixtime, (int32_t)data->rain_day, 0);
	mqtt_publish_data(mqtt_dew_point, unixtime, (int32_t)(data->dew_point * 1000), 0);
	mqtt_publish_data(mqtt_wind_direction_2m_ave, unixtime, (int32_t)(data->wind_direction_2m_ave * 1000), 0);
	mqtt_publish_data(mqtt_wind_speed_2m_ave, unixtime, (int32_t)(data->wind_speed_2m_ave), 0);
	mqtt_publish_data(mqtt_wind_speed_10m_gust, unixtime, (int32_t)(data->wind_speed_gust_10m), 0);
	mqtt_publish_data(mqtt_wind_direction_10m_gust, unixtime, (int32_t)(data->wind_direction_gust_10m * 22500), 0);
	mqtt_publish_data(mqtt_solar_voltage, unixtime, (int32_t)(data->solar_voltage), 0);
	mqtt_publish_data(mqtt_solar_current, unixtime, (int32_t)(data->solar_current), 0);
	mqtt_publish_data(mqtt_input_voltage, unixtime, (int32_t)(data->input_voltage), 0);
	mqtt_publish_data(mqtt_status, unixtime, (int32_t)(data->status), 0);
	mqtt_publish_data(mqtt_signal, unixtime, (int32_t)(WiFi.RSSI()), 0);

	if(new_hour)
	{
		mqtt_publish_data(mqtt_rain_hour_once, unixtime, (int32_t)data->rain_hour, 1);
	}

	if(new_day)
	{
		mqtt_publish_data(mqtt_rain_day_once, unixtime, (int32_t)data->rain_day, 1);
	}

	if(new_minute)
	{
		mqtt_publish_data(mqtt_pressure_trend, unixtime, (int32_t)data->pressure_trend, 1);
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
	uint8_t gust_dir = 0;
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
	Serial.print(F("Signal Strength         = "));
	Serial.print(WiFi.RSSI());
	Serial.println(F(" dBm"));
}

bool setup_clock(void)
{
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

	WiFi.lowPowerMode();

	Watchdog.reset();
	Serial.println(F("Waiting 5s for descovery of networks"));
	delay(5000);
	
	Watchdog.reset();
	wifi_list_networks();

	Watchdog.reset();
}

void wifi_connect()
{

	if(WiFi.status() == WL_CONNECTED)
	{
		return;
	}

	Watchdog.disable();

	int tries = 5;
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
		delay(5000);
	}
	
	// start the WiFi OTA library with Internal Based storage
	Serial.println(F("Setup OTA Programming ....."));
	WiFiOTA.begin(OTA_CONNECTION_NAME, ota_password, InternalStorage);

	// Re-enable the watchdog
	Watchdog.enable(WATCHDOG_TIME);
}

void mqtt_connect()
{

	if(mqtt_client.connected())
	{
		return;
	}

	Watchdog.disable();

	int tries = 50;
	while(!mqtt_client.connected())
	{
		Serial.print("Attempting MQTT connection...");
		if(mqtt_client.connect(MQTT_CLIENT_NAME))
		{
			Serial.println(F("Connected"));
			int i = 0;
			while(mqtt_subscribe[i] != 0)
			{
				mqtt_client.subscribe(mqtt_subscribe[i], 1);
				i++;
			}
		} else {
			Serial.print(F("Connection failed, rc="));
			Serial.print(mqtt_client.state());
			Serial.println(F(""));
			// Wait 5 seconds before retrying
			if(--tries == 0)
			{
				Serial.println(F("Enabling watchdog"));
				Watchdog.enable(WATCHDOG_TIME);
			}
		}
	}

	// Re-enable the watchdog
	Watchdog.enable(WATCHDOG_TIME);
}

void mqtt_publish_data(const char *pub, uint32_t timestamp, int32_t val, int persist)
{
    byte _val[8];

    _val[4] = (val >> 24) & 0xFF;
    _val[5] = (val >> 16) & 0xFF;
    _val[6] = (val >> 8) & 0xFF;
    _val[7] = val & 0xFF;

    _val[0] = (timestamp >> 24) & 0xFF;
    _val[1] = (timestamp >> 16) & 0xFF;
    _val[2] = (timestamp >> 8) & 0xFF;
    _val[3] = timestamp & 0xFF;

	// Publish and make persistent 
    mqtt_client.publish(pub, _val, 8, persist);
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

void wifi_list_networks()
{
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

void mqtt_callback(char* topic, byte* payload, unsigned int length)
{
	Serial.print(F("**** Message arrived ["));
	Serial.print(topic);
	Serial.print(F("] "));

	if(!strcmp(topic, mqtt_wifi_power_sp))
	{
		if(!strncmp((const char*)payload, "OFF", length))
		{
			WiFi.noLowPowerMode(); 
		} else if(!strncmp((const char*)payload, "ON", length)) {
			WiFi.lowPowerMode();
		}
	}
}

