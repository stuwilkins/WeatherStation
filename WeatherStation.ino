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
#include <eeprom_i2c.h>
#include <RemoteConsole.h>

#include <Adafruit_SI1145.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_INA219.h>
#include <Adafruit_SleepyDog.h>

#include <MAX17043GU.h>
#include <RTClib.h>
#include <PWFusion_AS3935_I2C.h>
#include <math.h>
#include <NTPClient.h>

#include "WeatherStation.h"
#include "auth.h"

// Global Variables	

volatile unsigned long wind_last = 0;
volatile unsigned long wind_clicks = 0;
unsigned long last_wind_check = 0;

volatile uint32_t rain_time;
volatile uint32_t rain_last;
volatile uint32_t rain_interval;
volatile uint16_t rain_total = 0;

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
PWF_AS3935_I2C lightning0(AS3935_IRQ, AS3935_ADD);
EEPROM_I2C eeprom = EEPROM_I2C(0x50);
RTC_DS3231 rtc;
RemoteConsole console(10000);
WiFiUDP ntp_udp;
NTPClient ntp_client(ntp_udp, NTP_ADDRESS, NTP_OFFSET, NTP_INTERVAL);

// Data for storing sensor readings

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
static const char* mqtt_rain                    = "homeauto/weather/rain";
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
static const char* mqtt_pressure_trend_val      = "homeauto/weather/pressure_trend_val";

static const char* mqtt_wifi_power_sp           = "homeauto/weather/wifi_power_sp";
static const char* mqtt_eeprom_reset_sp         = "homeauto/weather/eeprom_reset_sp";
static const char* mqtt_reset_sp                = "homeauto/weather/reset_sp";
static const char* mqtt_subscribe[]             = {mqtt_wifi_power_sp, 
                                                   mqtt_reset_sp, 
												   mqtt_eeprom_reset_sp, 0};

// Error handler 

void error(const __FlashStringHelper *err) {
	console.println(err);
	while(1)
	{
		digitalWrite(BUILTIN_LED_IO, !digitalRead(BUILTIN_LED_IO));
		delay(100);
	}
}

// ISR Routines for wind and rain sensors

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
		rain_last = rain_time;
	}
}

// Setup lightning sensor

void setup_lightning(void)
{
	lightning0.begin();
	lightning0.AS3935_ManualCal(AS3935_CAPACITANCE, AS3935_OUTDOORS, AS3935_DIST_EN);
}

// Setup Routine

void setup() {


	pinMode(ANEMOMETER_IO, INPUT_PULLUP);
	pinMode(RAIN_IO, INPUT_PULLUP);
	pinMode(SOLAR_CHARGER_CHRG, INPUT_PULLUP);
	pinMode(SOLAR_CHARGER_DONE, INPUT_PULLUP);
	pinMode(BUILTIN_LED_IO, OUTPUT);

	digitalWrite(BUILTIN_LED_IO, 1);

	console.begin(115200, 0);

	Watchdog.enable(WATCHDOG_TIME);

	// Setup WIFI

	console.println(F("Setup WIFI ...."));
	setup_wifi();
	wifi_connect();
	console.connect();
	Watchdog.reset();

	console.print(F("Setup MQTT ...."));
	console.print(MQTT_SERVER);
	console.print(F(" "));
	console.println(MQTT_PORT);
	mqtt_client.setServer(MQTT_SERVER, MQTT_PORT);
	mqtt_client.setCallback(mqtt_callback);
	mqtt_connect();
	Watchdog.reset();

	// Setup wind speed and rain interrupts

	rain_last = millis();
	wind_last = millis();

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

	Watchdog.reset();

	// Setup Rain
	rain_total = 0;

	// Setup eeprom
	console.println(F("Setup eeprom ...."));
	eeprom.begin();
	Watchdog.reset();

	update_counter = 0;

	// Setup lightning
	//setup_lightning();

	// Setup RTC
	console.println(F("Setup clock ...."));
	setup_clock();
	Watchdog.reset();

	// Now setup sensors
	console.println(F("Setup sensors ...."));
	setup_i2c_sensors();
	Watchdog.reset();

	console.println("Setting up interrupt handlers ...");

	attachInterrupt(digitalPinToInterrupt(ANEMOMETER_IO), wind_speed_ISR, FALLING);	
	attachInterrupt(digitalPinToInterrupt(RAIN_IO), rainfall_ISR, FALLING);	
	Watchdog.reset();

	console.println(F("Setting low power mode for WiFi ....."));
	WiFi.lowPowerMode();

	console.println(F("Finished setup ....."));
	Watchdog.reset();
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

	// Poll for MQTT updates
	mqtt_client.loop();

	// Loop through the remote colsole....
	console.loop();

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

	if((now.hour() == 0) && (last.hour() != 0))
	{
		// New Day!
		new_day = true;
		console.println("New Day .... ");
	}

	if(last.minute() != now.minute())
	{
		// New Minute
		console.println("New Minute .... ");
		console.print(F("Hour = "));
		console.print(now.hour());
		console.print(F(" Minute = "));
		console.println(now.minute());

		new_minute = true;
	
		int day_minute = (now.hour() * 60) + now.minute();

		int rc = eeprom.writeAndVerify(EEPROM_RAIN + (day_minute * sizeof(rain_total)), 
				          			  (uint8_t *)(&rain_total), sizeof(rain_total));
		if(rc)
		{
			console.print(F("Could not write rain data rc = "));
			console.println(rc);
		}

		rain_total = 0;

		compute_rain(&data, day_minute);
	}

	if(last.hour() != now.hour())
	{
		// New Hour!
		new_hour = true;	
		console.println("New Hour .... ");
	}

	if(last.second() != now.second())
	{
		// New Second !
		new_second = true;
		console.println("New Second .... ");
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
			calculate_pressure_trend(&data, this_3hr);	
		}

		if(((now.second() % 2) == 0) || new_minute || new_hour || new_day)
		{
			unsigned long loop_time = millis() - start_millis;
			write_sensors(&data, now, new_minute, new_hour, new_day);
			mqtt_publish_data(mqtt_loop_time, now.unixtime() - NTP_OFFSET, (int32_t)loop_time, 0);
			console.print(F("Tick tock .... "));
			console.println(loop_time);
		}

		digitalWrite(BUILTIN_LED_IO, !digitalRead(BUILTIN_LED_IO));
	}
	
	last = now;
}

bool calculate_pressure_trend(readings *data, int16_t this_3hr)
{
	// Do the pressure trend
	eeprom.writeAndVerify(EEPROM_PRESSURE + (this_3hr * sizeof(float)),
			(uint8_t*)(&data->pressure), sizeof(float));

	int16_t last_3hr = this_3hr + 1;
	if(last_3hr >= 180)
	{
		last_3hr = 0;
	}

	float last_pressure;
	eeprom.read(EEPROM_PRESSURE + (last_3hr * sizeof(float)),
			(uint8_t *)(&last_pressure), sizeof(float));
	float pressure_diff = data->pressure - last_pressure;

	if(pressure_diff >= 200)
	{
		data->pressure_trend = PRESSURE_RAPIDLY_RISING;
	}

	if((pressure_diff >= 100) && (pressure_diff < 200))
	{
		data->pressure_trend = PRESSURE_RISING;
	} 

	if(abs(pressure_diff) < 100)
	{
		data->pressure_trend = PRESSURE_STEADY;
	}

	if((pressure_diff <= -100) && (pressure_diff > -200))
	{
		data->pressure_trend = PRESSURE_FALLING;
	}

	if(pressure_diff <= -200) 
	{
		data->pressure_trend = PRESSURE_RAPIDLY_FALLING;
	}

	if(pressure_diff >= 50000)
	{
		data->pressure_trend = PRESSURE_NO_DATA;
	}

	data->pressure_trend_val = pressure_diff;

	return true;
}


bool read_lightning(readings *data)
{
	uint8_t int_src = lightning0.AS3935_GetInterruptSrc();
	if(0 == int_src)
	{
		console.println(F("read_lightning() : interrupt source result not expected"));
		return false;
	}
	else if(1 == int_src)
	{
		uint8_t lightning_dist_km = lightning0.AS3935_GetLightningDistKm();
		uint32_t lightning_energy = lightning0.AS3935_GetStrikeEnergyRaw();

		data->lightning_distance = (uint32_t)lightning_dist_km;
		data->lightning_energy = (uint32_t)lightning_energy;

		console.print(F("read_lightning() : Lightning detected! Distance to strike: "));
		console.print(lightning_dist_km);
		console.println(F(" kilometers"));

		return true;
	}
	else if(2 == int_src)
	{
		data->lightning_distance = 0;
		data->lightning_energy = 0;
		console.println(F("read_lightning() : Disturber detected"));
		return false;
	}
	else if(3 == int_src)
	{
		console.println(F("read_lightning() : Noise level too high"));
		return false;
	}

	return true;
}

void compute_rain(readings *data, int curr_min)
{
	// Compute rainfall for day
	uint32_t rain_day = 0;
	uint32_t rain_hour = 0;
	uint32_t rain = 0;

	int hr_start = curr_min - 60;

	console.println(F("compute_rain()"));
	console.print(F("curr_min = "));
	console.print(curr_min);
	console.print(F(" hr_start = "));
	console.println(hr_start);

	for(int i=0;i<1440;i++)
	{
		uint16_t _rain;
		eeprom.read(EEPROM_RAIN + (sizeof(_rain) * i),
		            (uint8_t*)(&_rain), sizeof(_rain));

		if(i <= curr_min)
		{
			rain_day += _rain;
		}

		if((i > hr_start) && (i <= curr_min)) 
		{
			rain_hour += _rain;
			//console.print(F("Rain hr = "));
			//console.println(i);
		}

		if(i == curr_min)
		{
			rain = _rain;
		}

		if(i > (hr_start + 1440))
		{
			//console.print(F("Rain hr (w) = "));
			//console.println(i);
			rain_hour += _rain;
		}
	}

	data->rain_day = rain_day * RAIN_BUCKETS_TO_UM;
	data->rain_hour = rain_hour * RAIN_BUCKETS_TO_UM;
	data->rain = rain * RAIN_BUCKETS_TO_UM;
}

bool write_lightning(readings *data, DateTime ts)
{
	console.println("write_lightning()");

	uint32_t unixtime = ts.unixtime() - NTP_OFFSET;
	mqtt_publish_data(mqtt_lightning_distance, unixtime, (int32_t)data->lightning_distance, 0);
	mqtt_publish_data(mqtt_lightning_energy, unixtime, (int32_t)data->lightning_energy, 0);
		
	return true;
}

bool write_sensors(readings *data, DateTime ts, bool new_minute, bool new_hour, bool new_day)
{
	uint32_t unixtime = ts.unixtime() - NTP_OFFSET;
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
		mqtt_publish_data(mqtt_pressure_trend_val, unixtime, (int32_t)(data->pressure_trend_val * 10), 0);
		mqtt_publish_data(mqtt_rain_day, unixtime, (int32_t)data->rain_day, 0);
		mqtt_publish_data(mqtt_rain_hour, unixtime, (int32_t)data->rain_hour, 0);
		mqtt_publish_data(mqtt_rain, unixtime, (int32_t)data->rain, 0);
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
	console.print(F("Status                  = 0x"));
	console.println(data->status, HEX);
	console.print(F("Battery voltage         = "));
	console.print(data->battery_voltage);
	console.println(F(" mV"));
	console.print(F("Battery SOC             = "));
	console.print(data->battery_soc);
	console.println(F(" %"));
	console.print(F("Solar Voltage           = "));
	console.print(data->solar_voltage);
	console.println(F(" mV"));
	console.print(F("Solar Current           = "));
	console.print(data->solar_current);
	console.println(F(" uA"));
	console.print(F("Input Voltage           = "));
	console.print(data->input_voltage);
	console.println(F(" mV"));
	console.print(F("Wind speed              = "));
	console.print(data->wind_speed);
	console.println(F(" mph"));
	console.print(F("Wind direction          = "));
	if((data->wind_direction >= 0) && (data->wind_direction < 16))
	{
		console.print(cardinal_points[data->wind_direction]);
		console.print(" ");
		console.print(data->wind_direction * 22.5);
	} else {
		console.print(F("ERROR"));
	}
	console.println(F("	"));
	console.print(F("Temperature             = "));
	console.print(data->temperature);
	console.println(F(" *C"));
	console.print(F("Humidity                = "));
	console.print(data->humidity);
	console.println(F(" %"));
	console.print(F("Dew point               = "));
	console.print(data->dew_point);
	console.println(F(" *C"));
	console.print(F("Pressure                = "));
	console.print(data->pressure);
	console.println(F(" "));
	console.print(F("UV Index                = "));
	console.print(data->uv_index);
	console.println(F(" "));
	console.print(F("Visible light           = "));
	console.print(data->vis_light);
	console.println(F(" "));
	console.print(F("IR Light                = "));
	console.print(data->ir_light);
	console.println(F(" "));
	console.print(F("Rain                    = "));
	console.print(data->rain);
	console.println(F(" "));
	console.print(F("Rain Hour               = "));
	console.print(data->rain_hour);
	console.println(F(" "));
	console.print(F("Rain Day                = "));
	console.print(data->rain_day);
	console.println(F(" "));
	console.print(F("Wind Speed 2m           = "));
	console.print(data->wind_speed_2m_ave);
	console.println(F(" "));
	console.print(F("Wind Direction 2m       = "));
	console.print(data->wind_direction_2m_ave);
	console.println(F(" "));
	console.print(F("Wind speed 10m gust     = "));
	console.print(data->wind_speed_gust_10m);
	console.println(F(" "));
	console.print(F("Wind Direction 10m gust = "));
	console.print(data->wind_direction_gust_10m);
	console.println(F(" "));
	console.print(F("Signal Strength         = "));
	console.print(WiFi.RSSI());
	console.println(F(" dBm"));
}

bool setup_clock(void)
{
	if(!rtc.begin()) {
		error(F("Couldn't find RTC"));
	}

	set_clock();

	return true;
}

void set_clock(void)
{
	ntp_client.update();
	DateTime now = rtc.now();
	unsigned long epoch_ntp = ntp_client.getEpochTime();
	unsigned long epoch_rtc = now.unixtime();

	long diff = epoch_ntp - epoch_rtc;

	console.print(F("**** RTC reports time as "));
	console.println(epoch_rtc);
	console.print(F("**** NTP reports time as "));
	console.println(epoch_ntp);
	console.print(F("**** RTC vs NTP diff is "));
	console.println(diff);

	rtc.adjust(epoch_ntp);
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
        console.println(F("Could not find a valid BME280 sensor, check wiring!"));
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
	//	console.println(F("Cound not find a valid SI1145 sensor, check wiring!"));
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
	console.print(t->year(), DEC);
    console.print('/');
    console.print(t->month(), DEC);
    console.print('/');
    console.print(t->day(), DEC);
    console.print(" (");
    console.print(days_of_the_week[t->dayOfTheWeek()]);
    console.print(") ");
    console.print(t->hour(), DEC);
    console.print(':');
    console.print(t->minute(), DEC);
    console.print(':');
    console.print(t->second(), DEC);
    console.println();
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

	WiFi.noLowPowerMode();

	Watchdog.reset();
	console.println(F("Waiting 5s for descovery of networks"));
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
		console.print(F("Attempting to connect to WPA SSID: "));
		console.println(wifi_ssid);
		WiFi.begin(wifi_ssid, wifi_password);
		if(--tries == 0)
		{
			console.println(F("Enabling watchdog"));
			Watchdog.enable(WATCHDOG_TIME);
		}
		delay(5000);
	}
	
	// start the WiFi OTA library with Internal Based storage
	console.println(F("Setup OTA Programming ....."));
	WiFiOTA.begin(ota_name, ota_password, InternalStorage);

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
		console.print("Attempting MQTT connection...");
		if(mqtt_client.connect(MQTT_CLIENT_NAME))
		{
			console.println(F("Connected"));
			int i = 0;
			while(mqtt_subscribe[i] != 0)
			{
				mqtt_client.subscribe(mqtt_subscribe[i], 1);
				console.print(F("Subscribing to : "));
				console.println(mqtt_subscribe[i]);
				i++;
			}
		} else {
			console.print(F("Connection failed, rc="));
			console.print(mqtt_client.state());
			console.println(F(""));
			// Wait 5 seconds before retrying
			if(--tries == 0)
			{
				console.println(F("Enabling watchdog"));
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
	console.println("**** Scan Networks ****");
	byte numSsid = WiFi.scanNetworks();

	// print the list of networks seen:
	console.print("Number of available networks: ");
	console.println(numSsid);

	// print the network number and name for each network found:
	for (int thisNet = 0; thisNet<numSsid; thisNet++) {
		console.print(thisNet);
		console.print(") ");
		console.print(WiFi.SSID(thisNet));
		console.print("\tSignal: ");
		console.print(WiFi.RSSI(thisNet));
		console.print(" dBm");
		console.print("\tEncryption: ");
		console.println(WiFi.encryptionType(thisNet));
	}
}

void mqtt_callback(char* topic, byte* payload, unsigned int length)
{
	console.print(F("**** Message arrived ["));
	console.print(topic);
	console.println(F("] "));

	if(!strcmp(topic, mqtt_wifi_power_sp))
	{
		if(!strncmp((const char*)payload, "HIGH", length))
		{
			WiFi.noLowPowerMode(); 
			console.println(F("*** Setting WIFI Power to HIGH ***"));
		} else if(!strncmp((const char*)payload, "LOW", length)) {
			WiFi.lowPowerMode();
			console.println(F("*** Setting WIFI Power to LOW ***"));
		}
	} else if (!strcmp(topic, mqtt_eeprom_reset_sp)) {
		if(length == 1)
		{
			if(payload[0])
			{
				eeprom_reset();
			}
		}
	} else if (!strcmp(topic, mqtt_reset_sp)) {
		if(length == 1)
		{
			if(payload[0])
			{
				error(F("RESET"));
			}
		}
	}
}

void eeprom_reset(void)
{
	float fzero = 0.0;
	uint16_t izero = 0;

	console.println(F("**** RESETTING EEPROM ****"));

	for(int i=0; i<180; i++)
	{
		int rc;
		if((rc = eeprom.writeAndVerify(EEPROM_PRESSURE + (i * sizeof(float)), 
			                           (uint8_t *)(&fzero), 
			                           sizeof(float))))
		{
			console.print(F("Error writing to EEPROM rc = "));
			console.println(rc);
		}
		Watchdog.reset();
	}

	for(int i=0; i<1440; i++)
	{
		int rc;
		if((rc = eeprom.writeAndVerify(EEPROM_RAIN + (i * sizeof(int16_t)), 
			                           (uint8_t *)(&izero), 
			                           sizeof(uint16_t))))
		{
			console.print(F("Error writing to EEPROM rc = "));
			console.println(rc);
		}
		Watchdog.reset();
	}
}

