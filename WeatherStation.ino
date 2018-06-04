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
#include <RTClib.h>

#include "BluefruitConfig.h"
#include "BluetoothConfig.h"
#include "battery.h"

#define ANEMOMETER_IO 			0
#define RAIN_IO       			1
#define WIND_DIRECTION_IO		A0
#define I2C_ENABLE_IO			5
#define CLOCK_ENABLE_IO			6
#define BUILTIN_LED_IO			13
#define BATTERY_IO    			A9

// Calibration constants

// For windspeed 1 click.s^-1 = 1.492 MPH
#define WINDSPEED_CALIBRATION 	1.492
#define SEALEVELPRESSURE_HPA    (1013.25)
#define UTC_TIME_OFFSET	        4

// Global Variables	

volatile long wind_last = 0;
volatile byte wind_clicks = 0;
uint32_t last_wind_check = 0;

volatile uint32_t rain_time;
volatile uint32_t rain_last;
volatile uint32_t rain_interval;
volatile uint32_t rain;

volatile uint32_t rain_day_total;
volatile uint32_t rain_hour_total;

// Global variables to keep track of accumulations.

DateTime last_sensor_update;
int32_t sensor_update_period = 30;
uint8_t current_minute = 0;	
int16_t last_day = -1;
int16_t last_minute = -1;
int16_t last_hour = -1;
int32_t wind_direction_total = 0;
int32_t wind_direction_n = 0;

// Stored arrays 

char days_of_the_week[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

// Cardinal points for wind direction

char cardinal_points[16][3] = { "N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE", 
								"S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW"};

// ADC Values for wind direction

uint16_t wind_dir_res[16] =   { 66, 84, 93, 127, 184, 244, 287, 406, 461, 599, 630, 702, 785, 827, 886, 944};
int16_t  wind_dir_point[16] = { 5, 3, 4, 7, 6, 9, 8, 1, 2, 11, 10, 15, 0, 13, 14, 12 };

// Hardware definitions

Adafruit_BME280 bme;
Adafruit_SI1145 uv;
RTC_DS3231 rtc;

Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);
Adafruit_BLEGatt  gatt = Adafruit_BLEGatt(ble);

int32_t battery_id;
int32_t temperature_id;
int32_t humidity_id;
int32_t pressure_id;
int32_t vis_light_id;
int32_t ir_light_id;
int32_t uv_index_id;
int32_t wind_speed_id;
int32_t wind_direction_id;
int32_t rain_hour_id;
int32_t rain_day_id;
int32_t rain_hour_current_id;
int32_t rain_day_current_id;


struct readings {
	int16_t wind_direction;
	float wind_speed;
	uint32_t battery_voltage;
	float temperature;
	float pressure;
	float humidity;
	uint16_t vis_light;
	uint16_t ir_light;
	uint16_t uv_index;
	uint32_t rain_hour;
	uint32_t rain_day;
};

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
		rain_day_total++;
		rain_hour_total++;
		rain_last = rain_time;
	}
}


void setup() {
	// Start serial and wait for host to connect

	Serial.begin(115200);

	pinMode(ANEMOMETER_IO, INPUT_PULLUP);
	pinMode(RAIN_IO, INPUT_PULLUP);
	pinMode(I2C_ENABLE_IO, OUTPUT);
	pinMode(CLOCK_ENABLE_IO, OUTPUT);
	pinMode(BUILTIN_LED_IO, OUTPUT);

	for(int i;i<10;i++)
	{
		digitalWrite(BUILTIN_LED_IO, 1);
		delay(500);
		digitalWrite(BUILTIN_LED_IO, 0);
		delay(500);
	}

	digitalWrite(BUILTIN_LED_IO, 1);

	digitalWrite(I2C_ENABLE_IO, 0);
	digitalWrite(CLOCK_ENABLE_IO, 0);

	attachInterrupt(digitalPinToInterrupt(ANEMOMETER_IO), wind_speed_ISR, FALLING);	
	attachInterrupt(digitalPinToInterrupt(RAIN_IO), rainfall_ISR, FALLING);	

	rain_last = millis();
	wind_last = millis();
	rain_day_total = 0;
	rain_hour_total = 0;

	// Setup bluetooth

	Serial.println(F("Setup bluetooth ...."));

	setup_bluetooth();
	setup_bluetooth_le();

	// Setup RTC
	Serial.println(F("Setup clock ...."));
	setup_clock();
	last_sensor_update = read_clock();

	// Now setup sensors
	Serial.println(F("Setup sensors ...."));
	setup_i2c_sensors();

	digitalWrite(BUILTIN_LED_IO, 0);
}

void loop() {
	readings data;

	digitalWrite(BUILTIN_LED_IO, 1);

	DateTime now = read_clock();

	if(last_day != now.day())
	{
		// We have a new day!
		Serial.println(F("New day!"));

		uint32_t rain = compute_rain_day(true);
		setChar(rain_day_id, now.unixtime(), rain);
		// Zero out daily rainfall
		rain_day_total = 0;
	}

	if(last_minute < now.minute())
	{
		Serial.println(F("New minute!"));
	}

	if(last_hour < now.hour())
	{
		Serial.println(F("New hour!"));
		uint32_t rain = compute_rain_hour(true);
		setChar(rain_hour_id, now.unixtime(), rain);
	}
		
	last_minute = now.minute();
	last_day = now.day();
	last_hour = now.hour();

	TimeSpan time_diff = now - last_sensor_update;
	int32_t diff = time_diff.totalseconds();

	// average wind direction
	wind_direction_total += get_wind_direction();
	wind_direction_n++;

	if(diff > sensor_update_period)
	{
		wind_direction_total /= wind_direction_n;
		data.wind_direction = wind_direction_total;
		wind_direction_total = 0;
		wind_direction_n = 0;
		
		read_sensors(&data);
		write_sensors(&data, now);
		print_sensors(&data);

		// Now do wind direction
		last_sensor_update = now;
	}

	Serial.print(F("Tick-tock... "));
	Serial.println(sensor_update_period - diff);

	digitalWrite(BUILTIN_LED_IO, 0);
	delay(1000);
}

uint32_t compute_rain_hour(bool zero)
{
	// Compute the hourly rainfall
	uint32_t rain = rain_hour_total * 279;
	if(zero)
	{
		rain_hour_total = 0;
	}
	return rain;
}

uint32_t compute_rain_day(bool zero)
{
	// Compute the hourly rainfall
	uint32_t rain = rain_day_total * 279;
	if(zero)
	{
		rain_day_total = 0;
	}
	return rain;
}

bool write_sensors(readings *data, DateTime ts)
{
	uint32_t unixtime = ts.unixtime();
	setChar(battery_id, unixtime, (int32_t)data->battery_voltage);
	setChar(temperature_id, unixtime, (int32_t)(data->temperature * 1000));
	setChar(humidity_id, unixtime, (int32_t)(data->humidity * 1000));
	setChar(pressure_id, unixtime, (int32_t)(data->pressure * 10));
	setChar(uv_index_id, unixtime, (int32_t)(data->uv_index));
	setChar(vis_light_id, unixtime, (int32_t)data->vis_light);
	setChar(ir_light_id, unixtime, (int32_t)data->ir_light);
	setChar(wind_speed_id, unixtime, (int32_t)(data->wind_speed * 1000));
	setChar(wind_direction_id, unixtime, (int32_t)data->wind_direction);
	setChar(rain_hour_current_id, unixtime, (int32_t)data->rain_hour);
	setChar(rain_day_current_id, unixtime, (int32_t)data->rain_day);
}

bool read_sensors(readings *data)
{
	data->battery_voltage = read_battery();
	data->wind_speed = get_wind_speed();

	bme.takeForcedMeasurement();
	data->temperature = bme.readTemperature();
	data->humidity = bme.readHumidity();
	data->pressure = bme.readPressure();

	data->ir_light = uv.readIR();
	data->vis_light = uv.readVisible();
	data->uv_index = uv.readUV();
}

void print_sensors(readings *data)
{
	Serial.print(F("Battery voltage       = "));
	Serial.print(data->battery_voltage);
	Serial.println(F(" mV"));
	Serial.print(F("Wind speed            = "));
	Serial.print(data->wind_speed);
	Serial.println(F(" mph"));
	Serial.print(F("Wind direction        = "));
	if(data->wind_direction != -1)
	{
		Serial.print(cardinal_points[data->wind_direction]);
		Serial.print(" ");
		Serial.print(data->wind_direction);
	} else {
		Serial.print(F("ERROR"));
	}
	Serial.println(F("	"));
	Serial.print(F("Temperature           = "));
	Serial.print(data->temperature);
	Serial.println(F(" *C"));
	Serial.print(F("Humidity              = "));
	Serial.print(data->humidity);
	Serial.println(F(" %"));
	Serial.print(F("Pressure              = "));
	Serial.print(data->pressure);
	Serial.println(F(" "));
	Serial.print(F("UV Index              = "));
	Serial.print(data->uv_index);
	Serial.println(F(" "));
	Serial.print(F("Visible light         = "));
	Serial.print(data->vis_light);
	Serial.println(F(" "));
	Serial.print(F("IR Light              = "));
	Serial.print(data->ir_light);
	Serial.println(F(" "));
}

bool setup_clock(void)
{
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

	status = uv.begin();
	if(!status){
		Serial.println(F("Cound not find a valid SI1145 sensor, check wiring!"));
	}

	return true;
}	

float get_wind_speed(void)
{
	uint32_t delta_time = millis() - last_wind_check;
	float wind_speed = (float)wind_clicks * 1000 / (float)delta_time;

	wind_clicks = 0;
	last_wind_check = millis();

	wind_speed *= WINDSPEED_CALIBRATION;

	return wind_speed;
}	

int16_t get_wind_direction(void)
{
	unsigned int adc;

	adc = average_analog_read(WIND_DIRECTION_IO);
	// Add a value which is the "error" on the wind pin
	adc-=5;

	for(int i=0;i<16;i++)
	{
		if(adc < wind_dir_res[i])
		{
			return wind_dir_point[i];
		}
	}
		
	return -1;
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

DateTime read_clock(void)
{
	DateTime now = rtc.now();

	Serial.print(now.year(), DEC);
    Serial.print('/');
    Serial.print(now.month(), DEC);
    Serial.print('/');
    Serial.print(now.day(), DEC);
    Serial.print(" (");
    Serial.print(days_of_the_week[now.dayOfTheWeek()]);
    Serial.print(") ");
    Serial.print(now.hour(), DEC);
    Serial.print(':');
    Serial.print(now.minute(), DEC);
    Serial.print(':');
    Serial.print(now.second(), DEC);
    Serial.println();

	return now;
}

uint32_t read_battery(void)
{	
	uint32_t bat = average_analog_read(BATTERY_IO);
	bat *= (3300 * 2); // 3.3V ref, divide/2 
	bat /= 1023; // ADC full ragne
	return bat;
}


void setup_bluetooth(void)
{
    if (!ble.begin(VERBOSE_MODE))
    {
        error(F("Couldn't find Bluefruit\n"));
    }
  
    if (! ble.factoryReset() ){
        error(F("Couldn't factory reset\n"));
    }
  
    // Disable command echo from Bluefruit
    ble.echo(false);
    ble.verbose(false);
  
    // Set name of device
    if (!ble.sendCommandCheckOK(F("AT+GAPDEVNAME=Weather Station")))
    {
        error(F("Could not set device name."));
    }
  
    if (!ble.sendCommandCheckOK(F("AT+BLEPOWERLEVEL=4")))
    {
        error(F("Could not set output power"));
    }
  
    if (!ble.sendCommandCheckOK(F("AT+HWMODELED=SPI")) )
	{
        error(F("Could not set led output"));
    }
}

int32_t setup_bluetooth_char(uint16_t uuid)
{
	int32_t	id = gatt.addCharacteristic(uuid, GATT_CHARS_PROPERTIES_NOTIFY | GATT_CHARS_PROPERTIES_READ, 
										8, 8, BLE_DATATYPE_BYTEARRAY);
  	if (id == 0) {
  	    error(F("Could not add characteristic"));
  	}
	return id;
}
	
void setup_bluetooth_le(void)
{
                               
	int32_t environmentServiceId = gatt.addService(dataServiceUUID);
  	if(!environmentServiceId)
  	{
  	    error(F("Could not add data service"));
  	}

	battery_id = setup_bluetooth_char(battery_char_UUID);
	temperature_id = setup_bluetooth_char(temperature_char_UUID);
	humidity_id = setup_bluetooth_char(humidity_char_UUID);
	pressure_id = setup_bluetooth_char(pressure_char_UUID);
	vis_light_id = setup_bluetooth_char(vis_light_char_UUID);
	ir_light_id = setup_bluetooth_char(ir_light_char_UUID);
	uv_index_id = setup_bluetooth_char(uv_index_char_UUID);
	wind_speed_id = setup_bluetooth_char(wind_speed_char_UUID);
	wind_direction_id = setup_bluetooth_char(wind_direction_char_UUID);
	rain_hour_id = setup_bluetooth_char(rain_hour_char_UUID);
	rain_day_id = setup_bluetooth_char(rain_day_char_UUID);
	rain_day_current_id = setup_bluetooth_char(rain_day_current_char_UUID);
	rain_hour_current_id = setup_bluetooth_char(rain_hour_current_char_UUID);

		
	ble.reset();
}

void setChar(int32_t gattID, uint32_t timestamp, int32_t val)
{
    uint8_t _val[8];
    int i;

    _val[4] = (val >> 24) & 0xFF;
    _val[5] = (val >> 16) & 0xFF;
    _val[6] = (val >> 8) & 0xFF;
    _val[7] = val & 0xFF;

    _val[0] = (timestamp >> 24) & 0xFF;
    _val[1] = (timestamp >> 16) & 0xFF;
    _val[2] = (timestamp >> 8) & 0xFF;
    _val[3] = timestamp & 0xFF;

    gatt.setChar(gattID, _val, 8);
}
