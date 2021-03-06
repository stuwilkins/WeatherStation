/*
 * =====================================================================================
 *
 *       Filename:  WeatherStation.h
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


#define ANEMOMETER_IO 	    		       1
#define RAIN_IO       	    		       0
#define WIND_DIRECTION_IO		           A0
#define I2C_ENABLE_IO		    	         5
#define CLOCK_ENABLE_IO	    		       6
#define BUILTIN_LED_IO	    		       13
#define BATTERY_IO    	    		       A7
#define SOLAR_CHARGER_CHRG             5
#define SOLAR_CHARGER_DONE             6
#define BUFSIZE                        128
#define VERBOSE_MODE                   true
#define MAX17043_ADDRESS               0x36
#define AS3935_IRQ                     19
#define AS3935_CAPACITANCE             80 
#define AS3935_OUTDOORS                0
#define AS3935_DIST_EN                 1
#define AS3935_ADD                     0x03

// EEPROM Memory Locations
#define EEPROM_PRESSURE                0
#define EEPROM_RAIN                    740

// Calibration constants

// For windspeed 1 click.s^-1 = 1.492 MPH
// For windspeed 1 click.ms^-1 = 1492 MPH
// report windspeed increased by a factor of 1000
#define WINDSPEED_CALIBRATION 	1492000
#define RAIN_BUCKETS_TO_UM      279
#define SEALEVELPRESSURE_HPA    (1013.25)
#define UTC_TIME_OFFSET	        4

// Wind direction values for ADC
// { 66, 84, 93, 127, 184, 244, 287, 406, 461, 599, 630, 702, 785, 827, 886, 944};
// { 5, 3, 4, 7, 6, 9, 8, 1, 2, 11, 10, 15, 0, 13, 14, 12 };

#define WIND_ADC_5              66
#define WIND_ADC_3              84
#define WIND_ADC_4              93
#define WIND_ADC_7              127
#define WIND_ADC_6              184
#define WIND_ADC_9              244
#define WIND_ADC_8              287
#define WIND_ADC_1              406
#define WIND_ADC_2              461
#define WIND_ADC_11             599
#define WIND_ADC_10             630
#define WIND_ADC_15             702
#define WIND_ADC_0              785
#define WIND_ADC_13             827
#define WIND_ADC_14             886
#define WIND_ADC_12             944
#define WIND_ADC_DELTA          5

#define SENSOR_UPDATE_PERIOD    1

#define MQTT_CONNECTION_NAME    "weather_station"

#define WATCHDOG_TIME           8000
	
#define MQTT_SERVER             "192.168.1.2"
#define MQTT_PORT               1883
#define MQTT_CLIENT_NAME        "weather_station"

#define NTP_ADDRESS             "north-america.pool.ntp.org"
#define NTP_OFFSET              -4 * 60 * 60 // In seconds
#define NTP_INTERVAL            60 * 1000 // In miliseconds

// Stored arrays 

const char _day_0[12] PROGMEM = "Sunday";
const char _day_1[12] PROGMEM = "Monday";
const char _day_2[12] PROGMEM = "Tuesday";
const char _day_3[12] PROGMEM = "Wednesday";
const char _day_4[12] PROGMEM = "Thursday";
const char _day_5[12] PROGMEM = "Friday";
const char _day_6[12] PROGMEM = "Saturday";
const char * const days_of_the_week[] PROGMEM = {_day_0, _day_1, _day_2, _day_3, 
                                                 _day_4, _day_5, _day_6};

// Cardinal points for wind direction

const char _cardinal_point_0[4] PROGMEM = "N";
const char _cardinal_point_1[4] PROGMEM = "NNE";
const char _cardinal_point_2[4] PROGMEM = "NE";
const char _cardinal_point_3[4] PROGMEM = "ENE";
const char _cardinal_point_4[4] PROGMEM = "E";
const char _cardinal_point_5[4] PROGMEM = "ESE";
const char _cardinal_point_6[4] PROGMEM = "SE";
const char _cardinal_point_7[4] PROGMEM = "SSE";
const char _cardinal_point_8[4] PROGMEM = "S";
const char _cardinal_point_9[4] PROGMEM = "SSW";
const char _cardinal_point_10[4] PROGMEM = "SW";
const char _cardinal_point_11[4] PROGMEM = "WSW";
const char _cardinal_point_12[4] PROGMEM = "W";
const char _cardinal_point_13[4] PROGMEM = "WNW";
const char _cardinal_point_14[4] PROGMEM = "NW";
const char _cardinal_point_15[4] PROGMEM = "NNW";
const char * const cardinal_points[] PROGMEM = {_cardinal_point_0, _cardinal_point_1, 
                                                _cardinal_point_2, _cardinal_point_3,
                                                _cardinal_point_4, _cardinal_point_5,
                                                _cardinal_point_6, _cardinal_point_7,
                                                _cardinal_point_8, _cardinal_point_9,
                                                _cardinal_point_10, _cardinal_point_11,
                                                _cardinal_point_12, _cardinal_point_13,
                                                _cardinal_point_14, _cardinal_point_15};
  
struct readings {
  uint32_t status;
  int8_t wind_direction;
  int8_t wind_direction_2m[120];
  int8_t wind_direction_gust_10m;
  float wind_direction_2m_ave;
  float wind_direction_10m_ave[600];
  unsigned long wind_speed;
  unsigned long wind_speed_2m_t[120];
  unsigned long wind_speed_2m_c[120];
  unsigned long wind_speed_2m_ave;
  unsigned long wind_speed_10m_ave[600];
  unsigned long wind_speed_gust_10m;
  uint32_t battery_voltage;
  uint32_t battery_soc;
  int32_t solar_voltage;
  int32_t solar_current;
  float temperature;
  float pressure;
  uint8_t pressure_trend;
  float pressure_trend_val;
  float humidity;
  float dew_point;
  uint16_t vis_light;
  uint16_t ir_light;
  uint16_t uv_index;
  uint16_t rain;
  uint32_t rain_hour;
  uint32_t rain_day;
  uint32_t lightning_distance;
  uint32_t lightning_energy;
  uint32_t input_voltage;
};

enum PressureTrend {
  PRESSURE_RAPIDLY_RISING = 3,
  PRESSURE_RAPIDLY_FALLING = 4,
  PRESSURE_STEADY = 0,
  PRESSURE_FALLING = 2,
  PRESSURE_RISING = 1,
  PRESSURE_NO_DATA = 5
};

void lightning_ISR();
void wind_speed_ISR();
void rainfall_ISR();
void setup_lightning(void);
void compute_rain(readings *data, int curr_min);
bool write_lightning(readings *data, DateTime ts);
bool write_sensors(readings *data, DateTime ts, bool new_minute, bool new_hour, bool new_day);
bool read_sensors(readings *data);
bool calculate_sensors(readings *data);
bool read_wind(readings *data, int cur_2m, int n, int cur_10m, int m);
void print_sensors(readings *data);
bool setup_clock(void);
bool setup_i2c_sensors(void);
void get_wind_speed(unsigned long *clicks, unsigned long *delta);
int16_t get_wind_direction(void);
uint16_t average_analog_read(int pin);
void print_clock(DateTime *t);
uint32_t read_battery(void);
void setup_wifi(void);
void wifi_connect();
void mqtt_connect();
void mqtt_publish_data(const char *pub, uint32_t timestamp, int32_t val, int persist);
double calculate_dew_point(double RH, double T);
float mean_mitsuta(int8_t *bearing, int N);
void wifi_list_networks();
void mqtt_callback(char* topic, byte* payload, unsigned int length);
bool read_lightning(readings *data);
void set_clock(void);
void eeprom_reset(void);
bool calculate_pressure_trend(readings *data, int16_t this_3hr);


