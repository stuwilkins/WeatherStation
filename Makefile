ARDUINO_DIR   = /Applications/Arduino.app/Contents/Java
ARDUINO_PACKAGE_DIR = $(HOME)/Library/Arduino15/packages
ALTERNATE_CORE_PATH = $(HOME)/Library/Arduino15/packages/adafruit/hardware/samd/1.2.1
CMSIS_DIR = $(ARDUINO_PACKAGE_DIR)/arduino/tools/CMSIS/4.5.0/CMSIS
CMSIS_ATMEL_DIR = $(ARDUINO_PACKAGE_DIR)/arduino/tools/CMSIS-Atmel/1.2.0/CMSIS
ARCHITECTURE = sam
BOARD_TAG     = adafruit_feather_m0
CXXFLAGS_STD = -DARDUINO_ARCH_SAMD
ARDUINO_OTA = $(ARDUINO_PACKAGE_DIR)/arduino/tools/arduinoOTA/1.2.0/

ARDUINO_LIBS += Wire SPI Adafruit_INA219 MAX17043GU \
				uCRC16Lib RTClib-master WiFi101 WiFi101OTA \
				SD Adafruit_SleepyDog_Library Adafruit_BME280_Library \
				Adafruit_Unified_Sensor Adafruit_SI1145_Library \
				PubSubClient NTPClient Adafruit_ASFcore

include /usr/local/opt/arduino-mk/Sam.mk

#CC = arm-none-eabi-g++
CXX = arm-none-eabi-g++


upload-ota : all
	$(ARDUINO_OTA)/bin/arduinoOTA -address 192.168.1.19 -port 65280 \
	                              -username arduino -password bHmkHvDM3Ka*^nQz \
								  -sketch build-adafruit_feather_m0/WeatherStation.bin \
								  -upload /sketch -b -v
