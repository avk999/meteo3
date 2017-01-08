# meteo3

Simple environment sensor.

ESP8266 controls OLED, BMP095 for temp and pressure, displays data and exports as mqtt.

SCL on D1 (D5) , SDA on D2 (D4) of nodemcu board.
Oled is the cheapest one from aliexpress, one with four pins (i2c, no reset).

DHT11 on GPIO14 (probably D5 on your board)
MH-Z19 on GPIO 13, 15

* TODO: add ambient light sensor
* TODO: add particles detector (Sharp).
* TODO: use WIFI  AP mode to configure MQTT


Thanks to the authors of esp8266 core and Adafruit for their great libraries!

To use create mqttconfig.h with MQTT connection data.

If esp8266 fails to connect to wifi it starts an AP. Follow instructions on the OLED to configure wifi params.

System resets on most errors, including failure to publish data to MQTT.


