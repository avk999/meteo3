#include <Arduino.h>

// for WiFiManager library
#include <ESP8266WiFi.h>          //https://github.com/esp8266/Arduino
#include <ESP8266WebServer.h>
#include <DNSServer.h>
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager

// Oled library
#include <U8g2lib.h>

//I2C
#include <Wire.h>


// BMP085
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>

// DHT11
#include <DHT.h>
// #include <DHT_U.h>
#define DHTPIN 14 
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE, 15); //11 is some magic value for ESP8266



// MQTT
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"

#define MYFONT u8g2_font_cu12_t_cyrillic
#define APNAME "Meteo"


#define SDA 4
#define SCL 5
U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, /* clock=*/ SCL, /* data=*/ SDA, /* reset=*/ U8X8_PIN_NONE);   // All Boards without Reset of the Display
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);


// MQTT setup
/* #define MQTT_HOST "broker.hivemq.com" */

#include "mqttconfig.h" 
/* should define MQTT_HOST,MQTT_LOGIN,MQTT_PASS,TOPIC_PREFIX */

/* 
#define DEBUG_ESP_WIFI
#define DEBUG_ESP_PORT Serial
#define MQTT_DEBUG 
*/
#define MQTT_PORT 1883
WiFiClient wifi;
// 
Adafruit_MQTT_Client mqtt(&wifi, MQTT_HOST, MQTT_PORT, MQTT_LOGIN, MQTT_PASS);
Adafruit_MQTT_Publish bmptemp = Adafruit_MQTT_Publish(&mqtt, TOPIC_PREFIX "temperature/bmp");
Adafruit_MQTT_Publish bmppressure = Adafruit_MQTT_Publish(&mqtt, TOPIC_PREFIX "pressure/bmp");
Adafruit_MQTT_Publish dhttemp = Adafruit_MQTT_Publish(&mqtt, TOPIC_PREFIX "temperature/dht");
Adafruit_MQTT_Publish dhthum = Adafruit_MQTT_Publish(&mqtt, TOPIC_PREFIX "humidity/dht");



void MQTT_connect() {
  int8_t ret;

  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }

  Serial.print("Connecting to MQTT... ");

  uint8_t retries = 3;
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
       Serial.println(mqtt.connectErrorString(ret));
       u8g2.clear();
       u8g2.setCursor(0,15);
       u8g2.print("MQTT err, ");
       u8g2.print(retries);
       u8g2.sendBuffer();
       Serial.println("Retrying MQTT connection in 10 seconds...");
       mqtt.disconnect();
       delay(5000);  // wait 5 seconds
       retries--;
       if (retries == 0) {
       
         // basically die and wait for WDT to reset me
         while (1);
       }
  }
  Serial.println("MQTT Connected!");
}



void configModeCallback (WiFiManager *myWiFiManager) {
  u8g2.clear();
  u8g2.setCursor(0,15);
  u8g2.print("AP running");
  u8g2.setCursor(0,31);
  u8g2.print(APNAME);
  u8g2.setCursor(0,46);
  u8g2.print("http://192.168.4.1/");
  u8g2.sendBuffer(); 
}




 
void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);

  u8g2.begin();
  u8g2.enableUTF8Print();
  u8g2.clear();
  u8g2.setFont(MYFONT);  // choose a suitable font
  u8g2.setCursor(0,15);
  u8g2.print("Стартуем/start");  // write something to the internal memory
  u8g2.sendBuffer();          // transfer internal memory to the display
   WiFiManager wifiManager;
   wifiManager.setAPCallback(configModeCallback);
   if (!wifiManager.autoConnect(APNAME)) {
      u8g2.clearBuffer();
      u8g2.drawStr(0,20,"Config failed");
      u8g2.sendBuffer();
      delay(3000);
      //reset and try again, or maybe put it to deep sleep
      ESP.reset();
      delay(5000);
    }
    else {
      // say "Connected" to OLED!
      u8g2.clear();
      u8g2.setCursor(0,15);
      u8g2.print( "Соединились!");
      u8g2.setCursor(0,32);
      u8g2.print("IP=");
      u8g2.print(WiFi.localIP());
      u8g2.sendBuffer();
      u8g2.setCursor(0,46);
//      delay(1000);
    }

    if (bmp.begin()){
      u8g2.print("BMP init ok!");
    } else {
      u8g2.print("BMP init failed!");
    }
    u8g2.sendBuffer();
    u8g2.setCursor(0,61);

    dht.begin();
    u8g2.print("DHT11 init done");
    u8g2.sendBuffer();
   // u8g2.setCursor(0,76);
   // u8g2.print("setup() done");
   // u8g2.sendBuffer();
 
    delay(1000);

   

}


void loop() {
 MQTT_connect();
 u8g2.clear();
 u8g2.setCursor(0,15);
 u8g2.setFont(u8g2_font_helvB14_tr );
 
 sensors_event_t event;
 bmp.getEvent(&event);
 if (!event.pressure){
  u8g2.print("BMP failed, reboot");
  u8g2.sendBuffer();
  delay(2000);
  ESP.restart();
 }

 u8g2.setFont(u8g2_font_helvB14_tr );

 float(temperature);
 bmp.getTemperature(&temperature);
// u8g2.print("t=");
 u8g2.print(temperature,1);
 //u8g2.print((char)223);
 u8g2.setFont(u8g2_font_unifont_t_symbols);
 u8g2.drawGlyph(60,15,0x2103); 
 //u8g2.setFont(MYFONT);
 u8g2.setFont(u8g2_font_helvB14_tr );

 //u8g2.print("C");

 float(mmhg);
 mmhg=event.pressure/1.3332239;
 u8g2.setCursor(0,35);
 //u8g2.print("p=");
 u8g2.print(mmhg);
 u8g2.print(" mmHg");
 u8g2.setCursor(0,46);
 //u8g2.print("p=");
 //u8g2.print(event.pressure);
 //u8g2.print(" hPa");
 //u8g2.setCursor(0,61);
 //u8g2.print("MQTT ");
 //if (bmptemp.publish(temperature)){
 // u8g2.print("temp OK ");
 //} else {
//  u8g2.print("temp fail");
// }

 bmptemp.publish(temperature);
 bmppressure.publish(event.pressure);
 


 u8g2.sendBuffer();

  // Read temperature as Celsius (the default)
  float t = dht.readTemperature();
  float h = dht.readHumidity();
  u8g2.setCursor(0,61);
  if  (isnan(h) || isnan(t)){
    u8g2.print("DHT error, reboot");
    delay(2000);
    ESP.restart();
  }
  dhttemp.publish(t);
  dhthum.publish(h);
  u8g2.print(h,1);
  u8g2.print(" %");
  u8g2.sendBuffer();
 
 delay(30000);
  
}
