#include <Arduino.h>

// for WiFiManager library
#include <ESP8266WiFi.h>          //https://github.com/esp8266/Arduino
#include <ESP8266WebServer.h>
#include <DNSServer.h>
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager
#define APNAME "Meteo"

//I2C
#include <Wire.h>
#define SDA 4
#define SCL 5


// Oled library
#include <U8g2lib.h>
#define MYFONT u8g2_font_cu12_t_cyrillic
U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, /* clock=*/ SCL, /* data=*/ SDA, /* reset=*/ U8X8_PIN_NONE);   // All Boards without Reset of the Display


// BMP085
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);


// DHT11
#include <DHT.h>
// #include <DHT_U.h>
#define DHTPIN 14 
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE, 15); //third arg is clock cycles - need to increase  for ESP8266


// MH-Z19
#define MHRX 13
#define MHTX 15
#define MHTIMEOUT 3000
#include <SoftwareSerial.h>
SoftwareSerial MySerial(MHRX,MHTX);



// MQTT
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include "mqttconfig.h" 
/* should define MQTT_HOST,MQTT_LOGIN,MQTT_PASS,TOPIC_PREFIX */
#define MQTT_PORT 1883
WiFiClient wifi;
// 
Adafruit_MQTT_Client mqtt(&wifi, MQTT_HOST, MQTT_PORT, MQTT_LOGIN, MQTT_PASS);
Adafruit_MQTT_Publish bmptemp = Adafruit_MQTT_Publish(&mqtt, TOPIC_PREFIX "temperature/bmp");
Adafruit_MQTT_Publish bmppressure = Adafruit_MQTT_Publish(&mqtt, TOPIC_PREFIX "pressure/bmp");
Adafruit_MQTT_Publish dhttemp = Adafruit_MQTT_Publish(&mqtt, TOPIC_PREFIX "temperature/dht");
Adafruit_MQTT_Publish dhthum = Adafruit_MQTT_Publish(&mqtt, TOPIC_PREFIX "humidity/dht");
Adafruit_MQTT_Publish ppmco2 = Adafruit_MQTT_Publish(&mqtt, TOPIC_PREFIX "co2/mhz19");


void MQTT_connect() {
  int8_t ret;
  if (mqtt.connected()) {
    return;
  }
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
       delay(5000); 
       retries--;
       if (retries == 0) {
       
         //  die and wait for WDT to reset me
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


void mhrange(){
    byte setrange[9]={0xFF, 0x01, 0x99, 0x00, 0x00, 0x00, 0x07, 0xD0, 0x8F};
    unsigned char response[9];
  MySerial.write(setrange, 9);
  memset(response, 0, 9);
  int r=MySerial.readBytes(response, 9);
  Serial.print("sent rangeset, response:");
  for (int i=0;i<8;i++){
    Serial.print(response[i],HEX); Serial.print(" ");
  }
  Serial.println("done");

}

int readCO2(){
  byte cmd[9] = {0xFF,0x01,0x86,0x00,0x00,0x00,0x00,0x00,0x79}; 
  byte setrange[9]={0xFF, 0x01, 0x99, 0x00, 0x00, 0x00, 0x07, 0xD0, 0x8F};
  unsigned char response[9];
  int ppm;
  MySerial.write(cmd, 9);
  memset(response, 0, 9);
  int r=MySerial.readBytes(response, 9);
  int i;
  byte crc = 0;
  for (i = 1; i < 8; i++) crc+=response[i];
  crc = 255 - crc;
  crc++;
  if ( !(response[0] == 0xFF && response[1] == 0x86 && response[8] == crc) ) {
   
    Serial.println("CRC error: " + String(crc) + " / "+ String(response[8]));
    ppm=-1;
    } 
  else {
    unsigned int responseHigh = (unsigned int) response[2];
    unsigned int responseLow = (unsigned int) response[3];
    ppm = (256*responseHigh) + responseLow;
 //   Serial.print("ppm="); Serial.println(ppm);

 unsigned int t=response[4]-40;
 Serial.print("mh-z19 temp="); Serial.println(t,DEC);
}
return(ppm);
}


 
void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  MySerial.begin(9600);
  MySerial.setTimeout(MHTIMEOUT);
  
  u8g2.begin();
  u8g2.enableUTF8Print();
  u8g2.clear();
  u8g2.setFont(MYFONT);  
  u8g2.setCursor(0,15);
  u8g2.print("Init WIFI");  
  u8g2.sendBuffer();          
   WiFiManager wifiManager;
   wifiManager.setAPCallback(configModeCallback);
   if (!wifiManager.autoConnect(APNAME)) {
      u8g2.clearBuffer();
      u8g2.drawStr(0,20,"Config failed");
      u8g2.sendBuffer();
      delay(3000);
      ESP.reset();
      delay(5000);
    }
    else {
      // say "Connected" to OLED!
      u8g2.clear();
      u8g2.setCursor(0,15);
      u8g2.print( "Connected!");
      u8g2.setCursor(0,32);
      u8g2.print("IP=");
      u8g2.print(WiFi.localIP());
      u8g2.setCursor(0,46);
      u8g2.print(WiFi.SSID());
      u8g2.sendBuffer();
      u8g2.setCursor(0,61);
    
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
  //  u8g2.print("DHT11 init done");
    
    u8g2.sendBuffer();
     delay(1000);
   //  mhrange();

   

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
// u8g2.print(mmhg);  Pressure not that interesting and OLED is small
// u8g2.print(" mmHg");

 
 bmptemp.publish(temperature);
 bmppressure.publish(event.pressure);
 


 u8g2.sendBuffer();

// DHT
  u8g2.setCursor(0,35);
  // Read temperature as Celsius (the default)
  float t = dht.readTemperature();
  float h = dht.readHumidity();
  if  (isnan(h) || isnan(t)){
    u8g2.print("DHT error, reboot");
    u8g2.sendBuffer();
    delay(2000);
    ESP.restart();
  }
  Serial.print("dht temp=");Serial.println(t, 2);
  u8g2.print(h,0);
  u8g2.print(" %");
  dhttemp.publish(t);
  dhthum.publish(h);
  u8g2.sendBuffer();
  u8g2.setCursor(0,55);
// MH-Z19
  
  if (millis() < 180000){
    Serial.print("Still heating - ");
    Serial.println(millis());
    u8g2.print("CO2 heat ");
    u8g2.print( (180000 - millis())/1000 );
  } else { 
    int ppm=readCO2();
    if (ppm < 0){
      u8g2.print("CO2 err! reboot.");
      u8g2.sendBuffer();
      delay(2000);
      ESP.restart();
       }
     u8g2.print(ppm);
     u8g2.print(" ppm CO2");
     ppmco2.publish(ppm);
    } 
  

  
  //u8g2.print(h);
  //u8g2.print(" %");
  u8g2.sendBuffer();
 
 delay(30000);
  
}
