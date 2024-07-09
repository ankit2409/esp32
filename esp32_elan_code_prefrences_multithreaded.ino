#ifdef ESP8266
 #include <ESP8266WiFi.h>
 #else
 #include <WiFi.h>
#endif

//#include "DHTesp.h"
#include <ArduinoJson.h>
#include<SPI.h>
#include<Wire.h>
#include <PubSubClient.h>
#include <WiFiClientSecure.h>
#include <LiquidCrystal_I2C.h>
// #include <NTPClient.h>
// #include <WiFiUdp.h>
#include<Preferences.h>
#include<time.h>


#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels



/**** LED Settings *******/
// WiFiUDP ntpUDP;
// NTPClient timeClient(ntpUDP, "pool.ntp.org");
// const int led = 2;
const long utcOffsetInSeconds = 19800; // UTC offset for Indian Standard Time (IST) in seconds (5 hours + 30 minutes)

//Set LED pin as GPIO5
/****** WiFi Connection Details *******/
// const char* ssid = "";
// const char* password = "";
// const char* backupSsid = "ankit" ;
// const char* backupPassword = "anki1234";

String ssid = "";
String password = "";
const char* backupSsid = "ankit" ;
const char* backupPassword = "anki1234";

unsigned int count;
unsigned int mqqtLastCount = 0;
unsigned int nvsLastCount = 0;

// unsigned long epochTime;

int sensorPin = 5;

String company = "XYZ";
String machineSiteId = "HDR-01";


const char* machineTopic = "PMP_data_test_HDR-01";
const char* configWiFiTopicPrecede = "config/update/wifi/";
const char* configMqttTopicPrecede = "config/update/mqtt/";
const char* configCompanyTopicPrecede = "config/update/company/";
const char* configResetCounter = "config/update/counter/";
const char* publishMsgTopicPrecede = "message/";

String resultMqttTopic = "";
String resultWiFiTopic = "";
String resultCompanyTopic = "";
String resultPublishTopic = "";
String resultCounterResetTopic = "";

const char* wifiStatus = "WiFi NA";

String deviceId = "";



Preferences preferences;


LiquidCrystal_I2C lcd(0x27,16,2);  // set the LCD address to 0x27 for a 16 chars and 2 line display

/******* MQTT Broker Connection Details *******/
const char* mqtt_server = " d08900cfdef5463098201f44a1532917.s2.eu.hivemq.cloud";
const char* mqtt_username = "ankit";
const char* mqtt_password = "Ankit@123";
const int mqtt_port =8883;

WiFiClientSecure espClient;

/**** Time NTP CLient *****/
const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 19800;
const int   daylightOffset_sec = 0;
struct tm timeinfo;

const unsigned long interval = 60000;  // 1 minute in milliseconds
unsigned long lastMinuteMillis = 0; // Time of the last minute
int count1=0;
int feed=0;
unsigned long last_mqtt_send_time = 0;

/**** MQTT Client Initialisation Using WiFi Connection *****/
PubSubClient client(espClient);

int lastSensorValue = LOW;

#define MQTT_KEEP_ALIVE_INTERVAL 86400 // 24 hours
const unsigned long mqtt_send_interval = 5000;

DynamicJsonDocument doc(1024);



void setup_wifi() {
  delay(10);

  ssid = preferences.getString("SSID",backupSsid); 
  password = preferences.getString("PASSWORD",backupPassword);

  bool usingBackup = false; // Flag to check if backup password is being used

  if (ssid == backupSsid && password == backupPassword) {
    usingBackup = true;
  }

  

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid.c_str(), password.c_str());

  if (WiFi.status() != WL_CONNECTED) {
    delay(500);
    wifiStatus = "WiFi NA";
    Serial.println("WiFi NA.........");
  }
  
  randomSeed(micros());
  if(WiFi.status() == WL_CONNECTED){
    Serial.println("\nWiFi connected IP address: ");
    Serial.print("\nConnecting to ");
    Serial.println(ssid+"...   "+password);
    wifiStatus = "WiFi Conn..";
    Serial.println(WiFi.localIP());
    if (usingBackup) {
    Serial.println("Connected using backup SSID and password");
    } else {
      Serial.println("Connected using stored SSID and password");
    }
  }  

}
void reconnect() {
  // Loop until we're reconnected
  if (!client.connected()) {
    Serial.println("Attempting MQTT connection...");
    String clientId = "MQTT_FX_Client";   // Create a random client ID
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str(), mqtt_username, mqtt_password)) {
      Serial.println("connected to Mqtt");
      
      client.subscribe(configMqttTopicPrecede);
      client.subscribe(configWiFiTopicPrecede);
      client.subscribe(configResetCounter);
      // break;
  
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");   // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}
void callback(char* topic, byte* payload, unsigned int length) {
  String incommingMessage = "";
  for (int i = 0; i < length; i++) incommingMessage+=(char)payload[i];

  Serial.println("Message arrived ["+String(topic)+"]"+incommingMessage);

  if(String(topic) == resultWiFiTopic){
    DynamicJsonDocument wifidoc(1024);
    deserializeJson(wifidoc, incommingMessage);
    ssid = wifidoc["ssid"].as<String>();
    password = wifidoc["password"].as<String>();
    preferences.putString("SSID", ssid); 
    preferences.putString("PASSWORD", password);  
    setup_wifi();
  }

  else if(String(topic) == resultMqttTopic){
    DynamicJsonDocument mqttdoc(1024);
    deserializeJson(doc, incommingMessage);
    mqtt_server = mqttdoc["mqttServer"];
    mqtt_username = mqttdoc["mqttUsername"];
    mqtt_password = mqttdoc["mqttPassword"];
    reconnect();

  }

  else if(String(topic) == resultCounterResetTopic){
    count = 0;
    preferences.putUInt("counter", count);
  }
}
void publishMessage(const char* topic, String payload , boolean retained){
  if(!client.connected())
    reconnect();
  if (client.publish(topic, payload.c_str(), true))
      Serial.println("Message publised ["+String(topic)+"]: "+payload);
}
void setup() {

  Serial.begin(115200);
  preferences.begin("my-app", false);   // initialize perfernces
  while (!Serial) delay(1);
  setup_wifi();
  
  deviceId = WiFi.macAddress();

  resultMqttTopic  = String(configMqttTopicPrecede) + deviceId;
  resultWiFiTopic = String(configWiFiTopicPrecede) + deviceId;
  resultCompanyTopic = String(configCompanyTopicPrecede) + deviceId;
  resultPublishTopic = String(publishMsgTopicPrecede) + company + "/" + deviceId;
  resultCounterResetTopic = String(configResetCounter) +  deviceId;


  configMqttTopicPrecede = resultMqttTopic.c_str();
  configWiFiTopicPrecede = resultWiFiTopic.c_str();
  configCompanyTopicPrecede = resultCompanyTopic.c_str();
  publishMsgTopicPrecede = resultPublishTopic.c_str();
  configResetCounter = resultCounterResetTopic.c_str();



  Serial.println(configMqttTopicPrecede);
  Serial.println(configWiFiTopicPrecede);
  Serial.println(configCompanyTopicPrecede);
  Serial.println(publishMsgTopicPrecede);

  
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer); //configure time 
  

  espClient.setInsecure();
 
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
  client.setKeepAlive(MQTT_KEEP_ALIVE_INTERVAL);
  reconnect();
  
  count = 0 ;

  lcd.init();                      // initialize the lcd 
  // Print a message to the LCD.
  lcd.backlight();
  lcd.setCursor(0,0);
  // lcd.print(machineSiteId);

  xTaskCreatePinnedToCore(
      publishMqttMessage, // Task function
      "publishMqttMessage", // Task name
      10000, // Stack size
      NULL, // Task parameters (pointer to the dynamically allocated memory)
      1, // Priority (higher priority than loop task)
      NULL, // Task handle
      0 // Core to run the task on (0 or 1)
    );
}
void loop() {
  // lcd.clear();
  // lcd.setCursor(0,0);
  // lcd.print(machineSiteId);
  // lcd.setCursor(7,0);
  // lcd.print(&timeinfo, "%H:%M:%S");
  // lcd.setCursor(7,1);
  // lcd.print(wifiStatus);
  // Serial.println(wifiStatus);
  // Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
    // Serial.println(" count from another thread is "+ String(count)+" mqqtLastCount is "+String(mqqtLastCount));

    if (WiFi.status() != WL_CONNECTED) {
      // lcd.clear();
      wifiStatus = "WiFi NA";
      // Serial.println("Not connected to Wi-Fi. Reconnecting...");
      setup_wifi(); // Reconnect to Wi-Fi
      configTime(gmtOffset_sec, daylightOffset_sec, ntpServer); //configure time 
      reconnect(); // Reconnect to MQTT broker
    }
    if(!getLocalTime(&timeinfo)){
    // Serial.println("Failed to obtain time");
    return;
    }
    client.loop();
  
  if(mqqtLastCount < count && millis() - last_mqtt_send_time >= mqtt_send_interval) {
      preferences.putUInt("counter", count);
      doc.clear();
      doc["deviceId"] = deviceId;
      doc["siteId"] = machineSiteId;
      doc["count"] = count;
      doc["publishedTime"] = mktime(&timeinfo);
      doc["nodeMcuCode"] = String(deviceId)+"_"+machineSiteId;
      
      char mqtt_message[400];
      serializeJson(doc, mqtt_message);
      mqqtLastCount = count;
      last_mqtt_send_time = millis();
      delay(5000);
      publishMessage(publishMsgTopicPrecede, mqtt_message, true);
  }
  
}
void publishMqttMessage(void * parameter) {
  
  while(true){

    int sensorValue = digitalRead(sensorPin);
  
  if(sensorValue==HIGH  && lastSensorValue == LOW)
  {
      if(count==0){
        count = preferences.getUInt("counter", 0);
      }

    count++;    
    Serial.println(count);

    count1++;
      
    unsigned long milli = millis();

    // Check if a minute has passed

    if (milli - lastMinuteMillis >= interval) {
      // Calculate events per minute
      float eventsPerMinute = count1 / ((milli - lastMinuteMillis) / 60000.0);
      lcd.setCursor(7,0);
      lcd.print("F");
      lcd.setCursor(9,0);
      lcd.print(eventsPerMinute);
      feed = count1;
      Serial.println("feed is ---------------------->"+ String(feed));
      
    // //   // Reset event count and update last minute time
      count1 = 0;
      lastMinuteMillis = milli;
    }
    
  }
  lastSensorValue = sensorValue;
  lcd.setCursor(0,1);
  lcd.print(count);
  }
}





