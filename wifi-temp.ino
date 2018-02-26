/*
 * A simple Dallas DS18B20 temperature reading and posting via MQTT.
 * 
 * Several pins can be used (see OW_BUS_COUNT define) to read the sensors.
 * It might be useful, when wires are connected in a star to minimize
 * overall length of wires on one bus. D1, D2 and D4 pins are used here.
 * 
 *  Created on: Feb 23, 2018
 *      Author: Darau, blė
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 */

#include <FS.h>                   //this needs to be first, or it all crashes and burns...
#include <ESP8266WiFi.h>          //https://github.com/esp8266/Arduino
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager
#include <ArduinoOTA.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>          //https://github.com/bblanchon/ArduinoJson
#include <Ticker.h>
#include <OneWire.h>
#include <DallasTemperature.h>

const PROGMEM char* CFG_FILE           = "/cfg.json";
const PROGMEM char* ESP_ID             = "%06X";
const PROGMEM char* HOST_NAME          = "WIFITEMP-%s";
const PROGMEM char* MQTT_LWT_TOPIC     = "darauble/wifitemp/%s/hb";
const PROGMEM char* MQTT_SCRATCH_TOPIC = "darauble/ds18x20/%02X%02X%02X%02X%02X%02X%02X%02X/scratchpad";
const PROGMEM char* MQTT_TEMP_TOPIC    = "darauble/ds18x20/%02X%02X%02X%02X%02X%02X%02X%02X/temperature";
const PROGMEM char* F_SCRATCH          = "%02X%02X%02X%02X%02X%02X%02X%02X%02X";
const PROGMEM char* F_TEMP             = "%.5f";
const PROGMEM char* ONLINE             = "online";
const PROGMEM char* OFFLINE            = "offline";

//#define DEBUG 1

#ifdef DEBUG
const PROGMEM char* MQTT_DEBUG_TOPIC    = "darauble/ds18x20/%02X%02X%02X%02X%02X%02X%02X%02X/debug";
const PROGMEM char* debug_pat = "%d, %04X, %.5f";
char debug_buf[200];
#endif

#define D0              16        //WAKE  =>  16
#define D1              5         //IOS   =>  5
#define D2              4         //      =>  4
#define D3              0         //      =>  0
#define D4              2         //      =>  2
#define D5              14        //CLK   =>  14
#define D6              12        //MISO  =>  12 
#define D7              13        //MOSI  =>  13
#define D8              15        //CS    =>  15
#define D9              3         //RX    =>  3

// The count of OneWire busses to be used
#define OW_BUS_CNT     3
#define MAX_SENSOR_CNT 20   
#define BUTTON_PIN     D3

#define ESP_ID_LEN 7
#define TOPIC_LEN 70
#define VALUE_LEN 30
#define HOSTNAME_LEN 20
#define SERVER_LEN 40
#define PORT_LEN 6
#define USR_PWD_LEN 20

#define RESET_CNT 100
#define MQTT_RETRY_CNT 10

#define STATE_SEARCH 0
#define STATE_START 1
#define STATE_READ 2
#define STATE_WAIT 3

char hostname[HOSTNAME_LEN];
char esp_id[ESP_ID_LEN];

char mqtt_server[SERVER_LEN];
char mqtt_port[PORT_LEN] = "1883";
char mqtt_usr[USR_PWD_LEN];
char mqtt_pwd[USR_PWD_LEN];

char topic_buf[TOPIC_LEN];
char value_buf[VALUE_LEN];


//int light = 0;
boolean saveConfig = false;
boolean OTAupdate = false;

Ticker btn_timer;
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);
IPAddress mqtt_server_ip;

typedef struct {
  uint8_t bus;
  uint8_t addr[8];
  uint8_t scratchpad[9];
} tsensor;

OneWire bus[OW_BUS_CNT] = {
  OneWire(D1),
  OneWire(D2),
  OneWire(D4)
};
DallasTemperature dt[OW_BUS_CNT] = {
  DallasTemperature(&bus[0]),
  DallasTemperature(&bus[1]),
  DallasTemperature(&bus[2])
};
tsensor sensors[MAX_SENSOR_CNT];
uint8_t sensors_count = 0;



boolean resetFlag = false;
void check_reset()
{
  if (resetFlag) {
    WiFi.disconnect();
    delay(3000);
    ESP.reset();
    delay(3000);
  }
}

int sendStatus = 0;
unsigned long count = 0;

void button()
{
  if (!digitalRead(BUTTON_PIN)) {
    count++;
  } 
  else {
    if (count > 0 && count <= RESET_CNT) {
      Serial.println("Button down, no reset");
    } else if (count > RESET_CNT){
      Serial.println("Schedule reset"); 
      resetFlag = true;
    } 
    count=0;
  }
}

void saveConfigCb()
{
  Serial.println("Schedule config save");
  saveConfig = true;
}

void subCb(char* t, byte* p, unsigned int plen)
{
  /*if (strcmp(t, switch_topic) == 0) {
    if (strncmp((char*) p, SW_ON, plen) == 0) {
      digitalWrite(LED_PIN, LOW);
      digitalWrite(RELAY_PIN, HIGH);
    } else if (strncmp((char*) p, SW_OFF, plen) == 0) {
      digitalWrite(LED_PIN, HIGH);
      digitalWrite(RELAY_PIN, LOW);
    }
    sendStatus = true;
  }//*/
}

void initStrings()
{
  sprintf(esp_id, ESP_ID, ESP.getChipId());
  sprintf(hostname, HOST_NAME, esp_id);
}

void setup()
{
  Serial.begin(115200);
  Serial.println("Darau, ble - WiFi temperature reader");

  //pinMode(RELAY_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  //pinMode(LED_PIN, OUTPUT);
  //digitalWrite(LED_PIN, HIGH);
  //digitalWrite(RELAY_PIN, LOW);

  initStrings();
  
  btn_timer.attach(0.05, button);
  
  // Read config
  Serial.println("Mount SPIFFS...");
  if (SPIFFS.begin()) {
    if (SPIFFS.exists(CFG_FILE)) {
      Serial.println("Read config...");
      File configFile = SPIFFS.open(CFG_FILE, "r");
      if (configFile) {
        Serial.println("opened config file");
        size_t size = configFile.size();
        // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[size]);

        configFile.readBytes(buf.get(), size);
        DynamicJsonBuffer jsonBuffer;
        JsonObject& json = jsonBuffer.parseObject(buf.get());
        json.printTo(Serial);
        if (json.success()) {
          Serial.println("\nparsed json");

          strcpy(mqtt_server, json["mqtt_server"]);
          strcpy(mqtt_port, json["mqtt_port"]);
          strcpy(mqtt_usr, json["mqtt_usr"]);
          strcpy(mqtt_pwd, json["mqtt_pwd"]);
        } else {
          Serial.println("Failed to parse json");
        }
      }
    } else {
      Serial.println("No config yet.");
    }
  } else {
    Serial.println("SPIFFS failed :-(");
  }

  WiFi.hostname(hostname);

  // WifiManager init
  WiFiManagerParameter custom_mqtt_server("server", "mqtt server", mqtt_server, SERVER_LEN);
  WiFiManagerParameter custom_mqtt_port("port", "mqtt port", mqtt_port, PORT_LEN);
  WiFiManagerParameter custom_mqtt_usr("usr", "mqtt usr", mqtt_usr, USR_PWD_LEN);
  WiFiManagerParameter custom_mqtt_pwd("pwd", "mqtt pwd", mqtt_pwd, USR_PWD_LEN);

  WiFiManager wifiManager;

  wifiManager.setSaveConfigCallback(saveConfigCb);
  wifiManager.addParameter(&custom_mqtt_server);
  wifiManager.addParameter(&custom_mqtt_port);
  wifiManager.addParameter(&custom_mqtt_usr);
  wifiManager.addParameter(&custom_mqtt_pwd);

  if (!wifiManager.autoConnect(hostname)) {
    delay(3000);
    ESP.reset();
    delay(3000);
  }

  Serial.println("Connected to WiFi");
  
  strcpy(mqtt_server, custom_mqtt_server.getValue());
  strcpy(mqtt_port, custom_mqtt_port.getValue());
  strcpy(mqtt_usr, custom_mqtt_usr.getValue());
  strcpy(mqtt_pwd, custom_mqtt_pwd.getValue());

  if (saveConfig) {
    Serial.println("Save the config!");
    DynamicJsonBuffer jsonBuffer;
    JsonObject& json = jsonBuffer.createObject();
    json["mqtt_server"] = mqtt_server;
    json["mqtt_port"] = mqtt_port;
    json["mqtt_usr"] = mqtt_usr;
    json["mqtt_pwd"] = mqtt_pwd;

    File configFile = SPIFFS.open(CFG_FILE, "w");
    if (configFile) {
      json.printTo(Serial);
      json.printTo(configFile);
      configFile.close();
    } else {
      Serial.println("Failed to open config file for writing");
    }
    
    saveConfig = false;
  }

  WiFi.hostByName(mqtt_server, mqtt_server_ip);
  mqttClient.setServer(mqtt_server_ip, atoi(mqtt_port));
  mqttClient.setCallback(subCb);

  for (int i=0; i<MQTT_RETRY_CNT; i++) {
    sprintf_P(topic_buf, MQTT_LWT_TOPIC, esp_id);
    if (strlen(mqtt_usr) > 0) {
      if (!mqttClient.connect(hostname, mqtt_usr, mqtt_pwd, topic_buf, 0, 1, OFFLINE)) {
        delay(3000);
      }
    } else {
      if (!mqttClient.connect(hostname, topic_buf, 0, 1, OFFLINE)) {
        delay(3000);
      }
    }
  }

  if (mqttClient.connected()) {
    //mqttClient.subscribe(switch_topic);
    sprintf_P(topic_buf, MQTT_LWT_TOPIC, esp_id);
    mqttClient.publish(topic_buf, ONLINE, 1);
  } else {
    Serial.println("MQTT connection failed!");
  }

  ArduinoOTA.setHostname(hostname);
  ArduinoOTA.setPassword((const char *)WiFi.psk().c_str());
  ArduinoOTA.onStart([]() {
    OTAupdate = true;
    //digitalWrite(LED_PIN, HIGH);
    Serial.println("OTA start");
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("OTA End");
    ESP.restart();
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    //digitalWrite(LED_PIN, LOW);
    delay(5);
    //digitalWrite(LED_PIN, HIGH);
    Serial.printf("Progress: %u%%\r\n", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    //blinkLED(LED, 40, 2);
    OTAupdate = false;
    Serial.printf("OTA Error [%u] ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("OTA Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("OTA Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("OTA  Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("OTA  Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("OTA End Failed");
  });
  ArduinoOTA.begin();
}

void check_status()
{
  if (sendStatus) {
    /*if (digitalRead(RELAY_PIN) == LOW) {
      mqttClient.publish(status_topic, SW_OFF, true);
    } else {
      mqttClient.publish(status_topic, SW_ON, true);
    }//*/
    float temp_c;
    int16_t raw;
    
    for (int i=0; i<sensors_count; i++) {
      
      raw = ((((int16_t) sensors[i].scratchpad[1]) << 11)
        | (((int16_t) sensors[i].scratchpad[0]) << 3));

      temp_c = 0.0078125*(float) raw;
#ifdef DEBUG
      sprintf_P(topic_buf, MQTT_DEBUG_TOPIC,
        sensors[i].addr[0], sensors[i].addr[1], sensors[i].addr[2], sensors[i].addr[3],
        sensors[i].addr[4], sensors[i].addr[5], sensors[i].addr[6], sensors[i].addr[7]
      );
      sprintf_P(debug_buf, debug_pat, raw, raw, temp_c);
      mqttClient.publish(topic_buf, debug_buf, false);
#endif
      
      Serial.print(sensors[i].bus);
      Serial.print(" ");
      for (int j=0; j<8; j++) {
        Serial.printf("%02X ", sensors[i].addr[j]);
      }
      Serial.print("  ");
      for (int j=0; j<9; j++) {
        Serial.printf("%02X ", sensors[i].scratchpad[j]);
      }
      Serial.print("  ");
      Serial.println(temp_c);

      sprintf_P(topic_buf, MQTT_SCRATCH_TOPIC,
        sensors[i].addr[0], sensors[i].addr[1], sensors[i].addr[2], sensors[i].addr[3],
        sensors[i].addr[4], sensors[i].addr[5], sensors[i].addr[6], sensors[i].addr[7]
      );
      sprintf_P(value_buf, F_SCRATCH,
        sensors[i].scratchpad[0], sensors[i].scratchpad[1], sensors[i].scratchpad[2], sensors[i].scratchpad[3],
        sensors[i].scratchpad[4], sensors[i].scratchpad[5], sensors[i].scratchpad[6], sensors[i].scratchpad[7],
        sensors[i].scratchpad[8]
       );
      mqttClient.publish(topic_buf, value_buf, false);
      sprintf_P(topic_buf, MQTT_TEMP_TOPIC,
        sensors[i].addr[0], sensors[i].addr[1], sensors[i].addr[2], sensors[i].addr[3],
        sensors[i].addr[4], sensors[i].addr[5], sensors[i].addr[6], sensors[i].addr[7]
      );
      sprintf_P(value_buf, F_TEMP, temp_c);
      mqttClient.publish(topic_buf, value_buf, false);
    }
    sendStatus = false;
  }
}

boolean restartFlag = false;

void check_restart()
{
  if (restartFlag) {
    delay(3000);
    ESP.restart();
  }
}

int connect_ms = 0;
void check_connection()
{
  int cur_ms = millis();
  if (cur_ms > connect_ms+60000 || cur_ms < connect_ms) {
    connect_ms = cur_ms;
    if (WiFi.status() == WL_CONNECTED)  {
      if (mqttClient.connected()) {
        Serial.println("MQTT connection ok");
      } 
      else {
        Serial.println("MQTT connection failed");
        restartFlag = true;
      }
    }
    else { 
      Serial.println("WiFi connection failed");
      restartFlag = true;
    }
  }
}

int state = STATE_SEARCH;
int state_ms = 0;
int search_ms = 0;

void temperature_loop()
{
  int cur_ms = millis();

  switch(state) {
    case STATE_SEARCH:
      Serial.print("Start search... ");
      sensors_count = 0;
      for (int i; i<OW_BUS_CNT; i++) {
        bus[i].reset_search();
        while(bus[i].search(sensors[sensors_count].addr)) {
          if (dt[i].validAddress(sensors[sensors_count].addr)) {
            sensors[sensors_count].bus = i;
            sensors_count++;
            if (sensors_count > MAX_SENSOR_CNT) {
              break;
            }
          }
        }
        // Need to check twice, as double-break is required
        if (sensors_count > MAX_SENSOR_CNT) {
          break;
        }
      }
      Serial.print(sensors_count);
      Serial.println(" devices found.");
      search_ms = cur_ms;
      state_ms = cur_ms;
      state++;
    break;// end SEARCH
    
    case STATE_START:
      Serial.print("Request conversion... ");
      for (int i; i<OW_BUS_CNT; i++) {
        dt[i].requestTemperatures();
      }
      Serial.println("done.");
      state++;
      state_ms = cur_ms;
    break;// end START
    
    case STATE_READ:
      if (cur_ms - state_ms >= 1000) {
        Serial.print("Read sensors... ");
        // Give 1 second for sensors to make conversion (i.e. more than 750 ms for 12 bit resolution)
        for (int i=0; i<sensors_count; i++) {
          dt[sensors[i].bus].readScratchPad(sensors[i].addr, sensors[i].scratchpad);
        }
        Serial.println("done");
        state_ms = cur_ms;
        state++;
        sendStatus = true;
      }
    break;// end READ
    
    case STATE_WAIT:
      if (cur_ms - state_ms >= 30000) {
        state++;
      }
    break;// end WAIT
    
    default: // rollover
      if (cur_ms - search_ms <= 300000) {
        state = STATE_START;
      } else {
        // Perform re-reading of the busses every 5 minutes
        state = STATE_SEARCH;
      }
    break;// end rollover
  }
}

void loop()
{
  ArduinoOTA.handle();
  if (!OTAupdate) {
    mqttClient.loop();
    temperature_loop();
    check_status();
    check_connection();
    check_reset();
  }
}

