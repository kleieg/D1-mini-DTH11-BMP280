
// Import required libraries
#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "LittleFS.h"
#include <Arduino_JSON.h>
#include <AsyncElegantOTA.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <MQTT.h>

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>


#include "Settings.h"
#include "WLAN_Credentials.h"
#include "config.h"
#include "log.h"

//<<<<<<<<<<<<<<<<
// current settings

struct PersistentState
{
  float Temp1Offset = 0.0f;
  float Temp2Offset = 0.0f;
  float HumOffset = 0.0f;
  float PressureOffset = 0.0f;
};

PersistentState g_state;

// will be computed as "<HOSTNAME>_<MAC-ADDRESS>"
String Hostname;

// define sensors & values

#define BMP_SCK  (13)
#define BMP_MISO (12)
#define BMP_MOSI (11)
#define BMP_CS   (10)

Adafruit_BMP280 bmp; // I2C

#define DHTTYPE    DHT11     // DHT 11

DHT dht(GPIO_DTH11_IN, DHTTYPE);

float Temp1 = 0, Temp2 = 0;
float Hum = 0;

float Pressure = 0;


int WiFi_reconnect = 0;


long lastSensorRead = 0;

// for WiFi
WiFiClient myClient;
long lastReconnectAttempt = 0;

// for MQTT
MQTTClient client(256);
int Mqtt_reconnect = 0;
bool Mqtt_refresh = false;

// NTP
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);
long My_time = 0;

// Timers auxiliar variables
long now = millis();
int LEDblink = 0;
bool led = 1;

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);

// Create a WebSocket object
AsyncWebSocket ws("/ws");

// end of variables -----------------------------------------------------

// Initialize LittleFS
void initLittleFS()
{
  if (!LittleFS.begin())
  {
    LOG_PRINTLN("An error has occurred while mounting LittleFS");
  }
  else
  {
    Settings::load(g_state);
  }
  LOG_PRINTLN("LittleFS mounted successfully");
}

// Initialize WiFi
void initWiFi()
{
  // dynamically determine hostname
  Hostname = HOSTNAME;
  Hostname += "_";
  Hostname += WiFi.macAddress();
  Hostname.replace(":", "");

  WiFi.mode(WIFI_STA);
  WiFi.hostname(Hostname);
  WiFi.begin(ssid, password);
  LOG_PRINT("Connecting to WiFi ..");
  while (WiFi.status() != WL_CONNECTED)
  {
    LOG_PRINT('.');
    delay(1000);
  }
  LOG_PRINTLN(WiFi.localIP());
}

String getOutputStates()
{
  JSONVar myArray;

  // system
  myArray["cards"][0]["c_text"] = String(Hostname) + "   /   " + String(VERSION);
  myArray["cards"][1]["c_text"] = String(WiFi.RSSI());
  myArray["cards"][2]["c_text"] = String(SENSOR_INTERVAL) + "ms";
  myArray["cards"][3]["c_text"] = String(My_time);
  myArray["cards"][4]["c_text"] = "WiFi = " + String(WiFi_reconnect) + "   MQTT = " + String(Mqtt_reconnect);
  myArray["cards"][5]["c_text"] = " to reboot click ok";

  // sensors
  myArray["cards"][6]["c_text"] = String(Temp1);
  myArray["cards"][7]["c_text"] = String(Temp2);
  myArray["cards"][8]["c_text"] = String(Hum);
  myArray["cards"][9]["c_text"] = String(Pressure);

  // configuration
  myArray["cards"][10]["c_text"] = String(g_state.Temp1Offset);
  myArray["cards"][11]["c_text"] = String(g_state.Temp2Offset);
  myArray["cards"][12]["c_text"] = String(g_state.HumOffset);
  myArray["cards"][13]["c_text"] = String(g_state.PressureOffset);

  String jsonString = JSON.stringify(myArray);
  return jsonString;
}

void notifyClients(String state)
{
  ws.textAll(state);
}

void handleWebSocketMessage(void *arg, uint8_t *data, size_t len)
{
  AwsFrameInfo *info = (AwsFrameInfo *)arg;
  if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT)
  {
    // according to AsyncWebServer documentation this is ok
    data[len] = 0;

    LOG_PRINTLN("Data received: ");
    LOG_PRINTF("%s\n", data);

    JSONVar json = JSON.parse((const char *)data);
    if (json == nullptr)
    {
      LOG_PRINTLN("Request is not valid json, ignoring");
      return;
    }
    if (!json.hasOwnProperty("action"))
    {
      LOG_PRINTLN("Request is not valid json, ignoring");
      return;
    }
    if (!strcmp(json["action"], "states"))
    {
      notifyClients(getOutputStates());
    }
    else if (!strcmp(json["action"], "reboot"))
    {
      LOG_PRINTLN("Reset..");
      ESP.restart();
    }
    else if (!strcmp(json["action"], "settings"))
    {
      if (!json.hasOwnProperty("data"))
      {
        LOG_PRINTLN("Settings request is missing data, ignoring");
        return;
      }
      bool updated = false;
      if (json["data"].hasOwnProperty("Temp1Offset"))
      {
        g_state.Temp1Offset = (double)json["data"]["Temp1Offset"];
        updated = true;
      }
      if (json["data"].hasOwnProperty("Temp2Offset"))
      {
        g_state.Temp2Offset = (double)json["data"]["Temp2Offset"];
        updated = true;
      }
      if (json["data"].hasOwnProperty("PressureOffset"))
      {
        g_state.PressureOffset = (double)json["data"]["PressureOffset"];
        updated = true;
      }
      if (json["data"].hasOwnProperty("HumOffset"))
      {
        g_state.HumOffset = (double)json["data"]["HumOffset"];
        updated = true;
      }
      if (updated)
      {
        Settings::save(g_state);
      }
    }
  }

  Mqtt_refresh = true;
}

void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type,
             void *arg, uint8_t *data, size_t len)
{
  switch (type)
  {
  case WS_EVT_CONNECT:
  {
    LOG_PRINTF("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
    break;
  }
  case WS_EVT_DISCONNECT:
    LOG_PRINTF("WebSocket client #%u disconnected\n", client->id());
    break;
  case WS_EVT_DATA:
    handleWebSocketMessage(arg, data, len);
    break;
  case WS_EVT_PONG:
  case WS_EVT_ERROR:
    break;
  }
}

void initWebSocket()
{
  ws.onEvent(onEvent);
#if defined CREDENTIALS_WEB_USER && defined CREDENTIALS_WEB_PASSWORD
  ws.setAuthentication(CREDENTIALS_WEB_USER, CREDENTIALS_WEB_PASSWORD);
#endif
  server.addHandler(&ws);
}

// reconnect to WiFi
void reconnect_wifi()
{
  LOG_PRINTF("%s\n", "WiFi try reconnect");
  
  WiFi_reconnect = WiFi_reconnect + 1;
  
  WiFi.reconnect();
  delay(500);
  
  if (WiFi.status() == WL_CONNECTED)
  {
    // Once connected, publish an announcement...
    LOG_PRINTF("%s\n", "WiFi reconnected");
  }
}

// This functions reconnects your ESP32 to your MQTT broker

void reconnect_mqtt()
{
  String willTopic = Hostname + "/LWT";
  String cmdTopic = Hostname + "/CMD/+";

  LOG_PRINTF("%s\n", "MQTT try reconnect");

  Mqtt_reconnect = Mqtt_reconnect + 1;
  
#if defined CREDENTIALS_MQTT_USER && defined CREDENTIALS_MQTT_PASSWORD
  if (client.connect(Hostname.c_str(), CREDENTIALS_MQTT_USER, CREDENTIALS_MQTT_PASSWORD))
#else
  if (client.connect(Hostname.c_str()))
#endif
  {
    LOG_PRINTF("%s\n", "connected");

    client.publish(willTopic.c_str(), "Online", true, 0);
  
    client.subscribe(cmdTopic.c_str());
  } else {
    LOG_PRINTF("Failed to connect to broker; error: %d\n", client.lastError());
  }
}

// receive MQTT messages
void MQTT_callback(String topic, String message)
{
  LOG_PRINTF("Message arrived on topic: %s; Data: %s\n", topic.c_str(), message.c_str());
}

// initialize MQTT
void initMQTT() {
  String willTopic = Hostname + "/LWT";
  
  LOG_PRINTF("setup MQTT\n");
  
  client.begin(myClient);
  client.setHost(CREDENTIALS_MQTT_BROKER, 1883);
  client.onMessage(MQTT_callback);
  client.setWill(willTopic.c_str(), "Offline", true, 0);
}

void MQTTsend()
{
  JSONVar mqtt_data, sensors, actuators;

  String mqtt_tag = Hostname + "/STATUS";
  LOG_PRINTF("%s\n", mqtt_tag.c_str());

  mqtt_data["Time"] = My_time;
  mqtt_data["RSSI"] = WiFi.RSSI();

  if(Temp1 > 0) {
    sensors["Temp1"] = Temp1;
  }
  if(Temp2 > 0) {
    sensors["Temp2"] = Temp2;
  }
  if(Hum > 0) {
    sensors["Hum"] = Hum;
  }
  if (Pressure > 0)
  {
    sensors["Pressure"] = Pressure;
  }

  mqtt_data["Sensors"] = sensors;

  String mqtt_string = JSON.stringify(mqtt_data);

  LOG_PRINTF("%s\n", mqtt_string.c_str());

  client.publish(mqtt_tag.c_str(), mqtt_string.c_str());

  notifyClients(getOutputStates());
}

void setup()
{
  // Serial port for debugging purposes
  LOG_INIT();

  delay(4000); // wait for serial log to be reday

  LOG_PRINTLN("start init\n");
  initLittleFS();
  initWiFi();
  initWebSocket();
  initMQTT();

  LOG_PRINTLN("init sensors\n");


  LOG_PRINTLN("BMP280 Forced Mode Test.");

  //if (!bmp.begin()) {
  if (!bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID)) {
    LOG_PRINTLN("Could not find a valid BMP280 sensor, check wiring or try a different address!");
    while (1) delay(10);
  }

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_FORCED,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  LOG_PRINTLN("DTH11 init.");
  dht.begin();
  
  LOG_PRINTLN("sensor initialized");

  // Route for root / web page
  LOG_PRINTF("set Webpage\n");
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(LittleFS, "/index.html", "text/html", false); });

  AsyncStaticWebHandler &handler = server.serveStatic("/", LittleFS, "/");
#if defined CREDENTIALS_WEB_USER && defined CREDENTIALS_WEB_PASSWORD
  handler.setAuthentication(CREDENTIALS_WEB_USER, CREDENTIALS_WEB_PASSWORD);
#endif

  // init NTP
  LOG_PRINTF("init NTP\n");
  timeClient.begin();
  timeClient.setTimeOffset(0);

  // Start ElegantOTA
  LOG_PRINTF("start Elegant OTA\n");
  AsyncElegantOTA.begin(&server);

  // Start server
  LOG_PRINTF("start server\n");
  server.begin();

  LOG_PRINTF("setup finished\n");
}

void loop()
{
  AsyncElegantOTA.loop();
  ws.cleanupClients();

  // update UPCtime
  timeClient.update();
  My_time = timeClient.getEpochTime();

  now = millis();

  /*
  // LED blinken
  if (now - LEDblink > LED_BLINK_INTERVAL)
  {
    LEDblink = now;
    if (led == 0)
    {
      digitalWrite(GPIO_LED_INTERN, 1);
      led = 1;
    }
    else
    {
      digitalWrite(GPIO_LED_INTERN, 0);
      led = 0;
    }
  }
*/


  if (now - lastSensorRead > SENSOR_INTERVAL)
  {
    lastSensorRead = now;
    Mqtt_refresh = true;

    // Read temperature as Celsius (the default)
    float newT = dht.readTemperature();
    // if temperature read failed, don't change t value
    if (isnan(newT)) {
      LOG_PRINTLN("Failed to read from DHT sensor!");
    }
    else {
      Temp1 = newT + g_state.Temp1Offset;
      LOG_PRINT("Temp1 *C = ");
      LOG_PRINT(Temp1);
      LOG_PRINT("\t\t");
    }
    // Read Humidity
    float newH = dht.readHumidity();
    // if humidity read failed, don't change h value 
    if (isnan(newH)) {
      LOG_PRINTLN("Failed to read from DHT sensor!");
    }
    else {
      Hum = newH + g_state.HumOffset;
      LOG_PRINT("Hum. % = ");
      LOG_PRINTLN(Hum);
    }

      // must call this to wake sensor up and get new measurement data
      // it blocks until measurement is complete
      if (bmp.takeForcedMeasurement()) {
        // can now print out the new measurements
        Temp2 = bmp.readTemperature() + g_state.Temp2Offset;
        Pressure = (bmp.readPressure()/100) + g_state.PressureOffset;

        LOG_PRINT(F("Temp2 *C = "));
        LOG_PRINT(Temp2);
        LOG_PRINT("\t\t");

        LOG_PRINT(F("Pressure hPa = "));
        LOG_PRINT(Pressure);
        LOG_PRINTLN("");
      } else {
        LOG_PRINTLN("BMP Forced measurement failed!");
      }
  }

  // check WiFi
  if (WiFi.status() != WL_CONNECTED)
  {
    // try reconnect every 5 seconds
    if (now - lastReconnectAttempt > RECONNECT_INTERVAL)
    {
      lastReconnectAttempt = now;
      // Attempt to reconnect
      LOG_PRINTF("WiFi reconnect");
      reconnect_wifi();
    }
  }
  else
  {
    // check if MQTT broker is still connected
    if (!client.connected())
    {
      // try reconnect every 5 seconds
      if (now - lastReconnectAttempt > RECONNECT_INTERVAL)
      {
        lastReconnectAttempt = now;
        // Attempt to reconnect
        LOG_PRINTF("MQTT reconnect");
        reconnect_mqtt();
      }
    }
    else
    {
      // Client connected
      client.loop();

      // send data to MQTT broker
      // do not send immediately after connecting, to make sure retained messages can arrive
      if (Mqtt_refresh && (now - lastReconnectAttempt > PUBLISH_DELAY))
      {
        Mqtt_refresh = false;
        MQTTsend();
      }
    }
  }

  // update Webpage and wait
  notifyClients(getOutputStates());
  delay(2000);
}