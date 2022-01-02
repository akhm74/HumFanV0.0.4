/*
Main 2022.01.01
*/

#include <Arduino.h>
#include "DHT.h"
#include <ESP8266WiFi.h>
#include <Ticker.h>
#include <AsyncMqttClient.h>
#include <ArduinoJson.h>

// WIFI Settings s
#define WIFI_SSID "mejsebo1"
#define WIFI_PASSWORD "byvejen55"

WiFiEventHandler wifiConnectHandler;
WiFiEventHandler wifiDisconnectHandler;
Ticker wifiReconnectTimer;

// MQTT Broker Settings
#define MQTT_HOST "broker.hivemq.com"
#define MQTT_PORT 1883
// Topics
#define MQTT_PUB_TEMP "c3698581-d0a0-4c7a-8041-b50f2e80fe53_T"
#define MQTT_PUB_VAL "c3698581-d0a0-4c7a-8041-b50f2e80fe53_VAL"
#define MQTT_PUB_MQTT "c3698581-d0a0-4c7a-8041-b50f2e80fe53_MQTT"
#define MQTT_PUB_SET "c3698581-d0a0-4c7a-8041-b50f2e80fe53_SET"

// Define
AsyncMqttClient mqttClient;
Ticker mqttReconnectTimer;

// DHT Settings
// Digital pin connected to the DHT sensor
#define DHTPIN D3
#define DHTTYPE DHT22 // DHT 22  (AM2302), AM2321
// Initialize DHT sensor
DHT dht(DHTPIN, DHTTYPE);

// Fan pin
int Fan1 = D5;
// fan status
bool fanStatus = false;

// Set up LED Blinker
// Uses LED_BUILTIN
Ticker blinker;

// Variables to hold sensor readings
float temp;
float hum;
float DewPoint;
float deltaT;
float FanOn = 2;
float FanOff = 3;

unsigned long previousMillis = 0; // Stores last time temperature was published
const long interval = 60000;      // Interval at which to publish sensor readings

void changeState()
{
  digitalWrite(LED_BUILTIN, !(digitalRead(LED_BUILTIN))); // Invert Current State of LED
}

void FanControl()
{
  if (deltaT < FanOn)
  {
    digitalWrite(Fan1, HIGH);
  }
  if (deltaT > FanOff)
  {
    digitalWrite(Fan1, LOW);
  }
}

void dewP(double temp, double hum)
{
  double A0 = 373.15 / (273.15 + (double)temp);
  double SUM = -7.90298 * (A0 - 1);
  SUM += 5.02808 * log10(A0);
  SUM += -1.3816e-7 * (pow(10, (11.344 * (1 - 1 / A0))) - 1);
  SUM += 8.1328e-3 * (pow(10, (-3.49149 * (A0 - 1))) - 1);
  SUM += log10(1013.246);
  double VP = pow(10, SUM - 3) * (double)hum;
  double Td = log(VP / 0.61078); // temp var
  Td = (241.88 * Td) / (17.558 - Td);
  DewPoint = Td;
}

void connectToWifi()
{
  Serial.println("Connecting to Wi-Fi...");
  blinker.attach_ms(100, changeState);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

void connectToMqtt()
{
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();
}

void onWifiConnect(const WiFiEventStationModeGotIP &event)
{
  Serial.println("Connected to Wi-Fi.");
  blinker.attach_ms(500, changeState);
  connectToMqtt();
}

void onWifiDisconnect(const WiFiEventStationModeDisconnected &event)
{
  Serial.println("Disconnected from Wi-Fi.");
  blinker.attach_ms(100, changeState);
  mqttReconnectTimer.detach(); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
  wifiReconnectTimer.once(2, connectToWifi);
}

void onMqttConnect(bool sessionPresent)
{
  mqttClient.publish(MQTT_PUB_MQTT, 2, true, "Reconnected");
  Serial.println("Connected to MQTT.");
  Serial.print("Session present: ");
  Serial.println(sessionPresent);
  blinker.attach_ms(2000, changeState);
  uint16_t packetIdSub = mqttClient.subscribe(MQTT_PUB_SET, 2);
  Serial.print("Subscribing at QoS 2, packetId: ");
  Serial.println(packetIdSub);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason)
{
  Serial.println("Disconnected from MQTT.");
  blinker.attach_ms(500, changeState);
  if (WiFi.isConnected())
  {
    mqttReconnectTimer.once(2, connectToMqtt);
  }
}

void onMqttSubscribe(uint16_t packetId, uint8_t qos)
{
  Serial.println("Subscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
  Serial.print("  qos: ");
  Serial.println(qos);
}

void onMqttUnsubscribe(uint16_t packetId)
{
  Serial.println("Unsubscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

void onMqttPublish(uint16_t packetId)
{
  Serial.print("Publish acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

void onMqttMessage(char *topic, char *payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total)
{
  Serial.println("Publish received.");
  Serial.print("  topic: ");
  Serial.println(topic);
  Serial.print("  qos: ");
  Serial.println(properties.qos);
  Serial.print("  dup: ");
  Serial.println(properties.dup);
  Serial.print("  retain: ");
  Serial.println(properties.retain);
  Serial.print("  len: ");
  Serial.println(len);
  Serial.print("  index: ");
  Serial.println(index);
  Serial.print("  total: ");
  Serial.println(total);
}

void measureandsend()
{
  // New DHT sensor readings
  hum = dht.readHumidity();
  // Read temperature as Celsius (the default)
  temp = dht.readTemperature();
  // Read temperature as Fahrenheit (isFahrenheit = true)
  // temp = dht.readTemperature(true);
  dewP((double)temp, (double)hum);
  deltaT = temp - DewPoint;

  String OutputValues;
  StaticJsonDocument<200> doc;
  doc["temp"] = String(temp, 2);
  doc["hum"] = String(hum, 2);
  doc["dP"] = String(DewPoint, 2);
  doc["dT"] = String(deltaT, 2);
  doc["state"] = String(digitalRead(Fan1), 0);
  doc["F-on"] = String(FanOn, 2);
  doc["F-off"] = String(FanOff, 2);
  serializeJson(doc, OutputValues);
  Serial.printf("Message: %.2f degC \n", temp);

  uint16_t packetIdPub1 = mqttClient.publish(MQTT_PUB_TEMP, 2, true, String(temp).c_str());
  Serial.printf("Publishing on topic %s at QoS 2, packetId: %i ", MQTT_PUB_TEMP, packetIdPub1);

  uint16_t packetIdPub2 = mqttClient.publish(MQTT_PUB_VAL, 2, true, OutputValues.c_str());
  Serial.printf("Publishing on topic %s at QoS 2, packetId: %i ", MQTT_PUB_VAL, packetIdPub2);
  Serial.printf("Message: %s degC \n", OutputValues.c_str());
}

void setup()
{
  Serial.begin(115200);
  Serial.println();

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(Fan1, OUTPUT);
  digitalWrite(Fan1, LOW);

  //Initialize Ticker every 0.5s
  blinker.attach_ms(100, changeState); //Use attach_ms if you need time in ms

  dht.begin();

  pinMode(Fan1, OUTPUT);
  digitalWrite(Fan1, fanStatus);

  wifiConnectHandler = WiFi.onStationModeGotIP(onWifiConnect);
  wifiDisconnectHandler = WiFi.onStationModeDisconnected(onWifiDisconnect);

  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onSubscribe(onMqttSubscribe);
  mqttClient.onUnsubscribe(onMqttUnsubscribe);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.onMessage(onMqttMessage);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  // If your broker requires authentication (username and password), set them below
  //mqttClient.setCredentials("REPlACE_WITH_YOUR_USER", "REPLACE_WITH_YOUR_PASSWORD");

  connectToWifi();
}

void loop()
{
  unsigned long currentMillis = millis();
  // Every X number of seconds (interval = 10 seconds)
  // it publishes a new MQTT message
  if (currentMillis - previousMillis >= interval)
  {
    // Save the last time a new reading was published
    previousMillis = previousMillis + interval;
    measureandsend();
    FanControl();
  }
}