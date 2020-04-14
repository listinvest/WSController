
#include <Arduino.h>

#include <ESPAsyncWebServer.h>
#include <esp_sleep.h>
#include <SPIFFS.h>
#include <WebSocketsServer.h>
#include <WiFi.h>
#include <ArduinoJson.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
extern "C" {
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
}
#include <AsyncMqttClient.h>
#include <driver/rtc_io.h>

#define D_SR04_EchoPin 14
#define D_SR04_TriggerPin 12
#define D_DHT11 13
#define D_BEWEGuNGSSENSOR 15
#define D_EVENTCOUNT 20
#define D_DeepSleepDauer 90       // sec
#define D_AwakeDauer 60           // sec
#define D_DHT11ReadIntervall 10   // sec
#define D_DistanceReadIntervall 3 // sec)
#define D_MovementReadIntervall 2 // sec)

typedef struct {
  uint8_t state;
  uint32_t ts;
} moveEvent_t;

#define MQTT_HOST IPAddress(192, 168, 18, 8)
#define MQTT_PORT 1883

AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;

int explode2int(String &s, char d);
uint getDistance();
void connectToWifi();
void connectToMqtt();

void WiFiEvent(WiFiEvent_t event);

void onMqttConnect(bool sessionPresent);

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason);

void onMqttSubscribe(uint16_t packetId, uint8_t qos);

void onMqttUnsubscribe(uint16_t packetId);

void onMqttMessage(char *topic, char *payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total);

void onMqttPublish(uint16_t packetId);

esp_sleep_wakeup_cause_t print_wakeup_reason();
void gotoDeepSleep();