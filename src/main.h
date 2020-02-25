
#include <Arduino.h>

#include <ESPAsyncWebServer.h>
#include <SPIFFS.h>
#include <WebSocketsServer.h>
#include <WiFi.h>
#include <ArduinoJson.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#define D_SR04_EchoPin 14
#define D_SR04_TriggerPin 12
#define D_DHT11     13
#define D_BEWEGuNGSSENSOR 15
#define D_EVENTCOUNT 20
int explode2int(String &s, char d);
uint getDistance();

typedef struct {
    uint8_t state;
    uint32_t ts;
} moveEvent_t;