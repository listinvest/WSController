
#include <Arduino.h>

#include <ESPAsyncWebServer.h>
#include <SPIFFS.h>
#include <WebSocketsServer.h>
#include <WiFi.h>
#include <ArduinoJson.h>
int explode2int(String &s, char d);