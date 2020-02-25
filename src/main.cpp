
#include "main.h"

// Constants
const char *ssid = "zaphod24";
const char *password = "w8Ar7bjx723gagxa";
const char *msg_toggle_led = "toggleLED";
const char *msg_get_led = "getLEDState";
const int dns_port = 53;
const int http_port = 80;
const int ws_port = 1337;
// const int led_pin = 15;
const int led_pin = 2; // ttgo
// Globals
AsyncWebServer server(80);
IPAddress IP;
WebSocketsServer webSocket = WebSocketsServer(1337);
char msg_buf[2048];
int led_state = 0;
uint ts_lastMovementRead = 0;
uint ts_lastDistanceRead = 0;
uint ts_lastDHT11Read = 0;
uint ts = 0;

uint8_t ptrEvent = 0;
moveEvent_t mvE[D_EVENTCOUNT];
uint8_t moveState = 0;

DHT_Unified dht(D_DHT11, DHT11);

// Variables to save date and time
String formattedDate;

WiFiUDP ntpUDP;

// You can specify the time server pool and the offset (in seconds, can be
// changed later with setTimeOffset() ). Additionaly you can specify the
// update interval (in milliseconds, can be changed using setUpdateInterval() ).
NTPClient timeClient(ntpUDP, "europe.pool.ntp.org", 0);
/***********************************************************
 * Functions
 */

// Callback: receiving any WebSocket message
void onWebSocketEvent(uint8_t client_num, WStype_t type, uint8_t *payload,
                      size_t length) {

  // Figure out the type of WebSocket event
  switch (type) {

  // Client has disconnected
  case WStype_DISCONNECTED:
    Serial.printf("[%u] Disconnected!\n", client_num);
    break;
    // New client has connected
  case WStype_CONNECTED: {
    IPAddress ip = webSocket.remoteIP(client_num);
    Serial.printf("[%u] Connection from ", client_num);
    Serial.println(ip.toString());
  } break;

  // Handle text messages from client
  case WStype_TEXT:

    // Print out raw message
    Serial.printf("[%u] Received text: %s\n", client_num, payload);

    // Toggle LED
    if (strcmp((char *)payload, "toggleLED") == 0) {
      led_state = led_state ? 0 : 1;
      Serial.printf("Toggling LED to %u\n", led_state);
      digitalWrite(led_pin, led_state);

      // Report the state of the LED
    } else if (strcmp((char *)payload, "getLEDState") == 0) {
      sprintf(msg_buf, "%d", led_state);
      Serial.printf("Sending to [%u]: %s\n", client_num, msg_buf);
      webSocket.sendTXT(client_num, msg_buf);

      // Message not recognized
    } else {
      Serial.println("[%u] Message not recognized");
    }
    break;

  // For everything else: do nothing
  case WStype_BIN:
  case WStype_ERROR:
  case WStype_FRAGMENT_TEXT_START:
  case WStype_FRAGMENT_BIN_START:
  case WStype_FRAGMENT:
  case WStype_FRAGMENT_FIN:
  default:
    break;
  }
}

// Callback: send homepage
void onIndexRequest(AsyncWebServerRequest *request) {
  IPAddress remote_ip = request->client()->remoteIP();
  Serial.println("[" + remote_ip.toString() + "] HTTP GET request of " +
                 request->url());
  request->send(SPIFFS, "/index.html", "text/html");
}

// Callback: send style sheet
void onCSSRequest(AsyncWebServerRequest *request) {
  IPAddress remote_ip = request->client()->remoteIP();
  Serial.println("[" + remote_ip.toString() + "] HTTP GET request of " +
                 request->url());
  request->send(SPIFFS, "/style.css", "text/css");
}

// Callback: send 404 if requested file does not exist
void onPageNotFound(AsyncWebServerRequest *request) {
  IPAddress remote_ip = request->client()->remoteIP();
  Serial.println("[" + remote_ip.toString() + "] HTTP GET request of " +
                 request->url());
  request->send(404, "text/plain", "Not found");
}

/***********************************************************
 * Main
 */
void setup() {
  // Init LED and turn off

  // Start Serial port
  Serial.begin(115200);

  // Make sure we can read the file system
  if (!SPIFFS.begin()) {
    Serial.println("Error mounting SPIFFS");
    while (1)
      ;
  }
  /*
    // Start access point
    WiFi.softAP(ssid, password);
    // Print our IP address
    Serial.println();
    Serial.println("AP running");
    Serial.print("My IP address: ");
    Serial.println(WiFi.softAPIP());
    */
  Serial.println();
  Serial.println("connecting to");
  Serial.println(ssid);
  uint8_t addr[4];
  String ipaddr = "192.168.18.10";
  for (int i = 0; i < 4; i++) {
    addr[i] = explode2int(ipaddr, '.');
  }
  IPAddress ip(addr[0], addr[1], addr[2], addr[3]);

  ipaddr = "192.168.18.1";
  for (int i = 0; i < 4; i++) {
    addr[i] = explode2int(ipaddr, '.');
  }
  IPAddress gateway(addr[0], addr[1], addr[2], addr[3]);

  ipaddr = "255.255.255.0";
  for (int i = 0; i < 4; i++) {
    addr[i] = explode2int(ipaddr, '.');
  }
  IPAddress subnet(addr[0], addr[1], addr[2], addr[3]);

  ipaddr = "192.168.18.1";
  for (int i = 0; i < 4; i++) {
    addr[i] = explode2int(ipaddr, '.');
  }
  IPAddress dns(addr[0], addr[1], addr[2], addr[3]);

  WiFi.config(ip, gateway, subnet, dns);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  uint8_t ccount = 0;
  Serial.print(ssid);
  Serial.print("-");
  Serial.print(password);
  while (WiFi.status() != WL_CONNECTED && ccount < 20) {
    delay(1500);
    ccount++;
    Serial.print(".");
  }
  if (WiFi.status() != WL_CONNECTED) {
    Serial.print("WiFi NOT connected:");
    Serial.print(WiFi.status());
    Serial.print("<");
  } else {
    Serial.print("WiFi connected:");
    Serial.println(WiFi.status());
    Serial.println(WiFi.localIP());
    Serial.println(WiFi.dnsIP());
    Serial.println(WiFi.gatewayIP());

    IP = WiFi.localIP();
    Serial.println(IP);
    Serial.print("<");
    // network = true;
  }
  // On HTTP request for root, provide index.html file
  server.on("/", HTTP_GET, onIndexRequest);

  // On HTTP request for style sheet, provide style.css
  server.on("/style.css", HTTP_GET, onCSSRequest);

  // Handle requests for pages that do not exist
  server.onNotFound(onPageNotFound);

  // Start web server
  server.begin();

  // Start WebSocket server and assign callback
  webSocket.begin();
  webSocket.onEvent(onWebSocketEvent);

  pinMode(D_SR04_EchoPin, INPUT_PULLDOWN);
  pinMode(D_SR04_TriggerPin, OUTPUT);
  pinMode(D_BEWEGuNGSSENSOR, INPUT_PULLDOWN);
  digitalWrite(D_SR04_TriggerPin, LOW);
  /*  Serial.printf("trigPin1: %d\n", digitalRead(D_SR04_TriggerPin));
    delay(5000);
    digitalWrite(D_SR04_TriggerPin, HIGH);
    Serial.printf("trigPin2: %d\n", digitalRead(D_SR04_TriggerPin));
    delay(5000);
    digitalWrite(D_SR04_TriggerPin, LOW);
    Serial.printf("trigPin3: %d\n", digitalRead(D_SR04_TriggerPin));*/

  dht.begin();
  Serial.println(F("DHTxx Unified Sensor Example"));
  // Print temperature sensor details.
  sensor_t sensor;
  dht.temperature().getSensor(&sensor);
  Serial.println(F("------------------------------------"));
  Serial.println(F("Temperature Sensor"));
  Serial.print(F("Sensor Type: "));
  Serial.println(sensor.name);
  Serial.print(F("Driver Ver:  "));
  Serial.println(sensor.version);
  Serial.print(F("Unique ID:   "));
  Serial.println(sensor.sensor_id);
  Serial.print(F("Max Value:   "));
  Serial.print(sensor.max_value);
  Serial.println(F("°C"));
  Serial.print(F("Min Value:   "));
  Serial.print(sensor.min_value);
  Serial.println(F("°C"));
  Serial.print(F("Resolution:  "));
  Serial.print(sensor.resolution);
  Serial.println(F("°C"));
  Serial.println(F("------------------------------------"));
  // Print humidity sensor details.
  dht.humidity().getSensor(&sensor);
  Serial.println(F("Humidity Sensor"));
  Serial.print(F("Sensor Type: "));
  Serial.println(sensor.name);
  Serial.print(F("Driver Ver:  "));
  Serial.println(sensor.version);
  Serial.print(F("Unique ID:   "));
  Serial.println(sensor.sensor_id);
  Serial.print(F("Max Value:   "));
  Serial.print(sensor.max_value);
  Serial.println(F("%"));
  Serial.print(F("Min Value:   "));
  Serial.print(sensor.min_value);
  Serial.println(F("%"));
  Serial.print(F("Resolution:  "));
  Serial.print(sensor.resolution);
  Serial.println(F("%"));
  Serial.println(F("------------------------------------"));
  // NTP
  timeClient.begin();
  for (int i = 0; i < D_EVENTCOUNT; i++) {
    mvE[i].ts = 0;
    mvE[i].state = 0;
  }
}

void loop() {
  ts = millis();

  // Look for and handle WebSocket data
  webSocket.loop();
  if (ts > ts_lastMovementRead + 2000) {

    ts_lastMovementRead = millis();
    bool movement = digitalRead(D_BEWEGuNGSSENSOR);

    Serial.printf("Move?: %d\n", movement);
    // webSocket.sendTXT(0, msg_buf);

    if (moveState == movement) {
      ptrEvent++;
      if (ptrEvent >= D_EVENTCOUNT) {
        ptrEvent = 0;
      }
      timeClient.update();
      mvE[ptrEvent].state = movement;
      mvE[ptrEvent].ts = timeClient.getEpochTime();
      const int capacity = JSON_OBJECT_SIZE(4 * D_EVENTCOUNT + 2);
      StaticJsonDocument<capacity> jsondoc;
      jsondoc["type"] = "movement";
      timeClient.update();
      jsondoc["datetime"] = timeClient.getEpochTime();
      for (int i = 0; i < D_EVENTCOUNT; i++) {
        // String varname = "ts_" + String(i);
        // Serial.println(varname);
        jsondoc["ts_" + String(i)] = mvE[i].ts;
        jsondoc["state_" + String(i)] = mvE[i].state;
      }
      Serial.println(msg_buf);
      serializeJson(jsondoc, msg_buf);
      // Serial.println(msg_buf);
      webSocket.sendTXT(0, msg_buf);
      delay(5900);
    }
    moveState = movement;
  }
  if (ts > ts_lastDistanceRead + 200000) {

    uint distance = getDistance();
    // Serial.printf("distance: %d\n", distance);
    ts_lastDistanceRead = millis();
    const int capacity = JSON_OBJECT_SIZE(3);
    StaticJsonDocument<capacity> jsondoc;
    jsondoc["type"] = "distance";
    jsondoc["distance"] = distance;
    jsondoc["ts"] = ts_lastDistanceRead / 1000;
    serializeJson(jsondoc, msg_buf);
    // Serial.println(msg_buf);
    webSocket.sendTXT(0, msg_buf);
  }
  if (ts > ts_lastDHT11Read + 5000) {
    ts_lastDHT11Read = millis();
    sensors_event_t event;
    const int capacity = JSON_OBJECT_SIZE(4);
    StaticJsonDocument<capacity> jsondoc;
    jsondoc["type"] = "klima";
    timeClient.update();
    jsondoc["datetime"] = timeClient.getEpochTime();
    dht.temperature().getEvent(&event);
    if (isnan(event.temperature)) {
      Serial.println(F("Error reading temperature!"));
    } else {
      Serial.print(F("Temperature: "));
      Serial.print(event.temperature);
      Serial.println(F("°C"));
    }
    jsondoc["temperatur"] = event.temperature;
    // Get humidity event and print its value.
    dht.humidity().getEvent(&event);
    if (isnan(event.relative_humidity)) {
      Serial.println(F("Error reading humidity!"));
    } else {
      Serial.print(F("Humidity: "));
      Serial.print(event.relative_humidity);
      Serial.println(F("%"));
    }
    jsondoc["luftfeuchtigkeit"] = event.relative_humidity;

    serializeJson(jsondoc, msg_buf);
    // Serial.println(msg_buf);
    webSocket.sendTXT(0, msg_buf);
  }
}

uint getDistance() {
  uint entfernung = 0;
  uint zeit = 0;

  digitalWrite(D_SR04_TriggerPin, LOW);
  delayMicroseconds(3);
  noInterrupts();
  digitalWrite(D_SR04_TriggerPin, HIGH); // Trigger Impuls 10 us
  delayMicroseconds(10);
  digitalWrite(D_SR04_TriggerPin, LOW);
  zeit = pulseIn(D_SR04_EchoPin, HIGH); // Echo-Zeit messen
  interrupts();
  zeit = (zeit / 2);        // Zeit halbieren
  entfernung = zeit / 29.1; // Zeit in Zentimeter umrechnen
  return (entfernung);
}

int explode2int(String &s, char d) {
  int index;
  int num;
  if (s.length() < 1) {
    return 0;
  }
  index = s.indexOf(d);
  if (index >= 0) {
    num = s.substring(0, index).toInt();
    s = s.substring(index + 1);
    return num;
  } else {
    return s.substring(0, index).toInt();
  }
}