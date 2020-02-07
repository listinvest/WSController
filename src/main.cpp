
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
const int led_pin = 2;//ttgo 
// Globals
AsyncWebServer server(80);
IPAddress IP;
WebSocketsServer webSocket = WebSocketsServer(1337);
char msg_buf[10];
int led_state = 0;
uint ts_lastRead = 0;
uint ts = 0;
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
  pinMode(led_pin, OUTPUT);
  digitalWrite(led_pin, LOW);

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
    WiFi.config(ip, gateway, subnet);
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
      Serial.print(WiFi.status());
      Serial.println(WiFi.localIP());
      IP = WiFi.localIP();
      Serial.println(IP);
      Serial.print("<");
      //network = true;
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
}

void loop() {
  ts = millis();
  // Look for and handle WebSocket data
  webSocket.loop();
  if (ts > ts_lastRead + 3000) {
    ts_lastRead = millis();
    led_state = digitalRead(led_pin);
    Serial.printf("led_state : %d\n", led_state);
    
    led_state = led_state ? 0 : 1;
    digitalWrite( led_pin, led_state);
    sprintf(msg_buf, "%d", led_state);
    Serial.printf("Sending loop [%u]: %s\n", 0, msg_buf);
    webSocket.sendTXT(0, msg_buf);
  }
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