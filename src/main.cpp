
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

RTC_DATA_ATTR static time_t last; // remember last boot in RTC Memory
// Globals
AsyncWebServer server(80);
IPAddress IP;
WebSocketsServer webSocket = WebSocketsServer(1337);
char msg_buf[2048];
int led_state = 0;
uint ts_lastMovementRead = 0;
uint ts_lastDistanceRead = 0;
uint ts_lastDHT11Read = 0;
uint ts_DeepSleepTime = 60000;
uint ts = 0;

uint8_t ptrEvent = 0;
moveEvent_t mvE[D_EVENTCOUNT];
uint8_t moveState = 0;

DHT_Unified dht(D_DHT11, DHT11);

WiFiUDP ntpUDP;

// You can specify the time server pool and the offset (in seconds, can be
// changed later with setTimeOffset() ). Additionaly you can specify the
// update interval (in milliseconds, can be changed using setUpdateInterval() ).
NTPClient timeClient(ntpUDP, "europe.pool.ntp.org", 0);
/***********************************************************
 * Functions
 */

/*void connectToWifi() {
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}*/

void connectToMqtt() {
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();
  if (mqttClient.connected()) {
    Serial.println("Connected MQTT");
  } else {
    Serial.println("NOT Connected MQTT...");
  }
}

void WiFiEvent(WiFiEvent_t event) {
  Serial.printf("[WiFi-event] event: %d\n", event);
  switch (event) {
  case SYSTEM_EVENT_STA_GOT_IP:
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
    connectToMqtt();
    break;
  case SYSTEM_EVENT_STA_DISCONNECTED:
    Serial.println("WiFi lost connection");
    xTimerStop(mqttReconnectTimer, 0); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
    xTimerStart(wifiReconnectTimer, 0);
    break;
  }
}

void onMqttConnect(bool sessionPresent) {
  /*
uint16_t publish(const char* topic, uint8_t qos, bool retain, const char* payload = nullptr, size_t length = 0, bool dup = false, uint16_t message_id = 0)
  */
  Serial.println("Connected to MQTT.");
  Serial.print("Session present: ");
  Serial.println(sessionPresent);

  /*uint16_t packetIdSub = mqttClient.subscribe("test/lol", 2);
  Serial.print("Subscribing at QoS 2, packetId: ");
  Serial.println(packetIdSub);
 */

  // mqttClient.publish("keller", 0, true, "test 1");
  Serial.println("Publishing at QoS 0");
  // uint16_t packetIdPub1 =
  mqttClient.publish("keller/temperatur", 1, true, "10");

  /*Serial.print("Publishing at QoS 1, packetId: ");
  Serial.println(packetIdPub1);
  uint16_t packetIdPub2 = mqttClient.publish("test/lol", 2, true, "test 3");
  Serial.print("Publishing at QoS 2, packetId: ");
  Serial.println(packetIdPub2);*/
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Disconnected from MQTT.");
  // int8_t x = 0;
  // String sx = reason;
  // Serial.println(x);
  if (WiFi.isConnected()) {
    xTimerStart(mqttReconnectTimer, 0);
  }
}

void onMqttSubscribe(uint16_t packetId, uint8_t qos) {
  Serial.println("Subscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
  Serial.print("  qos: ");
  Serial.println(qos);
}

void onMqttUnsubscribe(uint16_t packetId) {
  Serial.println("Unsubscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

void onMqttMessage(char *topic, char *payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) {
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

void onMqttPublish(uint16_t packetId) {
  Serial.println("Publish acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

// Callback: receiving any WebSocket message
void onWebSocketEvent(uint8_t client_num, WStype_t type, uint8_t *payload, size_t length) {

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
  Serial.println("[" + remote_ip.toString() + "] HTTP GET request of " + request->url());
  request->send(SPIFFS, "/index.html", "text/html");
}

// Callback: send style sheet
void onCSSRequest(AsyncWebServerRequest *request) {
  IPAddress remote_ip = request->client()->remoteIP();
  Serial.println("[" + remote_ip.toString() + "] HTTP GET request of " + request->url());
  request->send(SPIFFS, "/style.css", "text/css");
}

// Callback: send 404 if requested file does not exist
void onPageNotFound(AsyncWebServerRequest *request) {
  IPAddress remote_ip = request->client()->remoteIP();
  Serial.println("[" + remote_ip.toString() + "] HTTP GET request of " + request->url());
  request->send(404, "text/plain", "Not found");
}

void connectToWifi() {
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
  }
}
/***********************************************************
 * Main
 */
void setup() {
  // Init LED and turn off

  // Start Serial port
  Serial.begin(115200);
  print_wakeup_reason();
  connectToWifi();

  // Make sure we can read the file system
  if (!SPIFFS.begin()) {
    Serial.println("Error mounting SPIFFS");
    while (1)
      ;
  }

  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void *)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));

  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void *)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));

  WiFi.onEvent(WiFiEvent);
  Serial.print("nach wifievent.......");

  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onSubscribe(onMqttSubscribe);
  mqttClient.onUnsubscribe(onMqttUnsubscribe);
  mqttClient.onMessage(onMqttMessage);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  mqttClient.setCredentials("domo", "domoomod");
  Serial.print("nach onmqttt.......");
  delay(1000);
  // network = true;

  // On HTTP request for root, provide index.html file
  server.on("/", HTTP_GET, onIndexRequest);
  Serial.print("nach onmqttt2.......");

  // On HTTP request for style sheet, provide style.css
  server.on("/style.css", HTTP_GET, onCSSRequest);
  Serial.print("nach onmqttt.3......");

  // Handle requests for pages that do not exist
  server.onNotFound(onPageNotFound);
  Serial.print("nach onmqttt.......");

  // Start web server
  server.begin();
  Serial.print("webserver begin.......");

  // Start WebSocket server and assign callback
  webSocket.begin();
  webSocket.onEvent(onWebSocketEvent);
  Serial.print("nach websocket......");
  delay(3000);
  rtc_gpio_deinit((gpio_num_t)D_BEWEGuNGSSENSOR);
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
  Serial.println(F("DHTxx Unified Sensor Example")); // Print temperature sensor details.
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
  delay(1000);
  connectToMqtt();
  Serial.print("setup end......");
}

void loop() {
  ts = millis();

  // Look for and handle WebSocket data
  webSocket.loop();
  if (ts > ts_DeepSleepTime) {
    gotoDeepSleep();
  }
  if (ts > ts_lastMovementRead + 2000) {

    ts_lastMovementRead = millis();
    bool movement = digitalRead(D_BEWEGuNGSSENSOR);

    // Serial.printf("Move?: %d\n", movement);
    // webSocket.sendTXT(0, msg_buf);

    if (moveState != movement) { // Serial.printf("Move?: %d\n", movement);// Serial.printf("Move?: %d\n", movement);
      ptrEvent++;
      if (ptrEvent >= D_EVENTCOUNT) {
        ptrEvent = 0;
      }

      mvE[ptrEvent].state = movement;
      mvE[ptrEvent].ts = timeClient.getEpochTime();
      // größere capacity wg. String duplucation
      const int capacity = JSON_OBJECT_SIZE(4 * D_EVENTCOUNT + 1);
      StaticJsonDocument<capacity> jsondoc;
      jsondoc["type"] = "movement";
      timeClient.update();
      jsondoc["datetime"] = timeClient.getEpochTime();
      for (int i = 0; i < D_EVENTCOUNT; i++) {

        jsondoc["ts_" + String(i)] = mvE[i].ts;
        jsondoc["state_" + String(i)] = mvE[i].state;
      }
      Serial.println(msg_buf);
      serializeJson(jsondoc, msg_buf);
      // Serial.println(msg_buf);
      webSocket.sendTXT(0, msg_buf);
      mqttClient.publish("keller/movement", 1, true, String(movement * 100).c_str());
    }
    moveState = movement;
  }
  if (ts > ts_lastDistanceRead + 10000) {

    uint distance = getDistance();
    ts_lastDistanceRead = millis();
    const int capacity = JSON_OBJECT_SIZE(3);
    StaticJsonDocument<capacity> jsondoc;
    jsondoc["type"] = "distance";
    timeClient.update();
    jsondoc["datetime"] = timeClient.getEpochTime();
    jsondoc["distance"] = distance;
    jsondoc["ts"] = ts_lastDistanceRead / 1000;
    serializeJson(jsondoc, msg_buf);
    mqttClient.publish("keller/distance", 1, true, String(distance).c_str());
    webSocket.sendTXT(0, msg_buf);
  }
  if (ts > ts_lastDHT11Read + 30000) {
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
      mqttClient.publish("keller/temperatur", 1, true, String(event.temperature).c_str());
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
      mqttClient.publish("keller/humidity", 1, true, String(event.relative_humidity).c_str());
    }
    jsondoc["luftfeuchtigkeit"] = event.relative_humidity;

    serializeJson(jsondoc, msg_buf);
    // Serial.println(msg_buf);
    webSocket.sendTXT(0, msg_buf);
  }
}

void gotoDeepSleep() {
  struct timeval now;
  Serial.println("goto DeepSleep");
  gettimeofday(&now, NULL);

  Serial.printf("deep sleep (%lds since last reset, %lds since last boot)\n", now.tv_sec, now.tv_sec - last);

  last = now.tv_sec;

  esp_sleep_enable_timer_wakeup(1000000 * D_DeepSleepDauer); // set timer but don't sleep now
                                                             // esp_bluedroid_disable, esp_bt_controller_disable, esp_wifi_stop
  printf("config IO\n");
  // Wake if low=0 high=1
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
  gpio_pulldown_en((gpio_num_t)D_BEWEGuNGSSENSOR);
  esp_sleep_enable_ext0_wakeup((gpio_num_t)D_BEWEGuNGSSENSOR, 1);

  // pinMode(D_BEWEGuNGSSENSOR, INPUT_PULLDOWN);

  printf("deep sleep\n");
  esp_deep_sleep_start();
  //!< is needed by one of the wakeup options.
  //!< Otherwise power it down.
  // gpio_set_direction(GPIO_INPUT_IO_TRIGGER, GPIO_MODE_INPUT);
  // pinMode(GPIO_INPUT_IO_TRIGGER_DIO0, INPUT);

  /*
  However, if RTC peripherals are powered down, internal pullup and pulldown
  resistors will be disabled. To use internal pullup or pulldown resistors,
  request RTC peripherals power domain to be kept on during deep sleep, and
  configure pullup/pulldown resistors using rtc_gpio_ functions, before
  entering deep sleep:

  esp_deep_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
  gpio_pullup_dis(gpio_num);
  gpio_pulldown_en(gpio_num);

  Warning

  After wake up from deep sleep, IO pad(s) used for wakeup will be
  configured as RTC IO. Before using these pads as digital GPIOs,
  reconfigure them using rtc_gpio_deinit(gpio_num) function.
  */
  /*
    // Wake if low=0 high=1
    // esp_sleep_enable_ext0_wakeup(GPIO_INPUT_IO_TRIGGER, 1);
    // Only RTC IO can be used as a source for external wake
    // source. They are pins: 0,2,4,12-15,25-27,32-39.
    esp_sleep_enable_ext1_wakeup(BUTTON_WAKEUP_BITMASK, ESP_EXT1_WAKEUP_ANY_HIGH);
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_AUTO);
    //!< Keep power domain enabled in deep sleep, if it
    printf("forrrrrr\n");
    for (uint8_t i = 0; i < 9; i++) {
      delay(1);
      Serial.print(".");
    }
    printf("deep sleep\n");
    esp_deep_sleep_start();
    */
}

esp_sleep_wakeup_cause_t print_wakeup_reason() {
  esp_sleep_wakeup_cause_t wakeup_reason;
  wakeup_reason = esp_sleep_get_wakeup_cause();
  switch (wakeup_reason) {
  case ESP_SLEEP_WAKEUP_EXT0:
    Serial.println("Wakeup caused by external signal using RTC_IO");
    break;
  case ESP_SLEEP_WAKEUP_EXT1:
    Serial.println("Wakeup caused by external signal using RTC_CNTL");
    break;
  case ESP_SLEEP_WAKEUP_TIMER:
    Serial.println("Wakeup caused by timer");
    break;
  case ESP_SLEEP_WAKEUP_TOUCHPAD:
    Serial.println("Wakeup caused by touchpad");
    break;
  case ESP_SLEEP_WAKEUP_ULP:
    Serial.println("Wakeup caused by ULP program");
    break;
  default:
    Serial.println("Wakeup was not caused by deep sleep: ");
    break;
  }
  return wakeup_reason;
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