#include <Arduino.h>
#include <WiFi.h>
extern "C" {
  #include "freertos/FreeRTOS.h"
  #include "freertos/timers.h"
}
#include <AsyncMqttClient.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include "config/config.h"
#include "functions/ota.h"
#include "functions/attention.h"
#include "functions/temp-aht10.h"
#include "tasks/garage-door.h"
#include "tasks/leak-detect.h"

#ifdef __arm__
// should use uinstd.h to define sbrk but Due causes a conflict
extern "C" char* sbrk(int incr);
#else  // __ARM__
extern char *__brkval;
#endif  // __arm__

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Variables to hold sensor readings
float temp;
float humidity;
// float pressure;
int doorStatus;
bool doorChanged;
int leakStatus;
bool leakChanged;

// Variables to hold sensor info
int8_t wifi_strength;
IPAddress ip;

unsigned short int displayCount = 2;

// Temperature MQTT Topics
#define MQTT_PUB_TEMP "home/" DEVICE_NAME "/metrics"
#define MQTT_SUB_CMD "home/" DEVICE_NAME "/cmd"

AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;
TimerHandle_t checkDoorTimer;
TimerHandle_t checkLeakTimer;

const long intervalMQTT   = 300000;    // Interval at which to publish sensor readings
unsigned long prevMillisMQTT = 0;   // Stores last time sensor was published
const long intervalSensor = 60000;    // Interval at which to read sensor and update display
unsigned long prevMillisSensor = 0;   // Stores last time sensor was published
unsigned long prevMillisDisplay = 0;   // Stores last time display was updated
const unsigned long intervalAlertMillis = 30000;  // How often to alert when leak sensor is wet
unsigned long prevMillisAlert = 0;
unsigned int countdownPerc;


const char* wl_status_to_string(int status) {
  switch (status) {
    case WL_NO_SHIELD: return "WL_NO_SHIELD";
    case WL_IDLE_STATUS: return "WL_IDLE_STATUS";
    case WL_NO_SSID_AVAIL: return "WL_NO_SSID_AVAIL";
    case WL_SCAN_COMPLETED: return "WL_SCAN_COMPLETED";
    case WL_CONNECTED: return "WL_CONNECTED";
    case WL_CONNECT_FAILED: return "WL_CONNECT_FAILED";
    case WL_CONNECTION_LOST: return "WL_CONNECTION_LOST";
    case WL_DISCONNECTED: return "WL_DISCONNECTED";
    default: return "UNKNOWN";
  }
}

void connectToWifi() {
  btStop(); // turn off bluetooth
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(WIFI_NETWORK, WIFI_PASSWORD);
}

void connectToMqtt() {
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();
}

void WiFiEvent(WiFiEvent_t event) {
  Serial.printf("[WiFi-event] event: %d\n", event);
  switch(event) {
    case SYSTEM_EVENT_STA_GOT_IP:
      Serial.println("WiFi connected");
      Serial.println("IP address: ");
      ip = WiFi.localIP();
      Serial.println(WiFi.localIP());
      connectToMqtt();
      break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
      Serial.println("WiFi lost connection");
      xTimerStop(mqttReconnectTimer, 0); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
      xTimerStart(wifiReconnectTimer, 0);
      break;
    default:
      break;
  }
}

void updateWiFiSignalStrength() {
  if(WiFi.isConnected()) {
    //serial_println(F("[WIFI] Updating signal strength..."));
    wifi_strength = WiFi.RSSI();
    // Serial.print("[WIFI] signal strength: ");
    // Serial.println(wifi_strength);
  }
}

void onMqttConnect(bool sessionPresent) {
  Serial.println("Connected to MQTT.");
  Serial.print("Session present: ");
  Serial.println(sessionPresent);
  uint16_t packetIdSub = mqttClient.subscribe(MQTT_SUB_CMD, 2);
  Serial.print("Subscribing at QoS 2, packetId: ");
  Serial.println(packetIdSub);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Disconnected from MQTT.");
  if (WiFi.isConnected()) {
    xTimerStart(mqttReconnectTimer, 0);
  }
}

void onMqttPublish(uint16_t packetId) {
  Serial.print("Publish acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

void drawMeasurementProgress(int perc) {
  const byte Y = SCREEN_HEIGHT - 2;
  // display.drawRect(x, y, width, height, color)
  display.drawRect(0, Y, int(SCREEN_WIDTH * (perc/100.0)), 1, WHITE);
}

void displayStats() {
  // clear display
  display.clearDisplay();
  
  // display temperature
  display.setTextSize(1);
  display.setCursor(0,0);
  display.print("Temperature: ");
  display.setTextSize(2);
  display.setCursor(0,10);
  display.print(temp);
  display.print(" ");
  display.setTextSize(1);
  display.cp437(true);
  display.write(167);
  display.setTextSize(2);
  display.print("F");
  
  // display humidity
  display.setTextSize(1);
  display.setCursor(0, 35);
  display.print("Humidity: ");
  display.setTextSize(2);
  display.setCursor(0, 45);
  display.print(humidity);
  display.print(" %");

  // display wifi signal strength
  display.setTextSize(1);
  display.setCursor(110, 0);
  display.print(String(wifi_strength));

  drawMeasurementProgress(countdownPerc);
  
  display.display();
  //delay(1000); // Pause for 1 second
  vTaskDelay(1000 / portTICK_PERIOD_MS);
}

void displayInfo() {

  String currentState = wl_status_to_string(WiFi.status());

  // clear display
  display.clearDisplay();
  
  // display temperature
  display.setTextSize(1);
  display.setCursor(0,0);
  display.print("IP: ");
  display.print(ip);
  display.setCursor(0,10);
  display.print("Device: ");
  display.print(DEVICE_NAME);
  display.setCursor(0,20);
  display.print("WiFi: ");
  display.print(currentState);
  display.setCursor(0,30);
  display.print("Signal: ");
  display.print(String(wifi_strength));
  display.setCursor(0,40);
  display.print("Door state: ");
  if (doorStatus == 0) {
   display.print("CLOSED"); 
  } else {
    display.print("OPEN"); 
  }
  display.setCursor(0,50);
  display.print("Leak state: ");
  if (leakStatus == 0) {
   display.print("DRY"); 
  } else {
    display.print("WET"); 
  }

  drawMeasurementProgress(countdownPerc);

  display.display();
  vTaskDelay(1000 / portTICK_PERIOD_MS);
}

void sendMqttUpdate() {
  char msg[58];  // variable for MQTT payload
  sprintf(msg, "{\"t\":\"%0.2f\",\"h\":\"%0.2f\",\"s\":\"%d\",\"d\":\"%d\",\"w\":\"%d\"}", temp, humidity, wifi_strength, doorStatus, leakStatus);
  // uint16_t packetIdPub1 = mqttClient.publish(MQTT_PUB_TEMP, 1, true, msg);
  uint16_t packetIdPub1 = mqttClient.publish(MQTT_PUB_TEMP, 0, false, msg);
  Serial.printf("[MQTT] Publishing on topic %s at QoS 1, packetId: %i\n", MQTT_PUB_TEMP, packetIdPub1);
  Serial.println(msg);
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

void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) {
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

  char s_payload[len + 1];
  String payload2 = "";
  if (len < 20) {
    Serial.print("  payload: ");
    s_payload[len] = '\0';
    strncpy(s_payload, payload, len);
    Serial.println(s_payload);
    payload2 = s_payload;
  } else {
    Serial.println(" payload len>20");
  }
  if (payload2 == "blink") {
    blink_now();
    prevMillisMQTT = 0;
  }
  if (payload2 == "toggle") {
    play_sound();
    toggleGarageRelay();
  }

}

void setup() {
  Serial.begin(115200);
  Serial.println();
  pinMode (LED_PIN, OUTPUT);

  Wire.begin(I2C_SDA, I2C_SCL);

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { 
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }
  display.clearDisplay();
  display.setTextColor(WHITE);

  // Set up temp sensor
  setupTemp();

  // Set up garage dooor monitor and leak monitor
  monitorDoorSetup();
  leakSetup();
  
  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));
  checkDoorTimer = xTimerCreate("doorTimer", pdMS_TO_TICKS(GARAGE_DOOR_MONITOR_MS), pdTRUE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(monitorDoorState));
  checkLeakTimer = xTimerCreate("leakTimer", pdMS_TO_TICKS(LEAK_MONITOR_MS), pdTRUE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(checkLeakStatus));

  WiFi.onEvent(WiFiEvent);

  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onSubscribe(onMqttSubscribe);
  mqttClient.onUnsubscribe(onMqttUnsubscribe);
  mqttClient.onMessage(onMqttMessage);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  mqttClient.setCredentials(MQTT_USER, MQTT_PASSWORD);
  connectToWifi();
  setupOTA();
  xTimerStart(checkDoorTimer, 0);
  xTimerStart(checkLeakTimer, 0);
}

void loop() {
  ArduinoOTA.handle();

  unsigned long currentMillis = millis();
  bool forceMqttUpdate = false;

  // read sensor every intervalSensor millis
  if (currentMillis - prevMillisSensor >= intervalSensor) {
    // Save the last time a new reading was published
    prevMillisSensor = currentMillis;
    measureTemp();
  }

  // update display every DISPLAY_INTERVAL and rotate display screens
  if (currentMillis - prevMillisDisplay >= DISPLAY_INTERVAL) {
    prevMillisDisplay = currentMillis;
    countdownPerc = 100-(((currentMillis-prevMillisMQTT)/double(intervalMQTT))*100);  // time until MQTT interval expires
    updateWiFiSignalStrength();
    if (displayCount == 2) {
      displayStats();
      displayCount = 1;
    } else {
      displayInfo();
      displayCount = 2;
    }
  }

  if ((doorChanged == true) || (leakChanged == true)) {
    forceMqttUpdate = true;
  }

  // If wet, do sound alert every intervalAlertMillis
  if (leakStatus == 1) {
    if (currentMillis - prevMillisAlert >= intervalAlertMillis) {
      prevMillisAlert = millis();
      blink_now();
    }
  }

  // publish a new MQTT message everyintervalMQTT millis
  if ((currentMillis - prevMillisMQTT >= intervalMQTT) || (forceMqttUpdate == true)) {
    // Save the last time a new reading was published
    prevMillisMQTT = currentMillis;
    doorChanged = false;
    leakChanged = false;
    blink_now();
    sendMqttUpdate();
  }
}
