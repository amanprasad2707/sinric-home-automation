// #define ENABLE_DEBUG
#ifdef ENABLE_DEBUG
  #define DEBUG_ESP_PORT Serial
  #define NODEBUG_WEBSOCKETS
  #define NDEBUG
#endif 

#define SERIAL_DEBUG

#include <Arduino.h>
#if defined(ESP8266)
  #include <ESP8266WiFi.h>
#elif defined(ESP32) || defined(ARDUINO_ARCH_RP2040)
  #include <WiFi.h>
#endif

#include "SinricPro.h"
#include "SinricProSwitch.h"
#include "SinricProTemperaturesensor.h"
#include "DHT.h"
#include <EEPROM.h>
#include <IRrecv.h>
#include <map>
#include <WiFiManager.h> 

#define WIFI_SSID1         ""
#define WIFI_PASS1         ""
#define WIFI_SSID2         ""
#define WIFI_PASS2         ""
#define APP_KEY           ""
#define APP_SECRET        ""
#define TEMP_SENSOR_ID    ""
#define DEVICE_ID_1       ""
#define DEVICE_ID_2       ""
#define EVENT_WAIT_TIME   10000               // send event every 10 seconds
#define EEPROM_SIZE 512   // Define the size of EEPROM to store relay states
#define RELAY_STATE_ADDR_START 0 // Starting address in EEPROM for relay states

#define TACTILE_BUTTON 1
#define DHT_TYPE DHT11
#define BAUD_RATE   115200
#define DEBOUNCE_TIME 250
#define RELAYPIN_1 16  // D0
#define RELAYPIN_2 5   // D1
#define SWITCHPIN_1 4  // D2
#define SWITCHPIN_2 0  // D3
#define DHT_PIN     2  // D4
#define WIFI_STATUS_CONNECTED_LED 14  // D5
#define WIFI_STATUS_DISCONNECTED_LED 12  // D6
#define IR_REC_PIN 13  // D7





IRrecv irrecv(IR_REC_PIN);
decode_results results;

DHT dht(DHT_PIN, DHT_TYPE);                   // DHT sensor
float temperature;                            // actual temperature
float humidity;                               // actual humidity
float lastTemperature;                        // last known temperature (for compare)
float lastHumidity;                           // last known humidity (for compare)
unsigned long lastEvent = (-EVENT_WAIT_TIME); // last time event has been sent

// for ir receiver
unsigned long lastReceived = 0;  
unsigned long debounceDelay = 200; 

typedef struct {  // struct for the std::map below
  int relayPIN;
  int flipSwitchPIN;
  bool activeLow;
} deviceConfig_t;

std::map<String, deviceConfig_t> devices = {
  //{deviceId, {relayPIN,  flipSwitchPIN, activeLow}}
    {DEVICE_ID_1, {  RELAYPIN_1, SWITCHPIN_1, true }},
    {DEVICE_ID_2, {  RELAYPIN_2, SWITCHPIN_2, true }},
};

typedef struct {    // struct for the std::map below
  String deviceId;
  bool lastFlipSwitchState;
  unsigned long lastFlipSwitchChange;
  bool activeLow;
} flipSwitchConfig_t;

std::map<int, flipSwitchConfig_t> flipSwitches;

// ===================== EEPROM HELPERS =====================
void saveRelayStateToEEPROM(int relayIndex, bool state) {
  int address = RELAY_STATE_ADDR_START + relayIndex;
  EEPROM.write(address, state);
  EEPROM.commit();
#ifdef SERIAL_DEBUG
  Serial.printf("Saved relay %d state: %s to EEPROM at address %d\r\n", relayIndex, state ? "ON" : "OFF", address);
#endif
}

bool readRelayStateFromEEPROM(int relayIndex) {
  int address = RELAY_STATE_ADDR_START + relayIndex;
  return EEPROM.read(address);
}

// ===================== RELAY & FLIP SWITCH SETUP =====================
void setupRelays() { 
  int relayIndex = 0;
  for (auto &device : devices) {
    int relayPIN = device.second.relayPIN;
    pinMode(relayPIN, OUTPUT);
    bool relayState = readRelayStateFromEEPROM(relayIndex);
    digitalWrite(relayPIN, relayState ? LOW : HIGH);
#ifdef SERIAL_DEBUG
    Serial.printf("Restored relay %d state: %s\r\n", relayIndex, relayState ? "ON" : "OFF");
#endif
    relayIndex++;
  }
}

void setupFlipSwitches() {
  for (auto &device : devices) {
    flipSwitchConfig_t flipSwitchConfig;
    flipSwitchConfig.deviceId = device.first;
    flipSwitchConfig.lastFlipSwitchChange = 0;
    flipSwitchConfig.lastFlipSwitchState = false;
    int flipSwitchPIN = device.second.flipSwitchPIN;
    bool activeLow = device.second.activeLow; 
    flipSwitchConfig.activeLow = activeLow;
    flipSwitches[flipSwitchPIN] = flipSwitchConfig;

    if(activeLow) pinMode(flipSwitchPIN, INPUT_PULLUP);
    else pinMode(flipSwitchPIN, INPUT);
  }
}

// ===================== TOGGLE RELAY FUNCTIONS =====================
void toggleRelay(int relayIndex) {
  String deviceId = (relayIndex == 1) ? DEVICE_ID_1 : DEVICE_ID_2;
  int relayPIN = devices[deviceId].relayPIN;
  bool currentRelayState = digitalRead(relayPIN) == LOW;
  bool newRelayState = !currentRelayState;
  digitalWrite(relayPIN, newRelayState ? LOW : HIGH);
  int index = std::distance(devices.begin(), devices.find(deviceId));
  saveRelayStateToEEPROM(index, newRelayState);
  SinricProSwitch &mySwitch = SinricPro[deviceId];
  mySwitch.sendPowerStateEvent(newRelayState);
}

void toggleAllRelays() {
  for (auto &device : devices) {
    int relayPIN = device.second.relayPIN;
    bool currentRelayState = digitalRead(relayPIN) == LOW;
    bool newRelayState = !currentRelayState;
    digitalWrite(relayPIN, newRelayState ? LOW : HIGH);
    int index = std::distance(devices.begin(), devices.find(device.first));
    saveRelayStateToEEPROM(index, newRelayState);
    SinricProSwitch &mySwitch = SinricPro[device.first];
    mySwitch.sendPowerStateEvent(newRelayState);
  }
}

// ===================== TEMPERATURE SENSOR HANDLER =====================
void handleTemperaturesensor() {
  unsigned long actualMillis = millis();
  if (actualMillis - lastEvent < EVENT_WAIT_TIME) return;

  if (!SinricPro.isConnected()) {
#ifdef SERIAL_DEBUG
    // Serial.printf("Not connected to Sinric Pro...!\r\n");
#endif
    return; 
  }

  temperature = dht.readTemperature();
  humidity = dht.readHumidity();
  if (isnan(temperature) || isnan(humidity)) return;
  if (temperature == lastTemperature && humidity == lastHumidity) return;

  SinricProTemperaturesensor &mySensor = SinricPro[TEMP_SENSOR_ID];
  bool success = mySensor.sendTemperatureEvent(temperature, humidity);
#ifdef SERIAL_DEBUG
  if (success)
    Serial.printf("Temperature: %2.1f Celsius\tHumidity: %2.1f%%\r\n", temperature, humidity);
  else
    Serial.printf("Something went wrong...could not send Event to server!\r\n");
#endif
  lastTemperature = temperature;
  lastHumidity = humidity;
  lastEvent = actualMillis;
}

// ===================== WIFI (NON-BLOCKING) STATE =====================
unsigned long wifiAttemptStart = 0;
bool wifiTriedFirst = false;
bool wifiTriedSecond = false;
unsigned long wifiRetryInterval = 15000UL;

void setupWiFi() {
#ifdef SERIAL_DEBUG
  Serial.printf("\r\n[Wifi]: Starting connect (non-blocking)\r\n");
#endif

  // pinMode(WIFI_STATUS_CONNECTED_LED, OUTPUT);
  digitalWrite(WIFI_STATUS_CONNECTED_LED, LOW);
  // pinMode(WIFI_STATUS_DISCONNECTED_LED, OUTPUT);
  digitalWrite(WIFI_STATUS_DISCONNECTED_LED, HIGH);

#if defined(ESP8266)
  WiFi.setSleepMode(WIFI_NONE_SLEEP); 
  WiFi.setAutoReconnect(true);
#elif defined(ESP32)
  WiFi.setSleep(false); 
  WiFi.setAutoReconnect(true);
#endif

  WiFi.begin(WIFI_SSID1, WIFI_PASS1);
  wifiAttemptStart = millis();
  wifiTriedFirst = true;
  wifiTriedSecond = false;
}

void handleWiFi() {
  unsigned long now = millis();

  if (WiFi.status() == WL_CONNECTED) {
    digitalWrite(WIFI_STATUS_CONNECTED_LED, HIGH);
    digitalWrite(WIFI_STATUS_DISCONNECTED_LED, LOW);
    wifiTriedFirst = wifiTriedSecond = false;
    return;
  }

  digitalWrite(WIFI_STATUS_CONNECTED_LED, LOW);
  digitalWrite(WIFI_STATUS_DISCONNECTED_LED, HIGH);

  if (wifiTriedFirst && !wifiTriedSecond && now - wifiAttemptStart >= 10000UL) {
#ifdef SERIAL_DEBUG
    Serial.printf("\n[Wifi]: Could not connect to %s, switching to %s...\r\n", WIFI_SSID1, WIFI_SSID2);
#endif
    WiFi.disconnect();
    WiFi.begin(WIFI_SSID2, WIFI_PASS2);
    wifiAttemptStart = now;
    wifiTriedSecond = true;
    wifiTriedFirst = false;
  } 
  else if (wifiTriedSecond && now - wifiAttemptStart >= 10000UL) {
#ifdef SERIAL_DEBUG
    Serial.printf("\n[Wifi]: Failed to connect to both networks. Retrying again soon...\r\n");
#endif
    WiFi.disconnect();
    delay(10);
    WiFi.begin(WIFI_SSID1, WIFI_PASS1);
    wifiAttemptStart = now;
    wifiTriedFirst = true;
    wifiTriedSecond = false;
  }
}



// ===================== SINRIC PRO SETUP =====================
bool onPowerState(String deviceId, bool &state) {
#ifdef SERIAL_DEBUG
  Serial.printf("%s: %s\r\n", deviceId.c_str(), state ? "on" : "off");
#endif
  int relayPIN = devices[deviceId].relayPIN;
  digitalWrite(relayPIN, state ? LOW : HIGH);
  int relayIndex = std::distance(devices.begin(), devices.find(deviceId));
  saveRelayStateToEEPROM(relayIndex, state);
  return true;
}

void setupSinricPro() {
  SinricProTemperaturesensor &mySensor = SinricPro[TEMP_SENSOR_ID];
  SinricPro.onConnected([]() {
#ifdef SERIAL_DEBUG
    Serial.printf("Connected to SinricPro\r\n");
#endif
    digitalWrite(WIFI_STATUS_CONNECTED_LED, HIGH);
    digitalWrite(WIFI_STATUS_DISCONNECTED_LED, LOW);
  });
  SinricPro.onDisconnected([]() {
#ifdef SERIAL_DEBUG
    Serial.printf("Disconnected from SinricPro\r\n");
#endif
    digitalWrite(WIFI_STATUS_CONNECTED_LED, LOW);
    digitalWrite(WIFI_STATUS_DISCONNECTED_LED, HIGH);
  });
  for (auto &device : devices) {
    const char *deviceId = device.first.c_str();
    SinricProSwitch &mySwitch = SinricPro[deviceId];
    mySwitch.onPowerState(onPowerState);
  }
  SinricPro.begin(APP_KEY, APP_SECRET);
}

// ===================== INTERRUPT FLAGS & ISRs (with debounce for touch sensors) =====================
volatile bool buttonPressedFlag1 = false;
volatile bool buttonPressedFlag2 = false;
volatile unsigned long lastInterruptTime1 = 0;
volatile unsigned long lastInterruptTime2 = 0;
const unsigned long interruptDebounceDelay = 400; // debounce for touch

void IRAM_ATTR isrButton1() {
  unsigned long currentTime = millis();
  if (currentTime - lastInterruptTime1 > interruptDebounceDelay) {
    buttonPressedFlag1 = true;
    lastInterruptTime1 = currentTime;
  }
}

void IRAM_ATTR isrButton2() {
  unsigned long currentTime = millis();
  if (currentTime - lastInterruptTime2 > interruptDebounceDelay) {
    buttonPressedFlag2 = true;
    lastInterruptTime2 = currentTime;
  }
}






// ===================== SETUP =====================
void setup() {
// #ifdef SERIAL_DEBUG
  Serial.begin(BAUD_RATE);
// #endif
  EEPROM.begin(EEPROM_SIZE);
  pinMode(WIFI_STATUS_CONNECTED_LED, OUTPUT);
  digitalWrite(WIFI_STATUS_CONNECTED_LED, LOW);
  pinMode(WIFI_STATUS_DISCONNECTED_LED, OUTPUT);
  digitalWrite(WIFI_STATUS_DISCONNECTED_LED, HIGH);

  setupRelays();
  setupFlipSwitches();
  dht.begin();

  pinMode(SWITCHPIN_1, INPUT_PULLUP);
  pinMode(SWITCHPIN_2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(SWITCHPIN_1), isrButton1, FALLING);
  attachInterrupt(digitalPinToInterrupt(SWITCHPIN_2), isrButton2, FALLING);

  setupWiFi();
  setupSinricPro();
  irrecv.enableIRIn();
}

unsigned long lastWiFiHandle = 0;
const unsigned long wifiHandleInterval = 180000UL; // 3 minutes
// ===================== LOOP =====================

void loop() {
  SinricPro.handle();

  // Run handleWiFi() every 3 minutes (non-blocking)
  unsigned long now = millis();
  if (now - lastWiFiHandle >= wifiHandleInterval) {
    handleWiFi();
    lastWiFiHandle = now;
  }

  if (buttonPressedFlag1) {
    buttonPressedFlag1 = false;
    toggleRelay(1);
#ifdef SERIAL_DEBUG
    Serial.println("ISR->handled: Relay 1 toggled (button)");
#endif
  }

  if (buttonPressedFlag2) {
    buttonPressedFlag2 = false;
    toggleRelay(2);
#ifdef SERIAL_DEBUG
    Serial.println("ISR->handled: Relay 2 toggled (button)");
#endif
  }

  handleTemperaturesensor();

  if (irrecv.decode(&results)) {
    unsigned long currentTime = millis();
    if (currentTime - lastReceived > debounceDelay) {
#ifdef SERIAL_DEBUG
      Serial.println(results.value, HEX);
#endif
      if (results.value == 0xFFB04F) toggleRelay(1);
      else if (results.value == 0xFFD827) toggleRelay(2);
      else if (results.value == 0xFF906F) toggleAllRelays();
      lastReceived = currentTime;
    }
    irrecv.resume();
  }
}
