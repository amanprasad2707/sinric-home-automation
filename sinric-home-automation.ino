#define ENABLE_DEBUG
#ifdef ENABLE_DEBUG
  #define DEBUG_ESP_PORT Serial
  #define NODEBUG_WEBSOCKETS
  #define NDEBUG
#endif 

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

#define WIFI_SSID1         "VIRUS"
#define WIFI_PASS1         "VIRUS123"
#define WIFI_SSID2         "VIRUS"
#define WIFI_PASS2         "VIRUS123"
#define APP_KEY           "09c154f8-03be-4c2d-a162-52e518479173"
#define APP_SECRET        "1eb13d4f-9990-4d97-aaf9-b482e870824f-464779c7-0892-4b52-b94f-f063b9d2f118"
#define TEMP_SENSOR_ID    "67fbd0c7947cbabd20fb0cd9"
#define DEVICE_ID_1    "67fbd015947cbabd20fb0b4e"
#define DEVICE_ID_2    "67fbd0978ed485694c1664ce"
#define EVENT_WAIT_TIME   10000               // send event every 10 seconds
#define EEPROM_SIZE 512   // Define the size of EEPROM to store relay states
#define RELAY_STATE_ADDR_START 0 // Starting address in EEPROM for relay states



// comment the following line if you use a toggle switches instead of tactile buttons
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
unsigned long lastReceived = 0;  // Time of last received signal
unsigned long debounceDelay = 200; // Delay in milliseconds (0.2 seconds)


typedef struct {      // struct for the std::map below
  int relayPIN;
  int flipSwitchPIN;
  bool activeLow;
} deviceConfig_t;

// this is the main configuration
// please put in your deviceId, the PIN for Relay and PIN for flipSwitch
// this can be up to N devices...depending on how much pin's available on your device ;)
// right now we have 4 devicesIds going to 4 relays and 4 flip switches to switch the relay manually
std::map<String, deviceConfig_t> devices = {
    //{deviceId, {relayPIN,  flipSwitchPIN, activeLow}}
    {DEVICE_ID_1, {  RELAYPIN_1, SWITCHPIN_1, true }},
    {DEVICE_ID_2, {  RELAYPIN_2, SWITCHPIN_2, true }},
       
};

typedef struct {      // struct for the std::map below
  String deviceId;
  bool lastFlipSwitchState;
  unsigned long lastFlipSwitchChange;
  bool activeLow;
} flipSwitchConfig_t;

std::map<int, flipSwitchConfig_t> flipSwitches;    // this map is used to map flipSwitch PINs to deviceId and handling debounce and last flipSwitch state checks
                                                  // it will be setup in "setupFlipSwitches" function, using informations from devices map

void setupRelays() { 
  int relayIndex = 0; // Index for the relays in the devices map
  for (auto &device : devices) {           // for each device (relay, flipSwitch combination)
    int relayPIN = device.second.relayPIN; // get the relay pin
    pinMode(relayPIN, OUTPUT);             // set relay pin to OUTPUT
    // Restore relay state from EEPROM
    bool relayState = readRelayStateFromEEPROM(relayIndex);
    digitalWrite(relayPIN, relayState ? LOW : HIGH); // Active-low logic
    Serial.printf("Restored relay %d state: %s\r\n", relayIndex, relayState ? "ON" : "OFF");
    relayIndex++;
  }
}

void saveRelayStateToEEPROM(int relayIndex, bool state) {
  int address = RELAY_STATE_ADDR_START + relayIndex; // Calculate EEPROM address for the relay
  EEPROM.write(address, state);                      // Write the relay state to EEPROM
  EEPROM.commit();                                   // Commit changes to EEPROM
  Serial.printf("Saved relay %d state: %s to EEPROM at address %d\r\n", relayIndex, state ? "ON" : "OFF", address);
}

bool readRelayStateFromEEPROM(int relayIndex) {
  int address = RELAY_STATE_ADDR_START + relayIndex; // Calculate EEPROM address for the relay
  return EEPROM.read(address);                       // Read the relay state from EEPROM
}

void setupFlipSwitches() {
  for (auto &device : devices)  {                     // for each device (relay / flipSwitch combination)
    flipSwitchConfig_t flipSwitchConfig;              // create a new flipSwitch configuration

    flipSwitchConfig.deviceId = device.first;         // set the deviceId
    flipSwitchConfig.lastFlipSwitchChange = 0;        // set debounce time
    flipSwitchConfig.lastFlipSwitchState = false;     // set lastFlipSwitchState to false (LOW)
    int flipSwitchPIN = device.second.flipSwitchPIN;  // get the flipSwitchPIN
    bool activeLow = device.second.activeLow;         // set the activeLow
    flipSwitchConfig.activeLow = activeLow;           
    flipSwitches[flipSwitchPIN] = flipSwitchConfig;   // save the flipSwitch config to flipSwitches map
    
    if(activeLow) {
      pinMode(flipSwitchPIN, INPUT_PULLUP);           // set the flipSwitch pin to INPUT_PULLUP
    }
    else {
      pinMode(flipSwitchPIN, INPUT);                  // set the flipSwitch pin to INPUT  
    } 
  }
}

bool onPowerState(String deviceId, bool &state) {
  Serial.printf("%s: %s\r\n", deviceId.c_str(), state ? "on" : "off");
  int relayPIN = devices[deviceId].relayPIN; // get the relay pin for corresponding device
 // Active low logic: Turn relay on with LOW, off with HIGH
  digitalWrite(relayPIN, state ? LOW : HIGH);
  // Save relay state to EEPROM
  int relayIndex = std::distance(devices.begin(), devices.find(deviceId)); // Get index of the relay
  saveRelayStateToEEPROM(relayIndex, state);
  
  return true;
}

void handleFlipSwitches() {
  unsigned long actualMillis = millis();                                          // Get current time in milliseconds
  for (auto &flipSwitch : flipSwitches) {                                         // For each flipSwitch in flipSwitches map
    int flipSwitchPIN = flipSwitch.first;                                         // Get the flip switch pin
    bool lastFlipSwitchState = flipSwitch.second.lastFlipSwitchState;             // Get the last flipSwitch state
    unsigned long lastFlipSwitchChange = flipSwitch.second.lastFlipSwitchChange;  // Get the timestamp when flipSwitch was pressed last time (used to debounce / limit events)

    // Debounce logic: Check if sufficient time has passed since the last change
    if (actualMillis - lastFlipSwitchChange > DEBOUNCE_TIME) {                    // If time is > debounce time...
      // Read the current state of the flipSwitch
      bool flipSwitchState = digitalRead(flipSwitchPIN);                          // Read the current flipSwitch state
      if (flipSwitch.second.activeLow) {                                         
        flipSwitchState = !flipSwitchState;                                       // Invert state if active low
      }

      // If the state has changed, handle the flipSwitch event
      if (flipSwitchState != lastFlipSwitchState) {                               // If the flipSwitchState has changed...
        flipSwitch.second.lastFlipSwitchState = flipSwitchState;                 // Update the last flipSwitch state
        flipSwitch.second.lastFlipSwitchChange = actualMillis;                   // Update the last change time

#ifdef TACTILE_BUTTON
        if (flipSwitchState) {                                                   // If the tactile button is pressed
#endif
          String deviceId = flipSwitch.second.deviceId;                          // Get the associated device ID
          int relayPIN = devices[deviceId].relayPIN;                             // Get the relay pin for the device
          bool currentRelayState = digitalRead(relayPIN) == LOW;                 // Check current relay state (active low)

          // Toggle the relay state
          bool newRelayState = !currentRelayState;                               // Determine the new relay state
          digitalWrite(relayPIN, newRelayState ? LOW : HIGH);                    // Update the relay state (active low logic)

          // Save the new state to EEPROM
          int relayIndex = std::distance(devices.begin(), devices.find(deviceId)); // Get the relay index
          saveRelayStateToEEPROM(relayIndex, newRelayState);                      // Save the new relay state

          // Send the new state to SinricPro
          SinricProSwitch &mySwitch = SinricPro[deviceId];                        // Get Switch device from SinricPro
          mySwitch.sendPowerStateEvent(newRelayState);                            // Send the event

          // Debug output
          Serial.printf("Flip switch toggled: Device %s, Relay %d -> %s\n", 
                        deviceId.c_str(), relayPIN, newRelayState ? "ON" : "OFF"); // Log the toggle event
#ifdef TACTILE_BUTTON
        }
#endif
      }
    }
  }
}


void handleTemperaturesensor() {
  unsigned long actualMillis = millis();
  if (actualMillis - lastEvent < EVENT_WAIT_TIME) return; //only check every EVENT_WAIT_TIME milliseconds

  if (SinricPro.isConnected() == false) {
    Serial.printf("Not connected to Sinric Pro...!\r\n");
    return; 
  }

  temperature = dht.readTemperature();          // get actual temperature in °C
//  temperature = dht.getTemperature() * 1.8f + 32;  // get actual temperature in °F
  humidity = dht.readHumidity();                // get actual humidity
    // Serial.println(temperature);
  if (isnan(temperature) || isnan(humidity)) { // reading failed... 
    Serial.printf("DHT reading failed!\r\n");  // print error message
    return;                                    // try again next time
  } 

  if (temperature == lastTemperature || humidity == lastHumidity) return; // if no values changed do nothing...

  SinricProTemperaturesensor &mySensor = SinricPro[TEMP_SENSOR_ID];  // get temperaturesensor device
  bool success = mySensor.sendTemperatureEvent(temperature, humidity); // send event
  if (success) {  // if event was sent successfuly, print temperature and humidity to serial
    Serial.printf("Temperature: %2.1f Celsius\tHumidity: %2.1f%%\r\n", temperature, humidity);
  } else {  // if sending event failed, print error message
    Serial.printf("Something went wrong...could not send Event to server!\r\n");
  }

  lastTemperature = temperature;  // save actual temperature for next compare
  lastHumidity = humidity;        // save actual humidity for next compare
  lastEvent = actualMillis;       // save actual time for next compare
}


void setupWiFi() {
  Serial.printf("\r\n[Wifi]: Connecting");

  pinMode(WIFI_STATUS_CONNECTED_LED, OUTPUT);
  digitalWrite(WIFI_STATUS_CONNECTED_LED, LOW);
  pinMode(WIFI_STATUS_DISCONNECTED_LED, OUTPUT);
  digitalWrite(WIFI_STATUS_DISCONNECTED_LED, HIGH);

  #if defined(ESP8266)
    WiFi.setSleepMode(WIFI_NONE_SLEEP); 
    WiFi.setAutoReconnect(true);
  #elif defined(ESP32)
    WiFi.setSleep(false); 
    WiFi.setAutoReconnect(true);
  #endif

  // Try connecting to the first WiFi network (VIRUS)
  WiFi.begin(WIFI_SSID1, WIFI_PASS1);
  
  // Attempt to connect to the first WiFi for 10 seconds
  unsigned long startAttemptTime = millis();
  while (WiFi.status() != WL_CONNECTED) {
    if (millis() - startAttemptTime >= 10000) {  // 10 seconds timeout
      break;
    }
    delay(250);
  }

  // If connection to the first network fails, connect to the second one (404_Error)
  if (WiFi.status() != WL_CONNECTED) {
    Serial.printf("\r\n[Wifi]: Could not connect to %s, switching to %s...\r\n", WIFI_SSID1, WIFI_SSID2);
    WiFi.disconnect();  // Disconnect from the previous network
    WiFi.begin(WIFI_SSID2, WIFI_PASS2);  // Connect to the second network (404_Error)
    
    // Try to connect to the second WiFi network for another 10 seconds
    startAttemptTime = millis();
    while (WiFi.status() != WL_CONNECTED) {
      if (millis() - startAttemptTime >= 10000) {  // 10 seconds timeout
        break;
      }
      delay(250);
    }
  }

  // If still not connected, indicate failure
  if (WiFi.status() != WL_CONNECTED) {
    Serial.printf("\r\n[Wifi]: Failed to connect to both networks.\r\n");
    digitalWrite(WIFI_STATUS_DISCONNECTED_LED, HIGH);
    return;
  }

  // If connected, display the IP address and update LED status
  Serial.printf("Connected to WiFi network: %s\r\n", WiFi.SSID().c_str());
  Serial.printf("[WiFi]: IP Address: %s\r\n", WiFi.localIP().toString().c_str());
  digitalWrite(WIFI_STATUS_CONNECTED_LED, HIGH);
  digitalWrite(WIFI_STATUS_DISCONNECTED_LED, LOW);
}


void toggleRelay(int relayIndex) {
  String deviceId = (relayIndex == 1) ? DEVICE_ID_1 : DEVICE_ID_2;
  int relayPIN = devices[deviceId].relayPIN;
  bool currentRelayState = digitalRead(relayPIN) == LOW; // Active-low logic
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




void setupSinricPro() {
  SinricProTemperaturesensor &mySensor = SinricPro[TEMP_SENSOR_ID];
  // setup SinricPro
  SinricPro.onConnected([](){ Serial.printf("Connected to SinricPro\r\n");
  digitalWrite(WIFI_STATUS_CONNECTED_LED, HIGH);
  digitalWrite(WIFI_STATUS_DISCONNECTED_LED, LOW);
  }); 
  SinricPro.onDisconnected([](){ Serial.printf("Disconnected from SinricPro\r\n");
    digitalWrite(WIFI_STATUS_CONNECTED_LED, LOW); // Turn off LED when disconnected
    digitalWrite(WIFI_STATUS_DISCONNECTED_LED, HIGH); // Turn off LED when disconnected

   });
  for (auto &device : devices) {
    const char *deviceId = device.first.c_str();
    SinricProSwitch &mySwitch = SinricPro[deviceId];
    mySwitch.onPowerState(onPowerState);
  }

  
  SinricPro.begin(APP_KEY, APP_SECRET);  
}

void setup() {
  Serial.begin(BAUD_RATE);
  EEPROM.begin(EEPROM_SIZE);                 // Initialize EEPROM
  pinMode(WIFI_STATUS_CONNECTED_LED, OUTPUT);        // Set LED pin as OUTPUT
  digitalWrite(WIFI_STATUS_CONNECTED_LED, LOW);     // Ensure LED is off at startup
  pinMode(WIFI_STATUS_DISCONNECTED_LED, OUTPUT);        // Set LED pin as OUTPUT
  digitalWrite(WIFI_STATUS_DISCONNECTED_LED, HIGH);     // Ensure LED is off at startup
  setupRelays();
  setupFlipSwitches();
  dht.begin();
  setupWiFi();
  setupSinricPro();
  irrecv.enableIRIn();
}

void loop() {
  SinricPro.handle();
  handleFlipSwitches();
  handleTemperaturesensor();
  if (irrecv.decode(&results)) {
    unsigned long currentTime = millis();
    if (currentTime - lastReceived > debounceDelay) {
      Serial.println(results.value, HEX);
      
      if (results.value == 0xFFB04F) {  // Relay 1 Hex Code
        toggleRelay(1);
      }
      else if (results.value == 0xFFD827) {  // Relay 2 Hex Code
        toggleRelay(2);
      }
      else if (results.value == 0xFF906F) {  // All relays On/Off
        toggleAllRelays();
      }
      
      lastReceived = currentTime;  // Update last received time
    }
    irrecv.resume();
  }
}
