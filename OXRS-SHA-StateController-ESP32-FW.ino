/**
  ESP32 state controller firmware for the Open eXtensible Rack System
  
  See https://github.com/SuperHouse/OXRS-SHA-StateController-ESP32-FW for documentation.

  Compile options:
    ESP32

  External dependencies. Install using the Arduino library manager:
    "Adafruit_MCP23X17"
    "PubSubClient" by Nick O'Leary
    "OXRS-SHA-MQTT-ESP32-LIB" by SuperHouse Automation Pty
    "OXRS-SHA-IOHandler-ESP32-LIB" by SuperHouse Automation Pty

  Based on the multi-channel relay driver hardware found here:
    https://www.superhouse.tv/product/8-channel-relay-driver-shield/
    https://bmdesigns.com.au/shop/relay16-16-channel-relay-driver/
    https://bmdesigns.com.au/shop/relay128-128-channel-relay-driver/
    
  Bugs/Features:
   - See GitHub issues list.

  Written by Ben Jones on behalf of www.superhouse.tv
    https://github.com/sumnerboy12/
    https://github.com/superhouse/
  Copyright 2019-2021 SuperHouse Automation Pty Ltd
*/

/*--------------------------- Version ------------------------------------*/
#define FW_NAME    "OXRS-SHA-StateController-ESP32-FW"
#define FW_VERSION "1.0.0"

/*--------------------------- Configuration ------------------------------*/
// Should be no user configuration in this file, everything should be in;
#include "config.h"

/*--------------------------- Libraries ----------------------------------*/
#include <Wire.h>                     // For I2C
#include <Ethernet.h>                 // For networking
#include <PubSubClient.h>             // For MQTT
#include <OXRS_MQTT.h>                // For MQTT
#include <Adafruit_MCP23X17.h>        // For MCP23017 I/O buffers
#include <OXRS_Output.h>              // For output handling

#ifdef ARDUINO_ARCH_ESP32
#include <WiFi.h>                     // Also required for Ethernet to get MAC
#endif

/*--------------------------- Constants ----------------------------------*/
// Each MCP23017 has 16 I/O pins
#define MCP_PIN_COUNT   16

/*--------------------------- Global Variables ---------------------------*/
// Each bit corresponds to an MCP found on the IC2 bus
uint8_t g_mcps_found = 0;

// Last time the watchdog was reset
uint32_t g_watchdog_last_reset_ms = 0L;

/*--------------------------- Function Signatures ------------------------*/
void mqttCallback(char * topic, uint8_t * payload, unsigned int length);

/*--------------------------- Instantiate Global Objects -----------------*/
// I/O buffers
Adafruit_MCP23X17 mcp23017[MCP_COUNT];

// Output handlers
OXRS_Output oxrsOutput[MCP_COUNT];

// Ethernet client
EthernetClient ethernet;

// MQTT client
PubSubClient mqttClient(MQTT_BROKER, MQTT_PORT, mqttCallback, ethernet);
OXRS_MQTT mqtt(mqttClient);

/*--------------------------- Program ------------------------------------*/
/**
  Setup
*/
void setup()
{
  // Startup logging to serial
  Serial.begin(SERIAL_BAUD_RATE);
  Serial.println();
  Serial.println(F("==============================="));
  Serial.println(F("     OXRS by SuperHouse.tv"));
  Serial.println(FW_NAME);
  Serial.print  (F("            v"));
  Serial.println(FW_VERSION);
  Serial.println(F("==============================="));

  // Start the I2C bus
  Wire.begin();

  // Set up watchdog
  initialiseWatchdog();

  // Set up the I/O buffers
  initialiseIOBuffers();

  // Set I2C clock to 400 kHz for faster scan rate  
  Wire.setClock(I2C_CLOCK_SPEED);  

  // Determine Ethernet MAC address
  byte ethernet_mac[6];
  if (ENABLE_MAC_ADDRESS_ROM)
  {
#ifdef ARDUINO_ARCH_ESP32
    Serial.print(F("Getting Ethernet MAC address from ESP32 Base MAC: "));
    WiFi.macAddress(ethernet_mac);  // Temporarily populate Ethernet MAC with ESP32 Base MAC
    ethernet_mac[5] += 3;           // Ethernet MAC is Base MAC + 3 (see https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/system.html#mac-address)
#else
    Serial.print(F("Getting Ethernet MAC address from ROM: "));
    ethernet_mac[0] = readRegister(MAC_I2C_ADDRESS, 0xFA);
    ethernet_mac[1] = readRegister(MAC_I2C_ADDRESS, 0xFB);
    ethernet_mac[2] = readRegister(MAC_I2C_ADDRESS, 0xFC);
    ethernet_mac[3] = readRegister(MAC_I2C_ADDRESS, 0xFD);
    ethernet_mac[4] = readRegister(MAC_I2C_ADDRESS, 0xFE);
    ethernet_mac[5] = readRegister(MAC_I2C_ADDRESS, 0xFF);
#endif
  }
  else
  {
    Serial.print(F("Using static MAC address: "));
    memcpy(ethernet_mac, STATIC_MAC, sizeof(ethernet_mac));
  }
  char mac_address[18];
  sprintf_P(mac_address, PSTR("%02X:%02X:%02X:%02X:%02X:%02X"), ethernet_mac[0], ethernet_mac[1], ethernet_mac[2], ethernet_mac[3], ethernet_mac[4], ethernet_mac[5]);
  Serial.println(mac_address);

  // Set up Ethernet
  Ethernet.init(ETHERNET_CS_PIN);
  resetWiznetChip();
  if (ENABLE_DHCP)
  {
    Serial.print(F("Getting IP address via DHCP: "));
    Ethernet.begin(ethernet_mac);
  }
  else
  {
    Serial.print(F("Using static IP address: "));
    Ethernet.begin(ethernet_mac, STATIC_IP, STATIC_DNS);
  }
  Serial.println(Ethernet.localIP());

  // Set up our MQTT connection
  mqtt.setClientId("osm", ethernet_mac);
  mqtt.setAuth(MQTT_USERNAME, MQTT_PASSWORD);
  mqtt.setTopicPrefix(MQTT_TOPIC_PREFIX);
  mqtt.setTopicSuffix(MQTT_TOPIC_SUFFIX);

  // Listen for config and command messages
  mqtt.onConfig(mqttConfig);
  mqtt.onCommand(mqttCommand);
}

/**
  Main processing loop
*/
void loop()
{
  // Check our DHCP lease is still ok
  Ethernet.maintain();

  // Check our MQTT broker connection is still ok
  mqtt.loop();

  // Check each set of outputs for delay/timer activations
  for (uint8_t i = 0; i < MCP_COUNT; i++)
  {
    oxrsOutput[i].process();
  }

  // Pat the watchdog once our processing loop completes
  patWatchdog();
}

/**
  MQTT
*/
void mqttCallback(char * topic, uint8_t * payload, unsigned int length) 
{
  // Pass this message down to our MQTT handler
  mqtt.receive(topic, payload, length);
}

void mqttConfig(JsonObject json)
{
  uint8_t index = getIndex(json);
  if (index == 0) return;

  // Work out the MCP and pin we are configuring
  uint8_t mcp = (index - 1) / MCP_PIN_COUNT;
  uint8_t pin = (index - 1) % MCP_PIN_COUNT;

  if (json.containsKey("type"))
  {
    if (json["type"].isNull() || strcmp(json["type"], "relay") == 0)
    {
      oxrsOutput[mcp].setType(pin, RELAY);
    }
    else if (strcmp(json["type"], "motor") == 0)
    {
      oxrsOutput[mcp].setType(pin, MOTOR);
    }
    else if (strcmp(json["type"], "timer") == 0)
    {
      oxrsOutput[mcp].setType(pin, TIMER);
    }
    else 
    {
      Serial.println(F("[erro] invalid output type"));
    }
  }
  
  if (json.containsKey("timerSeconds"))
  {
    if (json["type"].isNull())
    {
      oxrsOutput[mcp].setTimer(pin, DEFAULT_TIMER_SECS);
    }
    else
    {
      oxrsOutput[mcp].setTimer(pin, json["timerSeconds"].as<int>());      
    }
  }
  
  if (json.containsKey("interlockIndex"))
  {
    // If an empty message then treat as 'unlocked' - i.e. interlock with ourselves
    if (json["interlockIndex"].isNull())
    {
      oxrsOutput[mcp].setInterlock(pin, pin);
    }
    else
    {
      uint8_t interlock_index = json["interlockIndex"].as<uint8_t>();
     
      uint8_t interlock_mcp = (interlock_index - 1) / MCP_PIN_COUNT;
      uint8_t interlock_pin = (interlock_index - 1) % MCP_PIN_COUNT;
  
      if (interlock_mcp == mcp)
      {
        oxrsOutput[mcp].setInterlock(pin, interlock_pin);
      }
      else
      {
        Serial.println(F("[erro] lock must be with pin on same mcp"));
      }
    }
  }
}

void mqttCommand(JsonObject json)
{
  uint8_t index = getIndex(json);
  if (index == 0) return;

  // Work out the MCP and pin we are processing
  uint8_t mcp = (index - 1) / MCP_PIN_COUNT;
  uint8_t pin = (index - 1) % MCP_PIN_COUNT;

  // Get the output type for this pin
  uint8_t type = oxrsOutput[mcp].getType(pin);
  
  if (json.containsKey("type"))
  {
    if ((strcmp(json["type"], "relay") == 0 && type != RELAY) ||
        (strcmp(json["type"], "motor") == 0 && type != MOTOR) ||
        (strcmp(json["type"], "timer") == 0 && type != TIMER))
    {
      Serial.println(F("[erro] command type doesn't match configured type"));
      return;
    }
  }
  
  if (json.containsKey("command"))
  {
    if (json["command"].isNull() || strcmp(json["command"], "query") == 0)
    {
      // Publish a status event with the current state
      uint8_t state = mcp23017[mcp].digitalRead(pin);
      publishEvent(index, type, state);
    }
    else
    {
      // Send this command down to our output handler to process
      if (strcmp(json["command"], "on") == 0)
      {
        oxrsOutput[mcp].handleCommand(mcp, pin, RELAY_ON);
      }
      else if (strcmp(json["command"], "off") == 0)
      {
        oxrsOutput[mcp].handleCommand(mcp, pin, RELAY_OFF);
      }
      else 
      {
        Serial.println(F("[erro] invalid command"));
      }
    }
  }
}

uint8_t getIndex(JsonObject json)
{
  if (!json.containsKey("index"))
  {
    Serial.println(F("[erro] missing index"));
    return 0;
  }
  
  uint8_t index = json["index"].as<uint8_t>();
  
  // Check the index is valid for this device
  if (index <= 0 || index > (MCP_COUNT * MCP_PIN_COUNT))
  {
    Serial.println(F("[erro] invalid index"));
    return 0;
  }

  return index;
}

void publishEvent(uint8_t index, uint8_t type, uint8_t state)
{
  char outputType[8];
  getOutputType(outputType, type);
  char eventType[7];
  getEventType(eventType, type, state);

  // Build JSON payload for this event
  StaticJsonDocument<64> json;
  json["index"] = index;
  json["type"] = outputType;
  json["event"] = eventType;
  
  // Publish to MQTT
  if (!mqtt.publishStatus(json.as<JsonObject>()))
  {
    Serial.println("FAILOVER!!!");
  }
}

void getOutputType(char outputType[], uint8_t type)
{
  // Determine what type of output we have
  sprintf_P(outputType, PSTR("error"));
  switch (type)
  {
    case MOTOR:
      sprintf_P(outputType, PSTR("motor"));
      break;
    case RELAY:
      sprintf_P(outputType, PSTR("relay"));
      break;
    case TIMER:
      sprintf_P(outputType, PSTR("timer"));
      break;
  }
}

void getEventType(char eventType[], uint8_t type, uint8_t state)
{
  // Determine what event we need to publish
  sprintf_P(eventType, PSTR("error"));
  switch (state)
  {
    case RELAY_ON:
      sprintf_P(eventType, PSTR("on"));
      break;
    case RELAY_OFF:
      sprintf_P(eventType, PSTR("off"));
      break;
  }
}

/**
  Event handlers
*/
void outputEvent(uint8_t id, uint8_t output, uint8_t type, uint8_t state)
{
  // Determine the index (1-based)
  uint8_t mcp = id;
  uint8_t pin = output;
  uint8_t raw_index = (MCP_PIN_COUNT * mcp) + pin;
  uint8_t index = raw_index + 1;
  
  // Update the MCP pin - i.e. turn the relay on/off (LOW/HIGH)
  mcp23017[mcp].digitalWrite(pin, state);

  // Publish the event
  publishEvent(index, type, state);
}

/**
  Watchdog
*/
void initialiseWatchdog()
{
  if (ENABLE_WATCHDOG)
  {
    Serial.print(F("Watchdog enabled on pin "));
    Serial.println(WATCHDOG_PIN);

    pinMode(WATCHDOG_PIN, OUTPUT);
    digitalWrite(WATCHDOG_PIN, LOW);
  }
  else
  {
    Serial.println(F("Watchdog NOT enabled"));
  }
}

void patWatchdog()
{
  if (ENABLE_WATCHDOG)
  {
    if ((millis() - g_watchdog_last_reset_ms) > WATCHDOG_RESET_MS)
    {
      digitalWrite(WATCHDOG_PIN, HIGH);
      delay(WATCHDOG_PULSE_MS);
      digitalWrite(WATCHDOG_PIN, LOW);

      g_watchdog_last_reset_ms = millis();
    }
  }
}

/**
  I/O buffers
 */
void initialiseIOBuffers()
{
  Serial.println(F("Scanning for devices on the I2C bus..."));

  // Scan for MCP's
  for (uint8_t mcp = 0; mcp < MCP_COUNT; mcp++)
  {
    Serial.print(F(" - 0x"));
    Serial.print(MCP_I2C_ADDRESS[mcp], HEX);
    Serial.print(F("..."));

    // Check if there is anything responding on this address
    Wire.beginTransmission(MCP_I2C_ADDRESS[mcp]);
    if (Wire.endTransmission() == 0)
    {
      bitWrite(g_mcps_found, mcp, 1);
      
      // If an MCP23017 was found then initialise and configure the outputs
      mcp23017[mcp].begin_I2C(MCP_I2C_ADDRESS[mcp]);
      for (uint8_t pin = 0; pin < MCP_PIN_COUNT; pin++)
      {
        mcp23017[mcp].pinMode(pin, OUTPUT);
        mcp23017[mcp].digitalWrite(pin, RELAY_OFF);
      }

      // Listen for output events
      oxrsOutput[mcp].onEvent(outputEvent);
      
      Serial.println(F("MCP23017"));
    }
    else
    {
      // No MCP found at this address
      Serial.println(F("empty"));
    }
  }
}

// Read 1 byte from I2C device register
uint8_t readRegister(int adr, uint8_t reg) 
{
  Wire.beginTransmission(adr);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(adr, 1);
  while (!Wire.available()) {}
  return Wire.read();
}

/**
  Ethernet
 */
void resetWiznetChip()
{
  // Reset the Wiznet Ethernet chip
#ifdef WIZNET_RESET_PIN
  Serial.print("Resetting Wiznet W5500 Ethernet chip...  ");
  pinMode(WIZNET_RESET_PIN, OUTPUT);
  digitalWrite(WIZNET_RESET_PIN, HIGH);
  delay(250);
  digitalWrite(WIZNET_RESET_PIN, LOW);
  delay(50);
  digitalWrite(WIZNET_RESET_PIN, HIGH);
  delay(350);
  Serial.println("Done.");
#endif
}
