/**
  ESP state controller firmware for the Open eXtensible Rack System

  Documentation:  
    https://oxrs.io/docs/firmware/state-controller-esp32.html

  Supported hardware:
    https://www.superhouse.tv/product/8-channel-relay-driver-shield/
    https://bmdesigns.com.au/shop/relay16-16-channel-relay-driver/
    https://bmdesigns.com.au/shop/relay128-128-channel-relay-driver/

  GitHub repository:
    https://github.com/OXRS-IO/OXRS-IO-StateController-ESP-FW
*/

/*--------------------------- Libraries -------------------------------*/
#include <Adafruit_MCP23X17.h>        // For MCP23017 I/O buffers
#include <OXRS_Rack32.h>              // Rack32 support
#include <OXRS_Output.h>              // For output handling
#include "logo.h"                     // Embedded maker logo

/*--------------------------- Constants -------------------------------*/
// Serial
#define       SERIAL_BAUD_RATE      115200

// Can have up to 8x MCP23017s on a single I2C bus
const byte    MCP_I2C_ADDRESS[]     = { 0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27 };
const uint8_t MCP_COUNT             = sizeof(MCP_I2C_ADDRESS);

// Each MCP23017 has 16 I/O pins
#define       MCP_PIN_COUNT         16

// Speed up the I2C bus to get faster event handling
#define       I2C_CLOCK_SPEED       400000L

// Internal constants used when output type parsing fails
#define       INVALID_OUTPUT_TYPE   99

/*--------------------------- Global Variables ------------------------*/
// Each bit corresponds to an MCP found on the IC2 bus
uint8_t g_mcps_found = 0;

// How many pins on each MCP are we controlling (defaults to all 16)
// Set via "outputsPerMcp" integer config option - should be set via
// the REST API so it is persisted to SPIFFS and loaded early enough
// in the boot sequence to configure the LCD and adoption payloads
uint8_t g_mcp_output_pins = MCP_PIN_COUNT;

/*--------------------------- Instantiate Globals ---------------------*/
// Rack32 handler
OXRS_Rack32 rack32(FW_LOGO);

// I/O buffers
Adafruit_MCP23X17 mcp23017[MCP_COUNT];

// Output handlers
OXRS_Output oxrsOutput[MCP_COUNT];

/*--------------------------- Program ---------------------------------*/
uint8_t getMaxIndex()
{
  // Count how many MCPs were found
  uint8_t mcpCount = 0;
  for (uint8_t mcp = 0; mcp < MCP_COUNT; mcp++)
  {
    if (bitRead(g_mcps_found, mcp) != 0) { mcpCount++; }
  }

  // Remember our indexes are 1-based
  return mcpCount * g_mcp_output_pins;  
}

void createOutputTypeEnum(JsonObject parent)
{
  JsonArray typeEnum = parent["enum"].to<JsonArray>();

  typeEnum.add("relay");
  typeEnum.add("motor");
  typeEnum.add("timer");  
}

uint8_t parseOutputType(const char * outputType)
{
  if (strcmp(outputType, "relay") == 0) { return RELAY; }
  if (strcmp(outputType, "motor") == 0) { return MOTOR; }
  if (strcmp(outputType, "timer") == 0) { return TIMER; }

  rack32.println(F("[scon] invalid output type"));
  return INVALID_OUTPUT_TYPE;
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

void setDefaultOutputType(uint8_t outputType)
{
  // Set all pins on all MCPs to this default output type
  for (uint8_t mcp = 0; mcp < MCP_COUNT; mcp++)
  {
    if (bitRead(g_mcps_found, mcp) == 0)
      continue;

    for (uint8_t pin = 0; pin < g_mcp_output_pins; pin++)
    {
      oxrsOutput[mcp].setType(pin, outputType);
    }
  }
}

uint8_t getIndex(JsonVariant json)
{
  if (!json.containsKey("index"))
  {
    rack32.println(F("[scon] missing index"));
    return 0;
  }
  
  uint8_t index = json["index"].as<uint8_t>();

  // Check the index is valid for this device
  if (index <= 0 || index > getMaxIndex())
  {
    rack32.println(F("[scon] invalid index"));
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

  JsonDocument json;
  json["index"] = index;
  json["type"] = outputType;
  json["event"] = eventType;
  
  if (!rack32.publishStatus(json.as<JsonVariant>()))
  {
    rack32.print(F("[scon] [failover] "));
    serializeJson(json, rack32);
    rack32.println();

    // TODO: add failover handling code here
  }
}

/**
  Config handler
 */
void setConfigSchema()
{
  // Define our config schema
  JsonDocument json;

  JsonObject outputsPerMcp = json["outputsPerMcp"].to<JsonObject>();
  outputsPerMcp["title"] = "Number Of Outputs Per MCP";
  outputsPerMcp["description"] = "Number of outputs connected to each MCP23017 I/O chip, which is dependent on the relay driver used (must be either 8 or 16, defaults to 16).";
  outputsPerMcp["type"] = "integer";
  outputsPerMcp["minimum"] = 8;
  outputsPerMcp["maximum"] = MCP_PIN_COUNT;
  outputsPerMcp["multipleOf"] = 8;

  JsonObject defaultOutputType = json["defaultOutputType"].to<JsonObject>();
  defaultOutputType["title"] = "Default Output Type";
  defaultOutputType["description"] = "Set the default output type for anything without explicit configuration below. Defaults to ‘relay’.";
  createOutputTypeEnum(defaultOutputType);

  JsonObject outputs = json["outputs"].to<JsonObject>();
  outputs["title"] = "Output Configuration";
  outputs["description"] = "Add configuration for each output in use on your device. The 1-based index specifies which output you wish to configure. The type defines how an output is controlled. For ‘timer’ outputs you can define how long it should stay ON (defaults to 60 seconds). Interlocking two outputs ensures they are never both on at the same time (useful for controlling motors).";
  outputs["type"] = "array";
  
  JsonObject items = outputs["items"].to<JsonObject>();
  items["type"] = "object";

  JsonObject properties = items["properties"].to<JsonObject>();

  JsonObject index = properties["index"].to<JsonObject>();
  index["title"] = "Index";
  index["type"] = "integer";
  index["minimum"] = 1;
  index["maximum"] = getMaxIndex();

  JsonObject type = properties["type"].to<JsonObject>();
  type["title"] = "Type";
  createOutputTypeEnum(type);

  JsonObject timerSeconds = properties["timerSeconds"].to<JsonObject>();
  timerSeconds["title"] = "Timer (seconds)";
  timerSeconds["type"] = "integer";
  timerSeconds["minimum"] = 1;

  JsonObject interlockIndex = properties["interlockIndex"].to<JsonObject>();
  interlockIndex["title"] = "Interlock With Index";
  interlockIndex["type"] = "integer";
  interlockIndex["minimum"] = 1;
  interlockIndex["maximum"] = getMaxIndex();

  JsonArray required = items["required"].to<JsonArray>();
  required.add("index");

  // Pass our config schema down to the Rack32 library
  rack32.setConfigSchema(json.as<JsonVariant>());
}

void jsonOutputConfig(JsonVariant json)
{
  uint8_t index = getIndex(json);
  if (index == 0) return;

  // Work out the MCP and pin we are configuring
  uint8_t mcp = (index - 1) / g_mcp_output_pins;
  uint8_t pin = (index - 1) % g_mcp_output_pins;

  if (json.containsKey("type"))
  {
    uint8_t outputType = parseOutputType(json["type"]);    

    if (outputType != INVALID_OUTPUT_TYPE)
    {
      oxrsOutput[mcp].setType(pin, outputType);
    }
  }
  
  if (json.containsKey("timerSeconds"))
  {
    if (json["timerSeconds"].isNull())
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
     
      uint8_t interlock_mcp = (interlock_index - 1) / g_mcp_output_pins;
      uint8_t interlock_pin = (interlock_index - 1) % g_mcp_output_pins;
  
      if (interlock_mcp == mcp)
      {
        oxrsOutput[mcp].setInterlock(pin, interlock_pin);
      }
      else
      {
        rack32.println(F("[scon] lock must be with pin on same mcp"));
      }
    }
  }
}

void jsonConfig(JsonVariant json)
{
  if (json.containsKey("outputsPerMcp"))
  {
    g_mcp_output_pins = json["outputsPerMcp"].as<uint8_t>();
  }
  
  if (json.containsKey("defaultOutputType"))
  {
    uint8_t outputType = parseOutputType(json["defaultOutputType"]);

    if (outputType != INVALID_OUTPUT_TYPE)
    {
      setDefaultOutputType(outputType);
    }
  }

  if (json.containsKey("outputs"))
  {
    for (JsonVariant output : json["outputs"].as<JsonArray>())
    {
      jsonOutputConfig(output);
    }
  }
}

/**
  Command handler
 */
void setCommandSchema()
{
  // Define our command schema
  JsonDocument json;

  JsonObject outputs = json["outputs"].to<JsonObject>();
  outputs["title"] = "Output Commands";
  outputs["description"] = "Send commands to one or more outputs on your device. The 1-based index specifies which output you wish to command. The type is used to validate the configuration for this output matches the command. Supported commands are ‘on’ or ‘off’ to change the output state, or ‘query’ to publish the current state to MQTT.";
  outputs["type"] = "array";
  
  JsonObject items = outputs["items"].to<JsonObject>();
  items["type"] = "object";

  JsonObject properties = items["properties"].to<JsonObject>();

  JsonObject index = properties["index"].to<JsonObject>();
  index["title"] = "Index";
  index["type"] = "integer";
  index["minimum"] = 1;
  index["maximum"] = getMaxIndex();

  JsonObject type = properties["type"].to<JsonObject>();
  type["title"] = "Type";
  createOutputTypeEnum(type);

  JsonObject command = properties["command"].to<JsonObject>();
  command["title"] = "Command";
  command["type"] = "string";
  JsonArray commandEnum = command["enum"].to<JsonArray>();
  commandEnum.add("query");
  commandEnum.add("on");
  commandEnum.add("off");

  JsonArray required = items["required"].to<JsonArray>();
  required.add("index");
  required.add("command");

  // Pass our config schema down to the Rack32 library
  rack32.setCommandSchema(json.as<JsonVariant>());
}

void jsonOutputCommand(JsonVariant json)
{
  uint8_t index = getIndex(json);
  if (index == 0) return;

  // Work out the MCP and pin we are processing
  uint8_t mcp = (index - 1) / g_mcp_output_pins;
  uint8_t pin = (index - 1) % g_mcp_output_pins;

  // Get the output type for this pin
  uint8_t type = oxrsOutput[mcp].getType(pin);
  
  if (json.containsKey("type"))
  {
    if (parseOutputType(json["type"]) != type)
    {
      rack32.println(F("[scon] command type doesn't match configured type"));
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
        rack32.println(F("[scon] invalid command"));
      }
    }
  }
}

void jsonCommand(JsonVariant json)
{
  if (json.containsKey("outputs"))
  {
    for (JsonVariant output : json["outputs"].as<JsonArray>())
    {
      jsonOutputCommand(output);
    }
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
  uint8_t raw_index = (g_mcp_output_pins * mcp) + pin;
  uint8_t index = raw_index + 1;
  
  // Update the MCP pin - i.e. turn the relay on/off (LOW/HIGH)
  mcp23017[mcp].digitalWrite(pin, state);

  // Publish the event
  publishEvent(index, type, state);
}

/**
  I2C bus
 */
void scanI2CBus()
{
  rack32.println(F("[scon] scanning for I/O buffers..."));

  for (uint8_t mcp = 0; mcp < MCP_COUNT; mcp++)
  {
    rack32.print(F(" - 0x"));
    rack32.print(MCP_I2C_ADDRESS[mcp], HEX);
    rack32.print(F("..."));

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

      // Initialise output handlers
      oxrsOutput[mcp].begin(outputEvent, RELAY);
      
      rack32.println(F("MCP23017"));
    }
    else
    {
      rack32.println(F("empty"));
    }
  }
}

/**
  Setup
*/
void setup()
{
  // Start serial and let settle
  Serial.begin(SERIAL_BAUD_RATE);
  delay(1000);
  Serial.println(F("[scon] starting up..."));

  // Start the I2C bus
  Wire.begin();

  // Scan the I2C bus and set up I/O buffers
  scanI2CBus();

  // Start Rack32 hardware
  rack32.begin(jsonConfig, jsonCommand);

  // Set up port display
  if (g_mcp_output_pins == 8)
  {
    rack32.getLCD()->drawPorts(PORT_LAYOUT_OUTPUT_AUTO_8, g_mcps_found);
  }
  else
  {
    rack32.getLCD()->drawPorts(PORT_LAYOUT_OUTPUT_AUTO, g_mcps_found);
  }

  // Set up config/command schemas (for self-discovery and adoption)
  setConfigSchema();
  setCommandSchema();
  
  // Speed up I2C clock for faster scan rate (after bus scan)
  Wire.setClock(I2C_CLOCK_SPEED);
}

/**
  Main processing loop
*/
void loop()
{
  // Iterate through each of the MCP23017s
  for (uint8_t mcp = 0; mcp < MCP_COUNT; mcp++)
  {
    if (bitRead(g_mcps_found, mcp) == 0) 
      continue;
    
    // Check for any output events
    oxrsOutput[mcp].process();

    // Read the values for all 16 pins on this MCP
    uint16_t io_value = mcp23017[mcp].readGPIOAB();

    // Show port animations
    rack32.getLCD()->process(mcp, io_value);
  }

  // Let Rack32 hardware handle any events etc
  rack32.loop();
}
