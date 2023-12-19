#include <ArduinoJson.h>
#include <Ethernet.h>
#include <Controllino.h>
#include "ModbusRtu.h"

//Ethernet stuff
byte RaspiIP[] = { 10, 65, 32, 140 };
byte ControllinoMaxiIP[] = { 10, 65, 32, 141 };
byte ControllinoMegaIP[] = { 10, 65, 32, 142 };
byte MAC[] = { 0x50, 0xD7, 0x53, 0x00, 0xC8, 0xB4 };
#define TCP_port = 3360
EthernetClient client;
#define OwnIP ControllinoMegaIP


//JSON stuff
DynamicJsonDocument doc(1024);
String rawMessage, serializedMessage;

struct JsonPart {  //define structure of the jsonParts array elements
  String key;
  String value;
};

JsonPart jsonParts[10];

uint8_t JsonElements;


struct SensorConfig {
  String sensorID;
  int pin;;
  int reference_0C;
  int reference_100C;
};

SensorConfig sensorConfigs[] = {
  {"1", A0, 0, 100},  // Sensor connected to analog pin A0
  {"2", A1, 10, 110}, // Sensor connected to analog pin A1
};


enum ErrorCode {
  JSON_PARSE_ERROR,
  MODBUS_ERROR,
  COMMAND_ERROR,
  // Add more as needed
};

void statusCheck() {

}

//Funtion to handle different error and sending them over to Raspberry PI
//exepected input for this funtion is Error code from list "ErrorCode", General overview of the error,
//actual error message and contextInfo about the error generally the funtion that called the error
void errorHandler(ErrorCode errorType, const String& errorMessage, const String& additionalInfo, const String& contextInfo) {
  //code to hanlde some Errors goes here later


  // Code to send the error message to Raspberry Pi
  DynamicJsonDocument doc(256);
  JsonObject outputObj = doc.to<JsonObject>();

  outputObj["ID"] = "0";  // Assuming a constant ID for the messages
  outputObj["SlaveID"] = "2";
  outputObj["ErrorType"] = errorType;
  outputObj["ErrorMessage"] = errorMessage;
  outputObj["ErrorInfo"] = additionalInfo;
  outputObj["contextInfo"] = contextInfo;

  String serializedMessage = "";
  serializeJson(outputObj, serializedMessage);

  sendJson(serializedMessage);
}


void parseJSON(String rawMessage) {
  DeserializationError error = deserializeJson(doc, rawMessage);

  if (error) {
    // Send error to errorHandler
    errorHandler(JSON_PARSE_ERROR, "Error parsing Incoming JSON", error.f_str(), "In function: parseJSON");
    return;
  }

  JsonObject obj = doc.as<JsonObject>();
  int i = 0;

  for (JsonPair kv : obj) {
    if (i >= 10) {
      // Send an error to Raspberry Pi (JSON message has too many parts)
      errorHandler(JSON_PARSE_ERROR, "Error parsing Incoming JSON", "Incoming JSON message has too many parts", "In function: parseJSON");
      break;
    }

    jsonParts[i].key = kv.key().c_str();
    jsonParts[i].value = kv.value().as<String>();
    i++;
  }

  commandComparison(jsonParts);
}


void commandComparison(JsonPart jsonParts[]) {
  enum commandList {
    statusCheck = 1,
    sendModbusCommand = 2,
    getTempeture = 3,
    setHeating = 4,
    getTemps = 5,
    readPneumaticState = 6,
    readPhValues = 7,
  };

  int command = jsonParts[1].value.toInt();
  if (jsonParts[0].key != "ID" && jsonParts[0].value != "2") {
  }
  else if (jsonParts[1].key != "command") {
    errorHandler(COMMAND_ERROR, "The JSON structure error", "incoming JSON message key 0 != command", "In funtion CommandComparison");
    // Send an error message (wrong JSON structure)
  } else {
    switch (command) {
      case statusCheck:
        statusCheck();
        break;
      case getTemps:
        getTemperature();
        break;
      default:
        // Send an error message that the command is not understood
        errorHandler(COMMAND_ERROR, "The JSON structure error", "incoming JSON message command not understanded", "In funtion CommandComparison");        
        break;
    }
  }
}





void sendJson(String serializedMessage) {
  // Function implementation
}



void getTemperature() {
  // Clear the serializedMessage string
  String serializedMessage = "";

  // Rescan the OneWire bus
  sensors.begin();

  // Request temperatures from all devices on the bus
  sensors.requestTemperatures();

  JsonObject sensorObj = doc.to<JsonObject>();

  // Loop through each device on the bus
  int deviceCount = sensors.getDeviceCount();
  if (deviceCount == 0) {
    // No sensors found on the bus
    sensorObj["error"] = "no sensors on the bus";
  } else {
    for (int i = 0; i < deviceCount; i++) {
      // Get the ROM address of the sensor
      DeviceAddress deviceAddress;
      if (sensors.getAddress(deviceAddress, i)) {
        // Convert the address to a hex string
        String addressStr = "";
        for (uint8_t j = 0; j < 8; j++) {
          if (deviceAddress[j] < 16) addressStr += "0";
          addressStr += String(deviceAddress[j], HEX);
        }

        // Get the temperature from the sensor
        float tempC = sensors.getTempC(deviceAddress);

        // Check for invalid temperature readings
        if (tempC == -127.00) {
          sensorObj[addressStr] = "Error";
        } else {
          sensorObj[addressStr] = tempC;
        }
      }
    }
  }

  // Serialize the JSON object to a string
  serializeJson(sensorObj, serializedMessage);

  // Send the serialized JSON string
  sendJson(serializedMessage);
}


void readAnalogTempetureData() {
  // Clear the serializedMessage string
  String serializedMessage = "";

  // Create a JSON object to hold sensor data
  JsonObject sensorObj = doc.to<JsonObject>();

  for (const SensorConfig& config : sensorConfigs) {
    int analogValue = analogRead(config.pin);
    float temperature = convertToCelsius(analogValue, config);


    // Add the sensor data to the JSON object
    sensorObj[config.sensorID] = temperature;
  }

  // Serialize the JSON object to a string
  serializeJson(sensorObj, serializedMessage);

  // Send the serialized JSON string
  sendJson(serializedMessage);
}


float convertToCelsius(int analogValue, const SensorConfig& config) {
  float temperature_C = 10 + ((100-10)/(config.reference_100C - config.reference_0C)) * (analogValue - config.reference_0C);
  return temperature_C;
}


void readPneumaticState() {
}


void readPhValues() {
}


void setup() {
  Ethernet.begin(MAC, OwnIP);


  pinMode(OneWire_data_pin, INPUT);

  //Future for loop to initialize the analog pins for dataGathering
}

void loop() {
}
