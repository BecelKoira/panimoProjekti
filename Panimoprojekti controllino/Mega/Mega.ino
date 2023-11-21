#include <ArduinoJson.h>
#include <Ethernet.h>
#include <Controllino.h>
#include <OneWire.h>
#include <DallasTemperature.h>

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


//OneWire stuff
#define OneWire_data_pin 11
OneWire oneWire(OneWire_data_pin);
DallasTemperature sensors(&oneWire);


enum ErrorCode {
  JSON_PARSE_ERROR,
  MODBUS_ERROR,
  COMMAND_ERROR,
  // Add more as needed
};

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
        statusCheak();
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


void statusCheak() {
  
}


void sendJson(String serializedMessage) {
  
}


void getTemperature() {
}


void readPneumaticState() {
}


void readPhValues() {
}


void setup() {
  Ethernet.begin(MAC, OwnIP);
  sensors.begin();

  pinMode(OneWire_data_pin, INPUT);

  //Future for loop to initialize the analog pins for dataGathering
}

void loop() {
}
