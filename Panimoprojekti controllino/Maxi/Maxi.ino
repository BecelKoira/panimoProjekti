#include <ArduinoJson.h>
#include <Ethernet.h>
#include <Controllino.h>
#include "ModbusRtu.h"


byte RaspiIP[] = { 10, 65, 32, 140 };
byte ControllinoMaxiIP[] = { 10, 65, 32, 141 };
byte ControllinoMegaIP[] = { 10, 65, 32, 142 };
byte MAC[] = { 0x50, 0xD7, 0x53, 0x00, 0xC8, 0xB3 };
#define TCP_port = 3360
EthernetClient client;
#define OwnIP ControllinoMaxiIP


DynamicJsonDocument doc(1024);
String rawMessage, serializedMessage;

struct JsonPart {  //define structure of the jsonParts array elements
  String key;
  String value;
};

JsonPart jsonParts[10];

uint8_t JsonElements;


Modbus ModbusRtu(0, Serial3, 0);

enum ErrorCode {
    JSON_PARSE_ERROR,
    MODBUS_ERROR,
    COMMAND_ERROR,
    // Add more as needed
};

void errorHandler(ErrorCode errorType, const String& errorMessage, const String& additionalInfo, const String& contextInfo) {
    // Handle the error based on the error type
    switch (errorType) {
        case JSON_PARSE_ERROR:
            Serial.print("JSON Parse Error: ");
            break;
        case MODBUS_ERROR:
            Serial.print("Modbus Error: ");
            break;
        case COMMAND_ERROR:
            Serial.print("Command Error: ");
            break;
        // Add more cases as needed
    }

    // Print the error message and additional information
    Serial.println(errorMessage);
    Serial.println("Additional Info: " + additionalInfo);
    Serial.println("Context: " + contextInfo);

    // Add more error handling logic as needed
    // For example, you might want to log the error, send an alert, etc.
}




void parseJSON() {

}


void commandComparison() {

}


void statusCheak() {

}


void sendJson() {

}


void emergencyStop() {

}


void setHeating() {

}


void modBusStatusMsg(){

}


void createModBusQuerry() {

}


void serializeModbusResponse() {

}



void setup() {
  //Initilizes RS485 for ModBus and Ethernet for communication
  Ethernet.begin(MAC, OwnIP);
  ModbusRtu.begin(19200);

  //Initializes pins D0-D6 to output
  for (int i = 2; i <= 7; i++) {
    pinMode(i, OUTPUT);
  }
}


void loop() {

}
