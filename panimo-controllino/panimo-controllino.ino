// Panimo Projects Controllino Mega Code
// This code gathers sensor data, controls heating, and operates the FC51 drives.
// It is currently fully functional for testing via the Serial interface.
// However, it still requires error handling and correction.
// Communication between the Raspberry Pi and Controllino is done using Ethernet TCP/IP protocol, and messages are sent in JSON format.
// For communication with the FC51 drives, the code uses ModBusRTU via a half-duplex RS-485 interface.
// Temperature data is gathered using the OneWire bus.

 

//Include the libaries for Controllino
#include <SPI.h>
#include <Controllino.h>

//Include the libaries for JSON documents
#include <ArduinoJson.h>

//Include the libaries for OneWire temperature sensors
#include <OneWire.h>
#include <DallasTemperature.h>

#include <Ethernet.h>   //Libary for communication via Ethernet
#include "ModbusRtu.h"  //Libary for Modbus communication


//machine state variables
uint8_t sendModbusQuerry = 0;

//list of all the available commands that can be sent
#define statusCheck 0
#define sendModbusCommand 1
#define getTempeture 2
#define setHeating 3

//Internet stuff
byte RaspiIP[] = { 10, 65, 32, 140 };
byte ControllinoIP[] = { 10, 65, 32, 141 };
byte MAC[] = { 0x50, 0xD7, 0x53, 0x00, 0xC8, 0xB4 };
#define TCP_port = 3360
EthernetClient client;


//OneWire stuff
#define OneWire_data_pin 11
OneWire oneWire(OneWire_data_pin);
DallasTemperature sensors(&oneWire);


//JSON stuff
DynamicJsonDocument doc(1024);
String rawMessage, serializedMessage;

struct JsonPart {  //define structure of the jsonParts array elements
  String key;
  String value;
};

JsonPart jsonParts[10];

uint8_t JsonElements;


//Modbus stuff

// Approved Modbus function codes
const uint8_t ApprovedModbusCommands[] = {
  0x01,  // Read Coils
  0x03,  // Read Holding Registers
  0x05,  // Write Single Coil
  0x06,  // Write Single Register
  0x0F,  // Write Multiple Coils
  0x10,  // Write Multiple Registers
  0x0B,  // Get Comm Event Counter
  0x11   // Report Slave ID
};

// Approved Modbus register addresses
const uint16_t ApprovedModbusRegisterAddresses[] = {
  0x00,  // PRESET_REFERENCE_LSB
  0x01,  // PRESET_REFERENCE_MSB
  0x02,  // DC_BRAKE
  0x03,  // COASTING
  0x04,  // QUICK_STOP
  0x05,  // FREEZE_FREQ
  0x06,  // MOTOR_STATE
  0x07,  // RESET
  0x08,  // JOG
  0x09,  // RAMP
  0x0A,  // DATA
  0x0B,  // RELAY1
  0x0C,  // RELAY2
  0x0D,  // SET_UP_LSB
  0x0F,  // REVERSING
  0x40   // WRITE_TO_RAM
};


modbus_t ModbusQuery;
uint16_t ModbusSlaveRegisters[64];

// This MACRO defines number of the comport that is used for RS 485 interface.
// For MAXI and MEGA RS485 is reserved UART Serial3.
#define RS485Serial 3
#define MasterModbusAdd 0
Modbus ControllinoModbusMaster(MasterModbusAdd, RS485Serial, 0);


//Setup funtion
void setup() {
  // initialize Serial, ethernet and oneWire busses for communication
  Serial.begin(115200);
  Ethernet.begin(MAC, ControllinoIP);
  sensors.begin();

  //initializes Modbus bus for communication and sets the timeout
  ControllinoModbusMaster.begin(19200);
  ControllinoModbusMaster.setTimeOut(5000);

  //setups the pins D0-D6 for output to toggle the heating realays
  for (int i = 2; i <= 8; i++) {
    pinMode(i, OUTPUT);
  }

  //setups the pin defined as(OneWire_data_pin) to input
  pinMode(OneWire_data_pin, INPUT);
}


//Loop funtion that sends modbusQueries and cheaks if there are incoming messages when idle
void loop() {
  static char buffer[256];
  static int index = 0;
  //cheaks if there is a new modbus query that have to be sent
  if (sendModbusQuerry == 1) {
    sendModbusQuerry = 0;

    ControllinoModbusMaster.query(ModbusQuery);  // Send Modbus query

    // Wait for a response
    uint8_t response = ControllinoModbusMaster.poll(ModbusSlaveRegisters, sizeof(ModbusSlaveRegisters) / sizeof(uint16_t));

    if (response == 0) {
      // Response received, send to serializeJSON function
      serializeModbusResponse(true, ModbusSlaveRegisters, ModbusQuery.u16CoilsNo);
    } else {
      // No response received, send error to serializeJSON function
      serializeModbusResponse(false, nullptr, 0);
    }
  }

  while (client.available()) {
    char c = client.read();
    if (c == '\n') {
      buffer[index] = '\0';  // Null-terminate the string
      rawMessage = String(buffer);
      Serial.println("Received: " + rawMessage);
      // Call your parseJSON function here
      parseJSON(rawMessage);
      index = 0;  // Reset index for the next message
    } else {
      if (index < 255) {  // Make sure to not overflow the buffer
        buffer[index++] = c;
      }
    }
  }

  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      buffer[index] = '\0';  // Null-terminate the string
      rawMessage = String(buffer);
      Serial.println("Received: " + rawMessage);
      // Call your parseJSON function here
      parseJSON(rawMessage);
      index = 0;  // Reset index for the next message
    } else {
      if (index < 255) {  // Make sure to not overflow the buffer
        buffer[index++] = c;
      }
    }
  }
}




//funtion that parses the incoming JSON message to jsonParts array that holds the key and value
void parseJSON(String rawMessage) {
  DeserializationError error = deserializeJson(doc, rawMessage);
  if (error) {
    // Send error to Raspberry Pi
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.f_str());
    return;
  }
  int i = 0;
  JsonObject obj = doc.as<JsonObject>();
  for (JsonPair kv : obj) {
    if (i >= 10) {
      // Send an error to Raspberry Pi (JSON message has too many parts)
      break;
    }
    jsonParts[i].key = kv.key().c_str();
    jsonParts[i].value = kv.value().as<String>();
    i++;
  }

  commandComparison(jsonParts);
}


//funtion that sends the JSON message via ethernet and then returns to loop
void sendJSON(String serializedMessage) {
  Serial.println(serializedMessage);
}



//
void serializeModbusResponse(bool success, uint16_t* registers, uint16_t length) {
  DynamicJsonDocument doc(256);                 // Create a new JSON document to hold the output
  JsonObject outputObj = doc.to<JsonObject>();  // Create a JSON object for the output

  if (success) {
    outputObj["status"] = "success";
    outputObj["id"] = ModbusQuery.u8id;
    outputObj["fct"] = ModbusQuery.u8fct;
    outputObj["RegAdd"] = ModbusQuery.u16RegAdd;
    outputObj["CoilsNo"] = ModbusQuery.u16CoilsNo;
    outputObj["reg"] = *ModbusQuery.au16reg;
  } else {
    outputObj["status"] = "failure";
    outputObj["error"] = "Timeout or other error description";
  }

  // Serialize the JSON object to a string
  String serializedMessage;
  serializeJson(outputObj, serializedMessage);

  // Send the JSON string as needed (e.g., via Ethernet, Serial, etc.)
  sendJSON(serializedMessage);
}



//
void commandComparison(JsonPart jsonParts[]) {
  String key0 = jsonParts[0].key;
  int value0 = jsonParts[0].value.toInt();
  if (key0 != "command") {
    // Send an error message (wrong JSON structure)
  } else {
    switch (value0) {
      case statusCheck:
        // ...
        break;
      case sendModbusCommand:           // Corrected case label
        CreateModbusQuerry(jsonParts);  // Corrected function call
        break;
      case getTempeture:
        getTemperature();
        break;
      case setHeating:
        toggleHeating(jsonParts);
        break;
      default:
        // Send an error message that the command is not understood
        break;
    }
  }
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
  sendJSON(serializedMessage);
}


void CreateModbusQuerry(JsonPart jsonParts[]) {
  // Convert the received values from hex string to integer
  uint8_t u8id = strtol(jsonParts[1].value.c_str(), NULL, 16);
  uint8_t u8fct = strtol(jsonParts[2].value.c_str(), NULL, 16);
  uint16_t u16RegAdd = strtol(jsonParts[3].value.c_str(), NULL, 16);
  uint16_t u16CoilsNo = strtol(jsonParts[4].value.c_str(), NULL, 16);
  uint16_t au16reg = strtol(jsonParts[5].value.c_str(), NULL, 16);

  // // Check if the function code is in the list of approved commands
  // bool fctIsValid = false;
  // for (uint8_t i = 0; i < sizeof(ApprovedModbusCommands) / sizeof(ApprovedModbusCommands[0]); ++i) {
  //   if (u8fct == ApprovedModbusCommands[i]) {
  //     fctIsValid = true;
  //     break;
  //   }
  // }

  // // Check if the register address is in the list of approved addresses
  // bool regAddIsValid = false;
  // for (uint16_t i = 0; i < sizeof(ApprovedModbusRegisterAddresses) / sizeof(ApprovedModbusRegisterAddresses[0]); ++i) {
  //   if (u16RegAdd == ApprovedModbusRegisterAddresses[i]) {
  //     regAddIsValid = true;
  //     break;
  //   }
  // }

  // if (!fctIsValid || !regAddIsValid) {
  //   // Handle the error here, for example, by sending an error message
  //   return;
  // }

  // If everything is valid, proceed to set up the Modbus query
  ModbusQuery.u8id = u8id;
  ModbusQuery.u8fct = u8fct;
  ModbusQuery.u16RegAdd = u16RegAdd;
  ModbusQuery.u16CoilsNo = u16CoilsNo;
  ModbusQuery.au16reg = &au16reg;

  sendModbusQuerry = 1;
}


//
void toggleHeating(JsonPart jsonParts[]) {
  DynamicJsonDocument doc(256);                 // Create a new JSON document to hold the output
  JsonObject outputObj = doc.to<JsonObject>();  // Create a JSON object for the output

  // Check if the keys index 1 - 7 are not D0 - D6; if they are not, send an error
  if (jsonParts[1].key != "D0" || jsonParts[2].key != "D1" || jsonParts[3].key != "D2" || jsonParts[4].key != "D3" || jsonParts[5].key != "D4" || jsonParts[6].key != "D5" || jsonParts[7].key != "D6") {
    // Send an error that the values of the jsonParts keys 1 - 7 are not the digital pin values
    
  } else {
    // Loop through jsonParts from index 1 to 7
    for (int i = 1; i <= 7; i++) {
      // Convert the jsonPart index to corresponding pinNumbers (offset of +1)
      int pinNumber = i + 1;

      // Check the value and set the pin accordingly
      if (jsonParts[i].value == "high") {
        digitalWrite(pinNumber, HIGH);
        outputObj[jsonParts[i].key] = "high";  // Add to the output JSON object
      } else if (jsonParts[i].value == "low") {
        digitalWrite(pinNumber, LOW);
        outputObj[jsonParts[i].key] = "low";  // Add to the output JSON object
      } else {
        // Handle invalid value (neither "high" nor "low")
        outputObj[jsonParts[i].key] = "wrong state";  // Add to the output JSON object
      }
    }

    serializedMessage = "";
    serializeJson(outputObj, serializedMessage);
  }

  // Send the JSON string using your sendJSON function
  sendJSON(serializedMessage);
}


//cheaks that the json message contains the parts to create the Modbus querry
bool modbusJsonKeyValidation(JsonPart jsonParts[]) {
  if (jsonParts[1].key != "id" || jsonParts[2].key != "fct" || jsonParts[3].key != "RegAdd" || jsonParts[4].key != "CoilsNo" || jsonParts[5].key != "reg") {
    return false;
  }
  return true;
}


//This end of the code includes the list of most defenations that the code might need for furure use or error cheaking but for now they are not used


/*enum ModbusSlaveAddresses {
  TJ1 = 1,
  TJ2 = 2,
  TJ3 = 3,
  TJ4 = 4,
  TJ5 = 5,
  TJ6 = 6,
};



enum ModbusRegisterAddresses {
  //This list has most of the coil/register addresses for modbus communication the unused defenations are commented out to save memory space

  //FC control words
  PRESET_REFERENCE_LSB = 0x00,
  PRESET_REFERENCE_MSB = 0X01,
  DC_BRAKE = 0X02,
  COASTING = 0X03,
  QUICK_STOP = 0X04,
  FREEZE_FREQ = 0X05,
  MOTOR_STATE = 0X06,
  RESET = 0X07,
  JOG = 0X08,
  RAMP = 0X09,
  DATA = 0X0A,
  RELAY1 = 0X0B,
  RELAY2 = 0X0C,
  SET_UP_LSB = 0X0D,
  REVERSING = 0X0F,
  WRITE_TO_RAM = 0x40,

  //FC status words
  CTRL_READY = 0X20,
  FC_READY = 0X21,
  COASTING_STATUS = 0X22,
  ALARM = 0X23,
  WARNING = 0X27,
  REFRENCE = 0X28,
  CTRL_MODE = 0X29,
  FREQ_RANGE = 0X2A,
  RUNNING = 0X2B,
  VOLT_WARNING = 0X2D,
  CURRENT_LIMIT = 0X2E,
  THERMAL_WARNING = 0X2F,

  //FC register addresses

};

enum ModbusCommands {
  Read_coils = 0x01,
  Read_holding_registers = 0x03,
  Write_single_coil = 0x05,
  Write_single_register = 0x06,
  Write_multiple_coils = 0x0F,
  Write_multiple_registers = 0x10,
  Get_comm_event_counter = 0x0B,
  Reoirt_slave_ID = 0x11,
};*/
