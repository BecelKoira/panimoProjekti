#include <Controllino.h> /* Usage of CONTROLLINO library allows you to use CONTROLLINO_xx aliases in your sketch. */
#include <Ethernet.h>
#include <ArduinoJson.h>
#include "ModbusRtu.h"   /* Usage of ModBusRtu library allows you to implement the Modbus RTU protocol in your sketch. */

// This MACRO defines Modbus master address.
// For any Modbus slave devices are reserved addresses in the range from 1 to 247.
// Important note only address 0 is reserved for a Modbus master device!

#define MasterModbusAdd 0
//#define SlaveModbusAdd 1

// This MACRO defines number of the comport that is used for RS 485 interface.
// For MAXI and MEGA RS485 is reserved UART Serial3.
#define RS485Serial 3

//Here we define the available functions to use with the frequency converter
#define Read_coils 0x01
#define Read_holding_registers 0x03
#define Write_single_coil 0x05
#define Write_single_register 0x06
#define Write_multiple_coils 0x0F
#define Write_multiple_registers 0x10
#define Get_comm_event_counter 0x0B
#define Report_slave_ID 0x11

// Pin configuration for heatingElements
const int heatingElementPins[] = {2, 3, 4, 5, 6, 7};
bool heatingElementStates[] = {false, false, false, false, false, false};
int dutyCycles[] = {0, 0, 0, 0, 0, 0};
const int numElements = sizeof(heatingElementPins) / sizeof(heatingElementPins[0]);

// On and off times fir each heating element (in milliseconds)
unsigned long onTimes[] = {0, 0, 0, 0};
unsigned long offTimes[] = {5000, 5000, 5000, 5000};

byte RaspiIP[] = { 10, 65, 32, 140 };
//byte ControllinoMaxiIP[] = { 10, 65, 32, 141 };
byte ControllinoMegaIP[] = { 10, 65, 32, 142 };
byte MAC[] = { 0x50, 0xD7, 0x53, 0x00, 0xC8, 0xB4 };
#define TCP_port  3360

EthernetServer server(TCP_port);
EthernetClient client;

#define OwnIP ControllinoMegaIP
String jsonString;

int motor_speed[5];
int slave_id;
int query_number;

DynamicJsonDocument doc(1024);


//JSON variables
int queryNumber;
uint16_t motorSpeed;
int slaveId;
int motorState;

bool coastingState = 0;
bool waitForResponse = false;

unsigned long WaitingTime;







// Define named constants for register addresses
enum ModbusRegisterAddresses {
  //FC control words.
  PRESET_REFERENCE_LSB = 0x00,  // Coil 1
  PRESET_REFERENCE_MSB = 0x01,  // Coil 2
  DC_BRAKE = 0x02,              // coil 3
  COASTING = 0x03,              // coil 4
  QUICK_STOP = 0x04,            // coil 5
  FREEZE_FREQ = 0x05,           // coil 6
  SET_MOTOR_STATE = 0x06,       // coil 7
  RESET = 0x07,                 // coil 8
  JOG = 0x08,                   // coil 9
  RAMP = 0x09,                  // coil 10
  DATA = 0x0A,                  // coil 11
  RELAY1 = 0x0B,                // coil 12
  RELAY2 = 0x0C,                // coil 13
  SETUP_LSB = 0x0D,             // coil 14
  REVERSING = 0x0F,             // coil 16

  // Speed control
  SPEED_SETPOINT = 0x10,

  //FC status words
  CONTROL_READY = 0x20,    // coil 33
  FC_READY = 0x21,         // coil 34
  COASTING_STATUS = 0x22,  // coil 35
  ALARM = 0x23,            // coil 36
  WARNING = 0x27,          // coil 40
  REFERENCE = 0x28,        // coil 41
  AUTO_MODE = 0x29,        // coil 42
  FREQ_RANGE = 0x2A,       // coil 43
  RUNNING = 0x2C,          // coil 44
  VOLT_WARNING = 0x2D,     // coil 46
  CURRENT_LIMIT = 0x2E,    // coil 47
  THERMAL_WARNING = 0x2F,  // coil 48

  // add registers as needed

};




// The object ControllinoModbuSlave of the class Modbus is initialized with three parameters.
// The first parameter specifies the address of the Modbus slave device.
// The second parameter specifies type of the interface used for communication between devices - in this sketch - RS485.
// The third parameter can be any number. During the initialization of the object this parameter has no effect.
Modbus Modbus(MasterModbusAdd, RS485Serial, 0);

// This uint16 array specified internal registers in the Modbus slave device.
// Each Modbus device has particular internal registers that are available for the Modbus master.
// check the frequency driver datasheet for a reference on the registers. We have 65 available registers to work with.
// All addresses are referenced to zero, so whichever register you want it's equivalent value would be address -1 in HEX format. ie. register 1 is addressed as 0x00 and register 65 is addressed as 0x40 (64 in HEX)
uint16_t ModbusSlaveRegisters[64] = { 0 };
uint16_t Motor_speed[5] = { 0 };



// We preset some default values
//ModbusSlaveRegisters[SPEED_SETPOINT] = 0xFFFF;

// Serial.print("SPEED_SETPOINT set to: ");
// Serial.println(ModbusSlaveRegisters[ModbusQuery[2].u16RegAdd]);

// This is a structure which contains a query to a slave device
modbus_t ModbusQuery[64];



// This function is used to transform the speed value from a percentage to it's corresponding hex value in the range of 0x000 to0xFFFF
uint16_t setMotorSpeed(float speedPercentage) {
  // Ensure the speed is no higuer than 200% or lower than -200%
  if (speedPercentage < -200.0f) {
    speedPercentage = -200.0f;
  } else if (speedPercentage > 200.0f) {
    speedPercentage = 200.0f;
  }

  // Map speedPercentage to the range -1.0 to 1.0
  float normalizedSpeed = speedPercentage / 200.0f;

  // Convert normalized speed to 16-bit range (0x0000 to 0xFFFF)
  uint16_t value = (uint16_t)((normalizedSpeed + 1.0f) * 0xFFFF / 2.0f);

  return value;

  // // Store the binary representation in the array
  // for (int i = 0; i < 16; i++) {
  //   int bit = (value >> (15 - i)) & 1;
  //   ModbusSlaveRegisters[16 + i] = bit;
  // }

  // for (int i = 16; i < 32; i++) {
  //   Serial.print(ModbusSlaveRegisters[i]);
  // }
}

void configureModbusQuery(int slaveAddress, int queryNum){
  switch (queryNum){
    case 0:
      // ModbusQuery 0: read registers (Read state of coil 34)
      ModbusQuery[0].u8id = slaveAddress;                      // slave address
      ModbusQuery[0].u8fct = Read_coils;                         // function code
      ModbusQuery[0].u16RegAdd = FC_READY;                       // start address in slave
      ModbusQuery[0].u16CoilsNo = 1;                             // number of elements (coils or registers) to read
      ModbusQuery[0].au16reg = &ModbusSlaveRegisters[FC_READY];  // pointer to a memory array in the CONTROLLINO
      break;

    case 1:
      //ModbusQuery 1; read registers (Read state of coil 33)
      ModbusQuery[1].u8id = slaveAddress;                           // slave address
      ModbusQuery[1].u8fct = Read_coils;                              // function code
      ModbusQuery[1].u16RegAdd = CONTROL_READY;                       // start address in slave
      ModbusQuery[1].u16CoilsNo = 1;                                  // number of elements (coils or registers) to read
      ModbusQuery[1].au16reg = &ModbusSlaveRegisters[CONTROL_READY];  // pointer to a memory array in the CONTROLLINO
      break;

    case 2:
      //ModbusQuery 2: Set motor speed
      ModbusQuery[2].u8id = slaveAddress;                            // slave address
      ModbusQuery[2].u8fct = Write_single_register;                    // function code (this one is write a single register)
      ModbusQuery[2].u16RegAdd = 0xc359;                               // start address in slave
      ModbusQuery[2].u16CoilsNo = 1;                                   // number of elements (coils or registers) to write
      ModbusQuery[2].au16reg = &ModbusSlaveRegisters[SPEED_SETPOINT];  // pointer to a memory array in the CONTROLLINO
      break;

    case 3:
      //ModbusQuery 3: Read motor speed
      ModbusQuery[3].u8id = slaveAddress;                            // slave address
      ModbusQuery[3].u8fct = Read_coils;                               // function code
      ModbusQuery[3].u16RegAdd = SPEED_SETPOINT;                       // start address in slave
      ModbusQuery[3].u16CoilsNo = 16;                                  // number of elements (coils or registers) to write
      ModbusQuery[3].au16reg = &ModbusSlaveRegisters[SPEED_SETPOINT];  // pointer to a memory array in the CONTROLLINO
      break;

    case 4:

      //ModbusQuery 4: Set motor state
      ModbusQuery[4].u8id = slaveAddress;                             // slave address
      ModbusQuery[4].u8fct = Write_single_coil;                         // function code (this one is write a single register)
      ModbusQuery[4].u16RegAdd = SET_MOTOR_STATE;                       // start address in slave
      ModbusQuery[4].u16CoilsNo = 1;                                    // number of elements (coils or registers) to write
      ModbusQuery[4].au16reg = &ModbusSlaveRegisters[SET_MOTOR_STATE];  // pointer to a memory array in the CONTROLLINO
      break;
    
    case 5:
      //ModbusQuery 4: Read motor speed
      ModbusQuery[5].u8id = slaveAddress;                      // slave address
      ModbusQuery[5].u8fct = Write_single_coil;                  // function code (this one is write a single register)
      ModbusQuery[5].u16RegAdd = COASTING;                       // start address in slave
      ModbusQuery[5].u16CoilsNo = 1;                             // number of elements (coils or registers) to write
      ModbusQuery[5].au16reg = &ModbusSlaveRegisters[COASTING];  // pointer to a memory array in the CONTROLLINO
      break;
      
  
  }
  

}


void sendQuery(int queryNumber, unsigned long RESPONSE_TIMEOUT_MS) {


  //Serial.print("Before sending query, register shows: ");
  //Serial.println(ModbusSlaveRegisters[SPEED_SETPOINT]);

  Serial.println("Sending query.");
  // Send the Modbus query
  Modbus.query(ModbusQuery[queryNumber]);

  // Wait for response with timeout
  unsigned long startTime = millis();
  while (millis() - startTime < RESPONSE_TIMEOUT_MS) {
    Modbus.poll();  // Poll for incoming messages
    if (Modbus.getState() == COM_IDLE) {
      // Response received, process data
      Serial.println("Response received");

      //Serial.print("After sending query, register shows: ");
      //Serial.println(ModbusSlaveRegisters[SPEED_SETPOINT]);

      // if(queryNumber==3){

      //   Serial.print("Speed Setting shows: ");
      //   for (int i = 16; i < 32; i++) {
      //     Serial.print(ModbusSlaveRegisters[i]);
      //   }

      //   Serial.println("");
      // }

      Serial.print("Error shows: ");
      Serial.println(Modbus.getLastError());
      // Process other data...
      break;
    }
  }

  // Check if response was received within the timeout
  if (millis() - startTime >= RESPONSE_TIMEOUT_MS) {
    // Handle timeout or error
    Serial.println("Timeout or error occurred");
  }
}
// This function handles receiving a JSON string from ethernet
String readIncomingJSON(EthernetClient& client){
  String jsonString = "";

  // check if data is available
  if (client.available()) {
    // Read data from the ethernet client and append it to the JSON string
    while (client.available()) {
      jsonString += static_cast<char>(client.read());
    }
  }

  Serial.print("Received JSON: ");
  Serial.println(jsonString);

  return jsonString;
}

// This function parses the JSON string into a JSON object
void parseJsonString(const String& jsonString, DynamicJsonDocument& doc){
  queryNumber = NULL;
  


  //Parse the JSOn string into the JSON document
  DeserializationError error = deserializeJson(doc, jsonString);

  //Check for parsing errors
  if (error) {
    Serial.print(F("Failed to parse JSON: "));
    Serial.println(error.c_str());
  }

  // Retrieve the values from the JSON Object
  JsonArray heatingElements = doc["heatingElements"].as<JsonArray>();

  

  // Loop through each heating element in the array
  for (int i = 0; i < heatingElements.size(); i++) {
    JsonObject heatingElement = heatingElements[i].as<JsonObject>();

    // Construct the key for isEnabled and dutyCycle
    String isEnabledKey = "heatingElement" + String(i + 1) + "_isEnabled";
    String dutyCycleKey = "heatingElement" + String(i + 1) + "_dutyCycle";

    // Extract isEnabled and dutyCycle and store in arrays
    if (heatingElement.containsKey(isEnabledKey) && heatingElement.containsKey(dutyCycleKey)) {
      heatingElementStates[i] = heatingElement[isEnabledKey];
      dutyCycles[i] = heatingElement[dutyCycleKey];
    } else {
      Serial.print("Value for heating element ");
      Serial.print(i + 1);
      Serial.println(" missing.");
      }
    }

  if (doc.containsKey("queryNumber")){queryNumber = doc["queryNumber"];}
  
  if (doc.containsKey("motorSpeed")){motorSpeed = doc["motorSpeed"];}
  
  if (doc.containsKey("motorState")){motorState = doc["motorState"];}
  
  if (doc.containsKey("slaveId")){slaveId = doc["slaveId"];}
  

}


void createExampleJSON(){
  // Create a JSON document
  const size_t capacity = JSON_OBJECT_SIZE(4);
  DynamicJsonDocument doc(capacity);

  // Add values to the JSON document
  JsonObject heating = doc.createNestedObject("heating");
  heating["onTime"] = 5000;
  heating["offTime"] = 10000;

  doc["slaveID"] = 1;
  doc["queryNumber"] = 2;
  doc["motorSpeed"] = 0x0FFF;

  // Serialize the JSON document to a string
  String jsonString;
  serializeJson(doc, jsonString);

  // Print the serialized JSON string
  Serial.println(jsonString);

} 

// Function to control heating elements based on duty cycle
void controlHeatingElement(int pin, int dutyCycle){

  static unsigned long previousMillis = 0;
  unsigned long currentMillis = millis();
  static bool heatingElementState = LOW;

  // Calculate the time for which the heating element should be ON based on dutyCycle
  unsigned long onTime = (5 * dutyCycle) / 100;
  unsigned long offTime = 5 - onTime;

  

  // Check if it's time to toggle the heating element
  if (currentMillis - previousMillis >= (heatingElementState ? onTime : offTime) * 1000) {
    // Toggle the heating element state
    heatingElementState = !heatingElementState;

    previousMillis = currentMillis;

    digitalWrite(pin, heatingElementState);

    // Print the current state to Serial
    Serial.print("Heating Element on Pin ");
    Serial.print(pin);
    Serial.print(" State: ");
    Serial.println(heatingElementState ? "ON" : "OFF");

    
  }


}



void reconnectClient(){
  static unsigned long lastConnectionTime = 0;
  static const unsigned long connectionInterval = 5000;

  if(!client.connected()) {
    unsigned long currentTime = millis();
    if (currentTime - lastConnectionTime >= connectionInterval){
      Serial.println("Attempting to reconnect...");
      client.stop();
      client.connect(ControllinoMegaIP, TCP_port);
    }
  }
}



void setup() {

  //Initialize heating element pins
  for (int i = 0; i < numElements; i++) {
    pinMode(heatingElementPins[i], OUTPUT);
  }


  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  Ethernet.begin(MAC, ControllinoMegaIP);
  server.begin();
  delay(2000);

  //Serial.println(F("Ethernet connection established."));

  Serial.println("-----------------------------------------");
  Serial.println("CONTROLLINO Modbus RTU Master Test Sketch");
  Serial.println("-----------------------------------------");
  Serial.println("");

  //Initialize the default values for the array
  //ModbusSlaveRegisters[SPEED_SETPOINT] = 0x0FFF;  // initial Speed value
  ModbusSlaveRegisters[COASTING] = 1;
  ModbusSlaveRegisters[SET_MOTOR_STATE] = 0;

  Modbus.begin(19200);      // baud-rate at 19200
  Modbus.setTimeOut(5000);  // if there is no answer in 5000 ms, roll over

  //use this to preset the values of all drives.
  WaitingTime = 2000;
  /*for(int slave = 1; slave < 6; slave++){
    configureModbusQuery(slave,  4);
    configureModbusQuery(slave,  5);
    sendQuery(4, WaitingTime);
    sendQuery(5, WaitingTime);
  }*/


}





void serialTest(){
   Serial.println("Input a number between 1-4 to choose action");
  Serial.println("1 - Set speed value");
  Serial.println("2 - Motor start/stop");
  Serial.println("3 - Heating duty cycle setup (Not in place yet)");
  Serial.println("4 - Heating start/stop (Not in place yet)");
  Serial.println("5 - Select direction");
  delay(1000);

  Serial.println(Serial.available());

  while(Serial.available() == 0){}

  int mode = Serial.parseInt();
  //serialDump = Serial.read();
  //Serial.flush();
  Serial.print("You selected ");
  Serial.println(mode);


  Serial.println("Select slave_id of FC you wish to control.");
  delay(1000);
  Serial.println(Serial.available());
  while(Serial.available() == 0){}

  int slave_id = Serial.parseInt();
  //serialDump = Serial.read();
  Serial.print("You selected ");
  Serial.println(slave_id);


  switch (mode){

    case 1:{
      
      Serial.println("Please give a speed value from 0 to 100%"); 
      delay(50);
      while (Serial.available() == 0){}
      int speed = Serial.parseInt();
      //serialDump = Serial.read();
      ModbusSlaveRegisters[SPEED_SETPOINT] = speed;
      //configureModbusQuery(3, slave_id, Write_multiple_coils, SPEED_SETPOINT, 16, SPEED_SETPOINT);
      sendQuery(2, WaitingTime);

      Serial.print("Speed value saved as ");
      Serial.print(ModbusSlaveRegisters[SPEED_SETPOINT]);
      Serial.print(", Hex: 0x");
      Serial.println(ModbusSlaveRegisters[SPEED_SETPOINT], HEX);
      //sendQuery(query, WaitingTime);
      break;
    }



    case 2:{
      Serial.println("select start or stop (0 or 1)");
      while(Serial.available() == 0){}
      int state = Serial.parseInt();
      //serialDump = Serial.read();

      ModbusSlaveRegisters[SET_MOTOR_STATE] = state;
      
      //configureModbusQuery(4, slave_id, Write_single_coil, SET_MOTOR_STATE, 1);
      sendQuery(4, WaitingTime);
      break;
    }

    case 3:{
      break;
    }

    case 4:{
      break;
    }

    case 5:{
      Serial.println("Select direction (0 or 1)");
      while(Serial.available() == 0){}
      int direction = Serial.parseInt();
      //serialDump = Serial.read();
      ModbusSlaveRegisters[REVERSING] = direction;

      //configureModbusQuery(6, slave_id, Write_single_coil, REVERSING, 1);
      break;
    }
      
    }



}

void loop() {
  // Check if there's a new client connected
  client = server.available();
  if (client) {
    Serial.println("New client connected.");

    //Read incoming JSON string
    String jsonString = readIncomingJSON(client);

    parseJsonString(jsonString, doc); 
    Serial.println(motorState);
    Serial.println(motorSpeed);

    if (motorState){
      ModbusSlaveRegisters[SET_MOTOR_STATE] = 1;
    }

    else{
      ModbusSlaveRegisters[SET_MOTOR_STATE] = 0;
    }
    
    ModbusSlaveRegisters[SPEED_SETPOINT] = motorSpeed;

    if(queryNumber != NULL){
      configureModbusQuery(slaveId, queryNumber);
      sendQuery(queryNumber, WaitingTime);
    }


    
    reconnectClient();

    

  }

  // Control each heating element individually
  for (int i = 0; i < numElements; i++) {
    if (heatingElementStates[i]){controlHeatingElement(heatingElementPins[i], dutyCycles[i]);}
    
    }



  
  /*else{
    client.stop();
    Serial.println("Client disconnected");
  }*/








}



