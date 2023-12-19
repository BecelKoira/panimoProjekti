#include <Controllino.h> /* Usage of CONTROLLINO library allows you to use CONTROLLINO_xx aliases in your sketch. */
#include "ModbusRtu.h"   /* Usage of ModBusRtu library allows you to implement the Modbus RTU protocol in your sketch. */

// This MACRO defines Modbus master address.
// For any Modbus slave devices are reserved addresses in the range from 1 to 247.
// Important note only address 0 is reserved for a Modbus master device!

#define MasterModbusAdd 0
#define SlaveModbusAdd 1

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

#define Motor_speed 50.0f

// Degine named constants for register addresses
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



// We preset some default values
// ModbusSlaveRegisters[SPEED_SETPOINT] = 0xFFFF;
// Serial.print("SPEED_SETPOINT set to: ");
// Serial.println(ModbusSlaveRegisters[ModbusQuery[2].u16RegAdd]);

// This is a structure which contains a query to a slave device
modbus_t ModbusQuery[10];

bool motorState = 0;  // machine state
bool coastingState = 0;

unsigned long WaitingTime;

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

void configureModbusQuery(int queryNumber, int slaveAddress, int functionCode, int startAddress, int coilNumber, int arrayPointer){

  ModbusQuery[queryNumber].u8id = slaveAddress;                      // slave address
  ModbusQuery[queryNumber].u8fct = functionCode;                         // function code
  ModbusQuery[queryNumber].u16RegAdd = startAddress;                       // start address in slave
  ModbusQuery[queryNumber].u16CoilsNo = coilNumber;                             // number of elements (coils or registers) to read
  ModbusQuery[queryNumber].au16reg = &ModbusSlaveRegisters[arrayPointer];  // pointer to a memory array in the CONTROLLINO

}

void sendQuery(int queryNumber, unsigned long RESPONSE_TIMEOUT_MS) {

  if (queryNumber == 4) {
    if (motorState == 0) {
      ModbusSlaveRegisters[SET_MOTOR_STATE] = 1;
      motorState = 1;
    } else {
      ModbusSlaveRegisters[SET_MOTOR_STATE] = 0;
      motorState = 0;
    }
  }

  if (queryNumber == 5) {
    if (coastingState == 0) {
      ModbusSlaveRegisters[COASTING] = 1;
      coastingState = 1;
    } else {
      ModbusSlaveRegisters[COASTING] = 0;
      coastingState = 0;
    }
  }

  Serial.print("Before sending query, register shows: ");
  Serial.println(ModbusSlaveRegisters[SPEED_SETPOINT]);

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

      Serial.print("After sending query, register shows: ");
      Serial.println(ModbusSlaveRegisters[SPEED_SETPOINT]);

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




void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  Serial.println("-----------------------------------------");
  Serial.println("CONTROLLINO Modbus RTU Master Test Sketch");
  Serial.println("-----------------------------------------");
  Serial.println("");

  //Initialize the default values for the array
  ModbusSlaveRegisters[SPEED_SETPOINT] = 0x0FFF;  // initial Speed value

  configureModbusQuery(0, SlaveModbusAdd, Read_coils, FC_READY, 1, FC_READY);
  configureModbusQuery(1, SlaveModbusAdd, Read_coils, CONTROL_READY, 1, CONTROL_READY);
  configureModbusQuery(2, SlaveModbusAdd, Write_single_register, 0xc359, 1, SPEED_SETPOINT);
  configureModbusQuery(3, SlaveModbusAdd, Read_coils, SPEED_SETPOINT, 16, SPEED_SETPOINT);
  configureModbusQuery(4, SlaveModbusAdd, Write_single_coil, SET_MOTOR_STATE, 1, SET_MOTOR_STATE);
  configureModbusQuery(5, SlaveModbusAdd, Write_single_coil, COASTING, 1, COASTING);
/*
  // ModbusQuery 0: read registers (Read state of coil 34)
  ModbusQuery[0].u8id = SlaveModbusAdd;                      // slave address
  ModbusQuery[0].u8fct = Read_coils;                         // function code
  ModbusQuery[0].u16RegAdd = FC_READY;                       // start address in slave
  ModbusQuery[0].u16CoilsNo = 1;                             // number of elements (coils or registers) to read
  ModbusQuery[0].au16reg = &ModbusSlaveRegisters[FC_READY];  // pointer to a memory array in the CONTROLLINO

  //ModbusQuery 1; read registers (Read state of coil 33)
  ModbusQuery[1].u8id = SlaveModbusAdd;                           // slave address
  ModbusQuery[1].u8fct = Read_coils;                              // function code
  ModbusQuery[1].u16RegAdd = CONTROL_READY;                       // start address in slave
  ModbusQuery[1].u16CoilsNo = 1;                                  // number of elements (coils or registers) to read
  ModbusQuery[1].au16reg = &ModbusSlaveRegisters[CONTROL_READY];  // pointer to a memory array in the CONTROLLINO


  //ModbusQuery 2: Set motor speed
  ModbusQuery[2].u8id = SlaveModbusAdd;                            // slave address
  ModbusQuery[2].u8fct = Write_single_register;                    // function code (this one is write a single register)
  ModbusQuery[2].u16RegAdd = 0xc359;                               // start address in slave
  ModbusQuery[2].u16CoilsNo = 1;                                   // number of elements (coils or registers) to write
  ModbusQuery[2].au16reg = &ModbusSlaveRegisters[SPEED_SETPOINT];  // pointer to a memory array in the CONTROLLINO


  //ModbusQuery 4: Read motor speed
  ModbusQuery[3].u8id = SlaveModbusAdd;                            // slave address
  ModbusQuery[3].u8fct = Read_coils;                               // function code (this one is write a single register)
  ModbusQuery[3].u16RegAdd = SPEED_SETPOINT;                       // start address in slave
  ModbusQuery[3].u16CoilsNo = 16;                                  // number of elements (coils or registers) to write
  ModbusQuery[3].au16reg = &ModbusSlaveRegisters[SPEED_SETPOINT];  // pointer to a memory array in the CONTROLLINO

  //ModbusQuery 3: Set motor state
  ModbusQuery[4].u8id = SlaveModbusAdd;                             // slave address
  ModbusQuery[4].u8fct = Write_single_coil;                         // function code (this one is write a single register)
  ModbusQuery[4].u16RegAdd = SET_MOTOR_STATE;                       // start address in slave
  ModbusQuery[4].u16CoilsNo = 1;                                    // number of elements (coils or registers) to write
  ModbusQuery[4].au16reg = &ModbusSlaveRegisters[SET_MOTOR_STATE];  // pointer to a memory array in the CONTROLLINO

  //ModbusQuery 4: Read motor speed
  ModbusQuery[5].u8id = SlaveModbusAdd;                      // slave address
  ModbusQuery[5].u8fct = Write_single_coil;                  // function code (this one is write a single register)
  ModbusQuery[5].u16RegAdd = COASTING;                       // start address in slave
  ModbusQuery[5].u16CoilsNo = 1;                             // number of elements (coils or registers) to write
  ModbusQuery[5].au16reg = &ModbusSlaveRegisters[COASTING];  // pointer to a memory array in the CONTROLLINO
*/






  Modbus.begin(19200);      // baud-rate at 19200
  Modbus.setTimeOut(5000);  // if there is no answer in 5000 ms, roll over

  WaitingTime = 2000;
}

void loop() {
  delay(1000);
  if (Serial.available()) {
    int query = Serial.parseInt();
    sendQuery(query, WaitingTime);
  }
}
