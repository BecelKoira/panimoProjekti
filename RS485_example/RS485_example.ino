#include <Controllino.h> /* Usage of CONTROLLINO library allows you to use CONTROLLINO_xx aliases in your sketch. */
#include "ModbusRtu.h" /* Usage of ModBusRtu library allows you to implement the Modbus RTU protocol in your sketch. */

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


// The object ControllinoModbuSlave of the class Modbus is initialized with three parameters.
// The first parameter specifies the address of the Modbus slave device.
// The second parameter specifies type of the interface used for communication between devices - in this sketch - RS485.
// The third parameter can be any number. During the initialization of the object this parameter has no effect.
Modbus ControllinoModbusMaster(MasterModbusAdd, RS485Serial, 0);

// This uint16 array specified internal registers in the Modbus slave device.
// Each Modbus device has particular internal registers that are available for the Modbus master.
// check the frequency driver datasheet for a reference on the registers. We have 65 available registers to work with.
// All addresses are referenced to zero, so whichever register you want it's equivalent value would be address -1 in HEX format. ie. register 1 is addressed as 0x00 and register 65 is addressed as 0x40 (64 in HEX)
uint16_t ModbusSlaveRegisters[64]
// This is a structure which contains a query to a slave device
modbus_t ModbusQuery[2];

uint8_t myState; // machine state
uint8_t currentQuery; // pointer to message query

unsigned long WaitingTime;

void setup() {
 // initialize serial communication at 9600 bits per second:
 Serial.begin(9600);
 Serial.println("-----------------------------------------");
 Serial.println("CONTROLLINO Modbus RTU Master Test Sketch");
 Serial.println("-----------------------------------------");
 Serial.println("");
 // ModbusQuery 0: read registers (Read state of coil 65)
 ModbusQuery[0].u8id = SlaveModbusAdd; // slave address
 ModbusQuery[0].u8fct = Read_coils; // function code
 ModbusQuery[0].u16RegAdd = 0x29; // start address in slave
 ModbusQuery[0].u16CoilsNo = 1; // number of elements (coils or registers) to read
 ModbusQuery[0].au16reg = ModbusSlaveRegisters+49; // pointer to a memory array in the CONTROLLINO

 // ModbusQuery 1: write a single register
 /*ModbusQuery[1].u8id = SlaveModbusAdd; // slave address
 ModbusQuery[1].u8fct = Write_single_coil; // function code (this one is write a single register)
 ModbusQuery[1].u16RegAdd = 0x40; // start address in slave
 ModbusQuery[1].u16CoilsNo = 0; // number of elements (coils or registers) to write
 ModbusQuery[1].au16reg = ModbusSlaveRegisters+64; // pointer to a memory array in the CONTROLLINO
 
 ModbusSlaveRegisters[64] = 1; // initial value for the relays */
 
 ControllinoModbusMaster.begin( 9600 ); // baud-rate at 19200
 ControllinoModbusMaster.setTimeOut( 5000 ); // if there is no answer in 5000 ms, roll over
 
 WaitingTime = millis() + 1000;
 myState = 0;
 currentQuery = 0; 
}

void loop() {
 switch( myState ) {
 case 0: 
 if (millis() > WaitingTime) myState++; // wait state
 break;
 case 1: 
 Serial.println(ModbusSlaveRegisters[33]);
 Serial.print("---- Sending query ");
 Serial.print("0");
 Serial.println(" -------------");
 ControllinoModbusMaster.query( ModbusQuery[0] ); // send query (only once)
 myState++;
 currentQuery++;
 if (currentQuery == 2) 
 {
 currentQuery = 0;
 }
 break;
 case 2:
 ControllinoModbusMaster.poll(); // check incoming messages
 if (ControllinoModbusMaster.getState() == COM_IDLE) 
 {
 // response from the slave was received
 myState = 0;
 WaitingTime = millis() + 1000; 
 // debug printout
 if (currentQuery == 0)
 {
 // registers write was proceed
 Serial.println("---------- WRITE RESPONSE RECEIVED ----");
 Serial.println("");
 }
 /*if (currentQuery == 1)
 {
 // registers read was proceed
 Serial.println("---------- READ RESPONSE RECEIVED ----");
 Serial.print("Slave ");
 Serial.print(SlaveModbusAdd, DEC); 
 Serial.print(" ADC0: 0x");
 Serial.print(ModbusSlaveRegisters[0], HEX);
 Serial.print(" , Digital0: ");
 Serial.print(ModbusSlaveRegisters[1], BIN);
 Serial.print(" , ModbusCounterIn: ");
 Serial.print(ModbusSlaveRegisters[2], DEC);
 Serial.print(" , ModbusCounterOut: ");
 Serial.println(ModbusSlaveRegisters[3], DEC);
 Serial.println("-------------------------------------");
 Serial.println("");

 // toggle with the relays
 ModbusQuery[1].u16RegAdd++;
 if (ModbusQuery[1].u16RegAdd == 8)
 {
 ModbusQuery[1].u16RegAdd = 4;
 if (ModbusSlaveRegisters[4]) 
 {
 ModbusSlaveRegisters[4] = 0;
 }
 else 
 {
 ModbusSlaveRegisters[4] = 1;
 }
 }
 }
 }
 break;
 }*/
}
}
}