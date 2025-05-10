#include <Arduino.h>
#include <Dynamixel2Arduino.h> // .h header file
#include <motorControl.h>

#define DXL_DIR_PIN 4
#define DXL_SERIAL Serial2
const long DXL_BAUDRATE = 57600;

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);  // Instantiate dxl

motorControl* motor;  // Declare motor pointer

void setup() {

  Serial.begin(115200);
  while(!Serial); // Wait for Serial Monitor
  delay(2000);
  // Initialize Serial2 explicitly
  DXL_SERIAL.begin(DXL_BAUDRATE);
  motor = new motorControl(dxl);
  pinMode(4, OUTPUT);

u_int8_t motorID = 3;

  if(dxl.ping(motorID)) {
    Serial.println("Motor found!");
} else {
    Serial.println("Motor not responding.");
}




}

void loop() {
  // Continuously read position
  u_int8_t motorID = 3;
  if(dxl.ping(motorID)) {
    int32_t position = dxl.getPresentPosition(motorID);
    Serial.print("Current Position: ");
    Serial.println(position);
  } else {
    Serial.println("Lost connection to motor!");
  }
  delay(1000);
}


// #include <Arduino.h>
// #include <Dynamixel2Arduino.h>

// // Configuration
// #define DXL_DIR_PIN 4         // Direction control pin
// #define DXL_BAUDRATE 1000000  // 1Mbps baud rate
// #define DXL_PROTOCOL_VERSION 2.0
// #define DXL_ID 3              // Target motor ID

// // Initialize Dynamixel communication
// Dynamixel2Arduino dxl(Serial2, DXL_DIR_PIN);

// void setup() {
//   // Serial monitor for debugging
//   Serial.begin(115200);
//   while(!Serial); // Wait for serial connection

//   // Initialize Dynamixel communication
//   dxl.begin(DXL_BAUDRATE);
//   dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  
//   Serial.println("\nStarting Dynamixel Connection Test...");
  
//   // Try to ping the motor
//   Serial.print("Pinging motor ID ");
//   Serial.println(DXL_ID);
  
//   if(dxl.ping(DXL_ID)) {
//     Serial.println("Success! Motor connected.");
    
//     // Read and display model number
//     uint16_t model_number = dxl.getModelNumber(DXL_ID);
//     Serial.print("Model Number: 0x");
//     Serial.println(model_number, HEX);
    
//     // Read current position
//     int32_t position = dxl.getPresentPosition(DXL_ID);
//     Serial.print("Current Position: ");
//     Serial.println(position);
//   } else {
//     Serial.println("Error: Motor not found!");
//   }
// }

// void loop() {
//   // Continuously read position
//   if(dxl.ping(DXL_ID)) {
//     int32_t position = dxl.getPresentPosition(DXL_ID);
//     Serial.print("Current Position: ");
//     Serial.println(position);
//   } else {
//     Serial.println("Lost connection to motor!");
//   }
//   delay(1000);
// }