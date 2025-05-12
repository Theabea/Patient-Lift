#include <Arduino.h>
#include <Dynamixel2Arduino.h>
#include <motorControl.h>

#define DXL_DIR_PIN 4
#define DXL_SERIAL Serial2
const long DXL_BAUDRATE = 57600;
const uint8_t motorID = 3;

const int16_t PWM_ASSIST = 40;          // Assist torque
const int16_t POSITION_MIN = 990;
const int16_t POSITION_MAX = 1535;
const uint16_t MOTION_THRESHOLD = 10;    // Minimum Δposition to count as movement
const uint16_t NO_MOTION_COUNT_MAX = 10;

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);
motorControl* motor;

int32_t lastPosition = 0;
int noMotionCounter = 0;

void setup() {
  Serial.begin(115200);
  while(!Serial); // Wait for Serial Monitor
  delay(2000);

  // Initialize Dynamixel communication
  DXL_SERIAL.begin(DXL_BAUDRATE);
  motor = new motorControl(dxl);
  pinMode(DXL_DIR_PIN, OUTPUT);

  dxl.setPortProtocolVersion(2.0);

  if(dxl.ping(motorID)) {
    Serial.println("Motor found!");
  } else {
    Serial.println("Motor not responding.");
    return;
  }

  // Set motor to PWM control mode
  dxl.writeControlTableItem(ControlTableItem::OPERATING_MODE, motorID, 16);
  delay(100);

  // Enable torque
  dxl.writeControlTableItem(ControlTableItem::TORQUE_ENABLE, motorID, 1);
  delay(100);

  lastPosition = dxl.getPresentPosition(motorID);
  Serial.print("Initial Position: ");
  Serial.println(lastPosition);
}

void loop() {
  int32_t currentPosition = dxl.getPresentPosition(motorID);
  int32_t delta = currentPosition - lastPosition;

  // Handle soft limit enforcement
  if (currentPosition < POSITION_MIN) {
    // Below min → apply strong force forward
    dxl.writeControlTableItem(ControlTableItem::TORQUE_ENABLE, motorID, 1);
    dxl.writeControlTableItem(ControlTableItem::GOAL_PWM, motorID, +100); // Adjust strength as needed
    Serial.println("Below range: resisting downward push.");
  }
  else if (currentPosition > POSITION_MAX) {
    // Above max → apply strong force backward
    dxl.writeControlTableItem(ControlTableItem::TORQUE_ENABLE, motorID, 1);
    dxl.writeControlTableItem(ControlTableItem::GOAL_PWM, motorID, -100); // Adjust strength as needed
    Serial.println("Above range: resisting upward push.");
  }
  else {
    // Inside valid assist range
    if (abs(delta) > MOTION_THRESHOLD) {
      // Assist in direction of motion
      int16_t pwmValue = (delta > 0) ? PWM_ASSIST : -PWM_ASSIST;
      dxl.writeControlTableItem(ControlTableItem::TORQUE_ENABLE, motorID, 1);
      dxl.writeControlTableItem(ControlTableItem::GOAL_PWM, motorID, pwmValue);
      noMotionCounter = 0;
    } else {
      noMotionCounter++;
      if (noMotionCounter >= 10) {
        dxl.writeControlTableItem(ControlTableItem::GOAL_PWM, motorID, 0);
        dxl.writeControlTableItem(ControlTableItem::TORQUE_ENABLE, motorID, 0);
      }
    }
  }

  lastPosition = currentPosition;
  delay(10);
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