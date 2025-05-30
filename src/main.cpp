#include <Arduino.h>
#include <Dynamixel2Arduino.h>
#include <motorControl.h>
#include <cmath> 

#define DXL_DIR_PIN 4
#define DXL_SERIAL Serial2
const long DXL_BAUDRATE = 57600;
const uint8_t motorID = 3;

// Constants for MX-64 motor (from your calculatePWM function)
// C1MX64_STAND_STILL will be used as a base, as we are now calculating the torque.
const float C2MX64 = 105.3303;
const float C1MX64_STAND_STILL = 152.6855;

// Physical parameters of your lift arm
const float ARM_MASS_KG = 0.169; // 169 g converted to kg
const float ARM_CM_DISTANCE_M = 0.0936; // 9.36 cm converted to meters
const float GRAVITY_ACCELERATION = 9.81; // m/s^2

// Your defined operating range in Dynamixel position units
const int16_t POSITION_MIN = 990;
const int16_t POSITION_MAX = 1535;

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);
motorControl* motor;

// Function to calculate PWM from desired torque and angular velocity
int calculatePWM(float torque, float angularVel) {
  float C1_val = C1MX64_STAND_STILL; // This is the conversion factor from Nm to PWM

  int PWM_out = torque * C1_val + angularVel * C2MX64;

  // Constrain PWM to valid range for Dynamixel (typically +/- 885 for MX series)
  if (PWM_out > 885) {
    PWM_out = 885;
  } else if (PWM_out < -885) {
    PWM_out = -885;
  }

  return PWM_out;
}

float deg2rad(float degrees) {
  return degrees * M_PI / 180.0f;
}

float calculateAngle(int position) {
  // Define the known points
  float position0Degrees = 990.0;
  float angle0Degrees = 0.0;

  float position45Degrees = 2000.0;
  float angle45Degrees = 90.0;

  // Calculate the slope (degrees per position unit)
  float slope = (angle45Degrees - angle0Degrees) / (position45Degrees - position0Degrees);

  // Calculate the angle using the linear equation: angle = angle0 + slope * (position - position0)
  float angle = angle0Degrees + slope * (position - position0Degrees);
  Serial.print("Current angle: ");
  Serial.print(angle);
  Serial.print(" degrees");
  Serial.print(" Position ");
  Serial.print(position);
  return angle;
}

void setup() {
  Serial.begin(115200);
  while (!Serial); // Wait for Serial Monitor
  delay(2000);

  // Initialize Dynamixel communication
  DXL_SERIAL.begin(DXL_BAUDRATE);
  motor = new motorControl(dxl); // Ensure motorControl constructor initializes DXL correctly
  pinMode(DXL_DIR_PIN, OUTPUT);

  dxl.setPortProtocolVersion(2.0);

  if (dxl.ping(motorID)) {
    Serial.println("Motor found!");
  } else {
    Serial.println("Motor not responding");
    while(1); // Halt if motor not found
  }

  // Set motor to PWM control mode
  dxl.setOperatingMode(motorID, OP_PWM);
  delay(100);

  // Enable torque
  dxl.torqueOn(motorID);
  delay(100);

}

void loop() {
  int32_t currentPosition = dxl.getPresentPosition(motorID);

  float angle = calculateAngle(currentPosition);
  
  // Gravity compensation
  float total_angle_rad = deg2rad(angle + 20.0f); // Add 20° offset 
  float desired_torque_nm = ARM_MASS_KG * GRAVITY_ACCELERATION * ARM_CM_DISTANCE_M * sin(total_angle_rad) * (-1);

  Serial.print("Torque needed (Nm): ");
  Serial.println(desired_torque_nm);
  
  float angularVel = 0.0;

  // Convert the final desired torque to PWM
  int16_t pwmValue = calculatePWM(desired_torque_nm, angularVel); // Removed rot_dir from here

  if (currentPosition <= POSITION_MIN) {
    int error = POSITION_MIN - currentPosition;
    pwmValue = constrain(error * 4, 0, 885); // simple proportional "pushback"
    Serial.print("Upper limit ");
  } else if (currentPosition >= POSITION_MAX) {
    int error = POSITION_MAX - currentPosition;
    pwmValue = constrain(error * 4, -885, 0); // pushback in opposite direction
    Serial.print("Lower limit ");
  }
  

  dxl.setGoalPWM(motorID, pwmValue);
  delay(10); // Loop delay
}