#include <motorControl.h>

motorControl::motorControl(Dynamixel2Arduino &dxl_in){

    dxl = &dxl_in;
    const float DXL_PROTOCOL_VERSION = 2.0;
    dxl->begin(57600);
    dxl->setPortProtocolVersion(DXL_PROTOCOL_VERSION);


    delay(500);
    // startMotor();

}

void motorControl::startMotor(){

    u_int8_t motorID = 3;
    dxl->torqueOff(motorID);
    dxl->setOperatingMode(motorID, OP_PWM);
    dxl->torqueOn(motorID);
    // dxl->ledOff(motorID);
    // dxl->ledOn(motorID);
}