#pragma once
#include "Dynamixel2Arduino.h"

class motorControl{

    private:

        Dynamixel2Arduino *dxl;

    public:

        motorControl(Dynamixel2Arduino &dxl);
        void startMotor();

};