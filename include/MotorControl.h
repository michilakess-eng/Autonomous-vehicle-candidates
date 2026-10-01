#ifndef MOTORCONTROL_H
#define MOTORCONTROL_H
#include <Arduino.h>
#include "Config.h"
#include "Sensors.h"
#include "Interrupts.h"
//herein will lie the methods for the motor control. It will be a differential driving system.

class MotorControl{
    public:
        void begin();
        void setMotors(int leftSpeed, int rightSpeed);
        void brakeM();
        
        //axis turn
        void driveDistance(float mm);
        void turnIMU(float target_angle);

};

extern MotorControl robotMotors;
#endif