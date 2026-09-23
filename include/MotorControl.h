#ifndef MOTORCONTROL_H
#define MOTORCONTROL_H
#include <Arduino.h>
#include <MPU6500_WE.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Config.h>
//herein will lie the methods for the motor control. It will be a differential driving system.

class MotorControl{
    private:
        void setLeftM(int speed, bool forward);
        void setRightM(int speed, bool forward);

    public:
        MotorControl();
        void begin();
        void driveForward(int speed);
        void driveBackward(int speed);
        void brakeM();
        //terrain and IMU
        void turnIMU(float targetAngle, bool turnRight, MPU6500_WE &imu);
        void rampTorque(float currentPitch);//heavy reconsiderar como se ve esto

};

#endif