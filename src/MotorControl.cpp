#include "MotorControl.h"

MotorControl::MotorControl(){}

void MotorControl::begin(){
    pinMode(M_left_PWM, OUTPUT);
    pinMode(M_left_IN1, OUTPUT);
    pinMode(M_left_IN2, OUTPUT);

    pinMode(M_right_PWM, OUTPUT);
    pinMode(M_right_IN1, OUTPUT);
    pinMode(M_right_IN2, OUTPUT);

    brakeM(); //in park when in boot
}

void MotorControl::setLeftM(int speed, bool forward){
    digitalWrite(M_left_IN1, forward? HIGH: LOW);
    digitalWrite(M_left_IN2, forward? LOW: HIGH);
    analogWrite(M_left_PWM, speed);
}

void MotorControl::setRightM(int speed, bool forward){
    digitalWrite(M_right_IN1, forward? HIGH: LOW);
    digitalWrite(M_right_IN2, forward? LOW: HIGH);
    analogWrite(M_right_PWM, speed);
}

//locomotion
void MotorControl::driveForward(int speed){
    setLeftM(speed, true);
    setRightM(speed, true);
}

void MotorControl::driveBackward(int speed){
    setLeftM(speed, false);
    setRightM(speed, false);
}

void breakM(){ //necsario ocupar la TB6612FNG
    digitalWrite(M_left_IN1, HIGH);
    digitalWrite(M_left_IN2, HIGH);
    analogWrite(M_left_PWM, 255);

    digitalWrite(M_right_IN1, HIGH);
    digitalWrite(M_right_IN2, HIGH);
    analogWrite(M_right_PWM, 255);
}

//imu and terrain
void MotorControl::rampTorque(float currentPitch){
    //will need to review this under practical tests to prevent over shooting
    if(abs(currentPitch)>pitch_ramp){
        driveForward(speed_max);
    }else{
        driveForward(speed_norm);
    }
}

void MotorControl::turnIMU(float targetAngle, bool turnRight, MPU6500_WE &imu){
    int i=1;//pass basically
}