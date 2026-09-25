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

void brakeM(){ //necsario ocupar la TB6612FNG
    digitalWrite(M_left_IN1, HIGH);
    digitalWrite(M_left_IN2, HIGH);
    analogWrite(M_left_PWM, 255);

    digitalWrite(M_right_IN1, HIGH);
    digitalWrite(M_right_IN2, HIGH);
    analogWrite(M_right_PWM, 255);
}

void MotorControl::driveSteering(int leftSpeed, int rightSpeed){
    leftSpeed= constrain(leftSpeed, 0, 255);
    rightSpeed= constrain(rightSpeed, 0, 255);

    setLeftM(leftSpeed, true);
    setRightM(rightSpeed, true);
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
    float currentYaw= 0.0;
    unsigned long lastTime= millis();

    if(turnRight){
        setLeftM(speed_turn, true);
        setRightM(speed_turn, false);
    }else{
        setLeftM(speed_turn, false);
        setRightM(speed_turn, true);
    }

    while(abs(currentYaw) < (targetAngle - turn_tolerance)){
        xyzFloat g= imu.getGyrValues();

        unsigned long currentTime= millis();
        float deltaTime= (currentTime - lastTime)/1000.0;
        lastTime= currentTime;

        currentYaw+= g.z * deltaTime;

        //gemini recommended delay to prevent esp32 watchdog timer crashes in blocking loop
        delay(10);
    }
    brakeM();
}