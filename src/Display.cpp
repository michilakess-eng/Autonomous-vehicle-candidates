#pragma once
#include <Arduino.h>
#include "Config.h"
#include "Pwm.h"

//Servo en LEDC (50Hz, 16bit). Angulos exactos vienen de Mecanica en Config.h.
class Claw {
public:
  void begin() { pwmAttach(SERVO_CLAW, PWM_CH_SERVO, 50, 16); setAngle(CLAW_OPEN_DEG); }
  
  void setAngle(float deg) {
    deg= clampf(deg, 0, 180);
    float pulseUs= 500.0f + deg * (2000.0f/180.0f); //0.5-2.5 ms
    pwmWrite(SERVO_CLAW, PWM_CH_SERVO, (uint32_t)(pulseUs/20000.0f * 65535.0f));
  }
  
  void open()  { setAngle(CLAW_OPEN_DEG); closed= false; }
  void close() { setAngle(CLAW_CLOSED_DEG); closed= true; }
  bool isClosed() const { return closed; }
  
private:
  bool closed= false;
};