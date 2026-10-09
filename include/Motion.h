#pragma once
#include <Arduino.h>
#include "Config.h"

//Dif drive. IMU orienta, encoders miden
class Motion {
public:
  void begin();
  void drive(int left, int right); //PWM con signo, (0,0)=brake
  void brake();
  void holdHeading(int base, float targetDeg, float offsetDeg= 0); //paso PID
  bool turnTo(float targetDeg); //bloqueante, false si timeout
  bool driveDistance(float mm, int speed, float headingDeg); //bloqueante, false obstaculo/timeout
};