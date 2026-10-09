#include "Motion.h"
#include "Robot.h"
#include "Pwm.h"
#include "Sensors.h"

static void sideDrive(int in1, int in2, int pin, int ch, int spd) {
  spd= constrain(spd, -255, 255);
  if(spd> 0) { digitalWrite(in1, HIGH); digitalWrite(in2, LOW); }
  else if(spd< 0) { digitalWrite(in1, LOW); digitalWrite(in2, HIGH); }
  else { digitalWrite(in1, LOW); digitalWrite(in2, LOW); }
  pwmWrite(pin, ch, abs(spd));
}

void Motion::begin() {
  const int pins[]= {M_LEFT_IN1, M_LEFT_IN2, M_RIGHT_IN1, M_RIGHT_IN2};
  for(int p : pins) pinMode(p, OUTPUT);
  pwmAttach(M_LEFT_PWM, PWM_CH_LEFT, PWM_FREQ, 8);
  pwmAttach(M_RIGHT_PWM, PWM_CH_RIGHT, PWM_FREQ, 8);
  brake();
}

void Motion::drive(int l, int r) {
  if(l== 0 && r== 0) { brake(); return; }
  Enc::setDirection(l, r); //signo encoder viene de aqui
  sideDrive(M_LEFT_IN1, M_LEFT_IN2, M_LEFT_PWM, PWM_CH_LEFT, l);
  sideDrive(M_RIGHT_IN1, M_RIGHT_IN2, M_RIGHT_PWM, PWM_CH_RIGHT, r);
}

void Motion::brake() { //TB6612 short brake: IN1=IN2=HIGH
  digitalWrite(M_LEFT_IN1, HIGH); digitalWrite(M_LEFT_IN2, HIGH); pwmWrite(M_LEFT_PWM, PWM_CH_LEFT, 255);
  digitalWrite(M_RIGHT_IN1, HIGH); digitalWrite(M_RIGHT_IN2, HIGH); pwmWrite(M_RIGHT_PWM, PWM_CH_RIGHT, 255);
}

void Motion::holdHeading(int base, float targetDeg, float offsetDeg) {
  float err= wrap180(targetDeg + offsetDeg - sensors.yaw());
  float steer= clampf(HEADING_KP * err - HEADING_KD * sensors.yawRate(), -STEER_MAX, STEER_MAX);
  //funciona en reversa con velocidades con signo
  drive((int)(base + steer), (int)(base - steer));
}

bool Motion::turnTo(float target) {
  uint32_t t0= millis(), okSince= 0;
  while(millis() - t0 < 3500) {
    tick();
    float err= wrap180(target - sensors.yaw());
    float rate= sensors.yawRate();
    bool inTol= fabsf(err)< TURN_TOL_DEG;
    if(inTol && fabsf(rate)< 15.0f) {
      if(!okSince) okSince= millis();
      brake();
      if(millis() - okSince > 80) return true;
    } else {
      okSince= 0;
      //frenar anticipado considerando inercia
      if(fabsf(err)<= fabsf(rate) * TURN_LEAD_S + TURN_TOL_DEG) {
        brake();
      } else {
        int p= constrain((int)(fabsf(err) * TURN_KP), SPEED_TURN_MIN, SPEED_TURN_MAX);
        if(err> 0) drive(p, -p); else drive(-p, p); //err> 0 gira derecha
      }
    }
    delay(1);
  }
  brake();
  return false;
}

bool Motion::driveDistance(float mm, int speed, float headingDeg) {
  float s0= Enc::travelMm();
  float dist= fabsf(mm);
  int dir= (mm>= 0)?1:-1;
  uint32_t t0= millis(), limit= 1500 + (uint32_t)(dist * 15.0f);
  while(millis() - t0 < limit) {
    tick();
    float done= fabsf(Enc::travelMm() - s0);
    if(done>= dist - (dir> 0?BRAKE_COMP_MM * 0.5f:0)) { brake(); return true; }
    if(dir> 0 && sensors.front()< FRONT_STOP_MM) { brake(); return false; } //failsafe frontal
    int sp= abs(speed);
    if(dist - done < 40.0f) sp= min(sp, SPEED_PROBE);
    holdHeading(dir * sp, headingDeg);
    delay(1);
  }
  brake();
  return false;
}