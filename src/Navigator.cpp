#include "Navigator.h"
#include "Robot.h"

Navigator nav;

MoveResult Navigator::moveOneCell(const MoveOpts& o) {
  MoveResult R;
  Enc::reset();
  tracker.reset();

  float target= CELL_MM; //fallback si no hay linea negra
  bool lineSeen= false; float lineAt= 0, lastLineAt= -1000;
  uint8_t votes[COL_COUNT]= {0};
  uint32_t lastRgb= 0;
  bool rampPending= false;
  float wallOffset= 0;
  bool haveW0= false, haveW1= false, wallsLost= false; //re-sync yaw
  float L0= 0, R0= 0, d0= 0, L1= 0, R1= 0, d1= 0;
  uint32_t t0= millis(), deadline= t0 + 7000;

  while(true) {
    tick();
    float d= Enc::travelMm();
    float pitch= sensors.pitch(), tilt= sensors.tilt();

    //obstaculos
    ObstacleEvent ev= tracker.update(pitch, sensors.roll(), d);
    if(ev== OBS_BUMP_HALF) R.bumpHalves++;
    else if(ev== OBS_STAIRS) { R.stairs= true; target= fmaxf(2.0f, roundf(d/CELL_MM)) * CELL_MM; }
    else if(ev== OBS_RAMP_UP) { R.ramp= true; rampPending= true; deadline= millis() + 15000; }
    else if(ev== OBS_RAMP_DOWN) { rampPending= false; target= fmaxf(3.0f, roundf(d/CELL_MM)) * CELL_MM; }
    if(tilt> TILT_ON_DEG && deadline< millis() + 8000) deadline= millis() + 8000;

    bool flat= tilt< TILT_OFF_DEG && !tracker.active() && !rampPending;

    //linea puente detectada (barra frontal)
    if(flat && d - lastLineAt >= LINE_MIN_SPACING_MM && sensors.frontLine()) {
      lastLineAt= d;
      if(!lineSeen) {
        lineSeen= true; lineAt= d;
        if(!R.stairs && !R.ramp) target= d + SENSOR_AHEAD_MM + CELL_MM * 0.5f; //centrar sig celda
      }
    }

    //color celda entrada
    if(o.readColor && lineSeen && d - lineAt >= COLOR_WIN_START_MM && d - lineAt <= COLOR_WIN_END_MM &&
       millis() - lastRgb >= RGB_PERIOD_MS) {
      lastRgb= millis();
      FloorColor c= sensors.color(RGB_CH_CENTER);
      votes[c]++;
      //evasion casilla roja (Pista A)
      if(!o.allowRed && c== COL_RED && votes[COL_RED] >= 2) {
        motion.brake(); waitMs(60);
        float back= Enc::travelMm();
        motion.driveDistance(-back, SPEED_REVERSE, o.heading); //retroceso seguro origen
        R.abortedRed= true; R.color= COL_RED;
        return R;
      }
    }

    //velocidad y picos de torque
    int base= o.speed;
    bool slowZone= o.readColor && ((!lineSeen && d< 60.0f) || (lineSeen && d - lineAt < COLOR_WIN_END_MM + 10.0f));
    if(slowZone) base= min(base, SPEED_PROBE);
    if(tilt> TILT_ON_DEG) base= (pitch< -TILT_ON_DEG) ? SPEED_DESCENT : SPEED_MAX; //100% PWM solo en subida
    else if(flat && target - d < 40.0f) base= min(base, SPEED_PROBE);

    //alineacion con paredes (Pista A)
    if(o.wallCenter && flat) {
      float L= sensors.left(), Rr= sensors.right(), want= 0;
      bool wl= L< WALL_SIDE_MM, wr= Rr< WALL_SIDE_MM;
      if(wl && wr)      want= WALL_CENTER_KP * (Rr - L) * 0.5f;
      else if(wl)       want= WALL_CENTER_KP * (WALL_TARGET_MM - L);
      else if(wr)       want= -WALL_CENTER_KP * (WALL_TARGET_MM - Rr);
      wallOffset+= 0.2f * (clampf(want, -WALL_CENTER_MAX, WALL_CENTER_MAX) - wallOffset);
      if(wl && wr) {
        if(!haveW0 && d> 20.0f) { haveW0= true; L0= L; R0= Rr; d0= d; }
        else if(haveW0) { haveW1= true; L1= L; R1= Rr; d1= d; }
      } else if(haveW0) wallsLost= true; //hueco detectado, abortar auto-alineacion
    } else wallOffset*= 0.9f;

    //failsafe frontal
    if(flat && d> 5.0f && sensors.front() < FRONT_STOP_MM) { motion.brake(); R.blocked= true; break; }

    motion.holdHeading(base, o.heading, wallOffset);

    //cierre de ciclo
    if(d>= target - BRAKE_COMP_MM && tilt< TILT_ON_DEG && !tracker.bigActive(d) && !rampPending) {
      motion.brake(); R.ok= true; break;
    }
    if(millis() > deadline) { motion.brake(); break; }
    delay(1);
  }

  //resincronizacion giroscopio (elimina drift en rectas)
  if(YAW_RESYNC && R.ok && R.cells== 1 && haveW1 && !wallsLost && d1 - d0 >= 150.0f) {
    float delta= atanf(((L1 - L0) - (R1 - R0)) / (2.0f * (d1 - d0))) * 57.29578f; //angulo vs pasillo
    float err= wrap180(o.heading - (sensors.yaw() - delta)); //error marco inercial
    sensors.shiftYaw(clampf(0.7f * err, -6.0f, 6.0f));
  }
  waitMs(40);
  float dEnd= Enc::travelMm();
  if(R.ok) R.cells= (uint8_t)constrain((int)roundf(dEnd/CELL_MM), 1, 3);
  uint8_t best= 0;
  for(uint8_t c= 0; c< COL_COUNT; c++) if(votes[c]> best) { best= votes[c]; R.color= (FloorColor)c; }
  if(best< 2) R.color= COL_NONE;
  if(R.ok && R.cells> 1 && o.readColor) { //lectura estacionaria post-obstaculo
    uint8_t v[COL_COUNT]= {0}; uint8_t b2= 0;
    for(int i= 0; i< 3; i++) { v[sensors.color(RGB_CH_CENTER)]++; waitMs(10); }
    for(uint8_t c= 0; c< COL_COUNT; c++) if(v[c]> b2) { b2= v[c]; R.colorEnd= (FloorColor)c; }
  }
  return R;
}

float Navigator::measureWallAngle(float heading, float mm, bool reverse) {
  ToF a= sensors.avgToF(5);
  float s0= Enc::travelMm();
  motion.driveDistance(reverse ? -mm : mm, SPEED_PROBE, heading);
  waitMs(80);
  ToF b= sensors.avgToF(5);
  float travel= Enc::travelMm() - s0;
  if(fabsf(travel) < 20.0f) return 0;
  float sum= 0; int n= 0;
  if(a.left< WALL_SIDE_MM && b.left< WALL_SIDE_MM)    { sum+= atanf(((float)b.left - a.left) / travel) * 57.29578f; n++; }
  if(a.right< WALL_SIDE_MM && b.right< WALL_SIDE_MM)  { sum-= atanf(((float)b.right - a.right) / travel) * 57.29578f; n++; }
  return n ? sum/n : 0.0f;
}