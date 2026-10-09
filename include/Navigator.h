#pragma once
#include <Arduino.h>
#include "Config.h"
#include "Obstacles.h"

struct MoveOpts {
  float heading= 0; //target yaw
  int speed= SPEED_CRUISE;
  bool readColor= true;
  bool allowRed= true; //aborta si ve rojo
  bool wallCenter= true; //centrado ToF
};

struct MoveResult {
  bool ok= false;
  bool blocked= false; //pared frontal
  bool abortedRed= false;
  uint8_t cells= 0; //1 normal, 2 stairs, 3 ramp
  FloorColor color= COL_NONE;
  FloorColor colorEnd= COL_NONE;
  uint8_t bumpHalves= 0;
  bool stairs= false, ramp= false;
};

class Navigator {
public:
  MoveResult moveOneCell(const MoveOpts& o);
  float measureWallAngle(float headingDeg, float mm, bool reverse); //yaw relativo a paredes
private:
  ObstacleTracker tracker;
};

extern Navigator nav;