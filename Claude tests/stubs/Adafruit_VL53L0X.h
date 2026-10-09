#pragma once
#include <Wire.h>
class Adafruit_VL53L0X{ public:
  bool begin(uint8_t a=0x29,bool d=false,TwoWire* w=&Wire){return true;}
  bool startRangeContinuous(uint16_t p=50){return true;}
  bool isRangeComplete(){return true;} uint16_t readRangeResult(){return 0;}
};
