#pragma once
#include <Wire.h>
#define TCS34725_INTEGRATIONTIME_24MS 0xF6
typedef enum { TCS34725_GAIN_1X=0, TCS34725_GAIN_4X=1 } tcs34725Gain_t;
class Adafruit_TCS34725{ public:
  Adafruit_TCS34725(uint8_t it, tcs34725Gain_t g){}
  bool begin(uint8_t a, TwoWire* w){return true;}
  void getRawData(uint16_t*r,uint16_t*g,uint16_t*b,uint16_t*c){}
};
