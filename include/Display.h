#pragma once
#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include "Config.h"

//hold() garantiza msg >=3s (reglamento). status() es idle
class Display {
public:
  void begin();
  void status(const char* l0, const char* l1);
  void hold(const char* l0, const char* l1, uint32_t ms= 3000);
  void update();

private:
  struct Msg { char a[17]; char b[17]; uint32_t ms; };
  static constexpr uint8_t QN= 6;
  Msg q[QN]; uint8_t qHead= 0, qCount= 0;
  bool holding= false; uint32_t holdUntil= 0;
  char cur0[17]= "", cur1[17]= "", st0[17]= "", st1[17]= "";
  LiquidCrystal_I2C lcd{LCD_ADDR, 16, 2};
  
  void draw(const char* a, const char* b);
  static void copy16(char* dst, const char* src);
};