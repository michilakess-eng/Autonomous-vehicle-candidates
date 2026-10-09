#pragma once
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdarg.h>
#include <algorithm>
using std::min; using std::max;
#define HIGH 1
#define LOW 0
#define INPUT 0
#define OUTPUT 1
#define INPUT_PULLUP 2
#define FALLING 2
#define IRAM_ATTR
#define SERIAL_8N1 0
#define constrain(a,lo,hi) ((a)<(lo)?(lo):((a)>(hi)?(hi):(a)))
inline void pinMode(int,int){}
inline void digitalWrite(int,int){}
inline int digitalRead(int){return 0;}
inline int analogRead(int){return 0;}
extern uint32_t g_ms;
inline uint32_t millis(){return g_ms;}
inline uint32_t micros(){return 0;}
inline void delay(uint32_t){}
inline int digitalPinToInterrupt(int p){return p;}
inline void attachInterrupt(int,void(*)(),int){}
inline void noInterrupts(){} inline void interrupts(){}
inline void ledcSetup(int,int,int){} inline void ledcAttachPin(int,int){} inline void ledcWrite(int,uint32_t){}
struct SerialStub{
  void begin(long,int=0,int=-1,int=-1){}
  int available(){return 0;} int read(){return 0;}
  void println(const char* m){ if(getenv("SIMLOG")) fprintf(stderr,"%s\n",m); }
  void printf(const char* f,...) __attribute__((format(printf,2,3))){ if(getenv("SIMLOG")){ va_list a; va_start(a,f); vfprintf(stderr,f,a); va_end(a);} }
};
extern SerialStub Serial, Serial2;
