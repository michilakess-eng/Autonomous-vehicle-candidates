#ifndef INTERRUPTS_H
#define INTERRUPTS_H
#include <Arduino.h>
#include "Config.h"

//volatile varaibles
volatile int gridSteps= 0;
volatile bool leftLineHit= false;
volatile bool rightLineHit= false;
volatile bool isTurning= false; //in IMU turns, its true to ignore floor lines inshallah it will work

volatile unsigned long lastLeftIrTime= 0;
volatile unsigned long lastRightIrTime= 0;

//left IR interrupt service routine
void IRAM_ATTR onLeftLineISR(){
    unsigned long now= millis();
    if(!isTurning && (now - lastLeftIrTime > ir_debounce_ms)){
        leftLineHit= true;
        gridSteps++; //cell counter on grid lines
        lastLeftIrTime= now;
    }
}

//right IR interrupt service routine
void IRAM_ATTR onRightLineISR(){
    unsigned long now= millis();
    if(!isTurning && (now - lastRightIrTime > ir_debounce_ms)){
        rightLineHit= true;
        lastRightIrTime= now;
    }
}

//missing setipInterruptor function that will be called in main. research due.

#endif