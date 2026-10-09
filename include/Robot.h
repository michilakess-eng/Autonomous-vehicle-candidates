#pragma once
#include "Config.h"
#include "Sensors.h"
#include "Display.h"
#include "Motion.h"

//Instancias unicas (definidas en main.cpp)
extern Sensors sensors;
extern Display display;
extern Motion motion;

void tick();              //mantiene vivos IMU/ToF/LCD/Pi. Llamar en loops de espera
void waitMs(uint32_t ms); //delay() con llamadas a tick()