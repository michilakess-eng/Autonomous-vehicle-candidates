#ifndef SENSORS_H
#define SENSORS_H
#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <Adafruit_TCS34725.h>
#include <MPU6500_WE.h>
#include "Config.h"

//struct to return the tof readings in one same call
struct ToFDistances{
    uint16_t left;
    uint16_t front;
    uint16_t right;
};

//struct to hold the raw color values
struct RGBValues{
    uint16_t r,g,b,c;
};

//HAL class, learned in STM32 course.
//main code calls simple functions, much cleaner
class Sensors{
    private:
        Adafruit_VL53L0X tofLeft;
        Adafruit_VL53L0X tofFront;
        Adafruit_VL53L0X tofRight;
        Adafruit_TCS34725 rgbSensor;
        MPU6500_WE imu;

        void initToF();
        void initIMU();
        void initRGBs();

        public:
            Sensors();
            void begin();
            
            //switch multiplexer to specific channelr 
            void tcaselect(uint8_t channel);

            //read funcs
            ToFDistances readToF();
            RGBValues readRGB(uint8_t channel);

            //IMu
            float getMaxTilt();
            float getYaw();

            //IR sensors
            bool isLeftOuterBlack();
            bool isRightOuterBlack();
            bool isInnerFrontBlack();
            bool isBackCornerBlack();
};

//global idgaf
extern Sensors robotSensors;
#endif