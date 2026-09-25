#ifndef SENSORS_H
#define SENSORS_H
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <Adafruit_TCS34725.h>
#include <MPU6500_WE.h>
#include "Config.h"

class Sensors{
    private:
        Adafruit_VL53L0X tofLeft;
        Adafruit_VL53L0X tofFront;
        Adafruit_VL53L0X tofRight;
        Adafruit_TCS34725 rgbSensor;
        MPU6500_WE imu;

        unsigned long lastColorScoreTime;
        FloorColor classifyColor(float r, float g, float b, uint16_t c);

        public:
            Sensors();
            bool begin();
            
            //ToF reading mm
            int getLeftDist();
            int getFrontDist();
            int getRightDist();
            bool isWallAhead();

            //IMU
            float getPitch();
            MPU6500_WE& getIMU();

            //RGB floor color
            FloorColor readFloorColor(bool useCooldown);
};

#endif