#pragma once
#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <Adafruit_TCS34725.h>
#include "Config.h"

//struct to return the tof readings in one same call
struct ToF { uint16_t left, front, right; };

//struct to hold the raw color values
struct RGBRaw { uint16_t r, g, b, c; };

//La dirección verndrá más bien del motor command sign, reversa es neg, axis turn cancels out
namespace Enc{
    void begin();
    void reset();
    long left();
    long right();
    float travelMm();
    void setDirection(int l, int r); // -1, 0, +1 
}

class Sensors{
    public:
        uint8_t begin(); // b0 IMU, b1-3 ToF L/F/R, b4-6 RGB L/C/R
        void calibrateGyro(uint32_t maxWaitMs= 12000);
        void update();

        //IMU
        float yaw() const { return yawDeg; }
        float yawRate() const { return yawRateDps; }
        float pitch() const { return pitchDeg; }
        float roll() const { return rollDeg; }
        float tilt() const { return fmaxf(fabsf(pitchDeg), fabsf(rollDeg)); }
        void shiftYaw(float d) { yawDeg += d; }

        //ToF latest values
        uint16_t left() const { return dL; }
        uint16_t front() const { return dF; }
        uint16_t right() const { return dR; }
        ToF avgToF(uint8_t frames= 4);

        //RGB
        RGBRaw readRGB(uint8_t ch);
        FloorColor classify(const RGBRaw& v);
        FloorColor color(uint8_t ch) { return classify(readRGB(ch)); }
        void hsv(const RGBRaw& v, float& h, float& s, float& val);

        //IR
        bool frontLine();                 // any front IR over black
        bool backLine();
        int  outerLeftAnalog() { return analogRead(IR_OUTER_LEFT); }
        int  outerRightAnalog() { return analogRead(IR_OUTER_RIGHT); }

        //Raspberry Pi aruco inshallah
        int  arucoId() const { return arucoLocked; }
        bool takeArucoNew() { bool f = arucoFresh; arucoFresh = false; return f; }


    private:
        bool initIMU();
        uint8_t initToF();
        uint8_t initRGB();
        void updateIMU();
        void pollToF();
        void pollPi();
        void tcaSelect(uint8_t ch);
        bool readIMURaw(float& ax, float& ay, float& az, float& gx, float& gy, float& gz);
        Adafruit_VL53L0X tofL, tofF, tofR;
        Adafruit_TCS34725 rgb[3] = {
            Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_24MS, TCS34725_GAIN_4X),
            Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_24MS, TCS34725_GAIN_4X),
            Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_24MS, TCS34725_GAIN_4X)};


        float yawDeg = 0, pitchDeg = 0, rollDeg = 0, yawRateDps = 0;
        float biasGx = 0, biasGy = 0, biasGz = 0, pitch0 = 0, roll0 = 0;
        uint32_t lastImuUs = 0, lastTofMs = 0;
        uint16_t dL = TOF_FAR_MM, dF = TOF_FAR_MM, dR = TOF_FAR_MM;
        uint32_t tofFrames = 0;

        char piBuf[24]; uint8_t piLen = 0;
        int arucoCand = -1, arucoCount = 0, arucoLocked = -1; bool arucoFresh = false;
};