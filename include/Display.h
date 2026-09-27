#ifndef DISPLAY_H
#define DISPLAY_H
#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include "Config.h"

class Display{
    private:
        LiquidCrystal_I2C lcd;
        String lastLine0;
        String lastLine1;

    public:
        Display();
        void begin();
        void showStatus(const String &line0, const String &line1);
        void showTile(FloorColor color);
        String colorToString(FloorColor color);
};

#endif