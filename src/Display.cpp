#include "Display.h"

Display::Display()
    : lcd(LCD_ADDR, 16, 2), lastLine0(""), lastLine1(""){}

void Display::begin(){
    lcd.init();
    lcd.backlight();
    showStatus("Booting", "Keep very still");
}

void Display::showStatus(const String &line0, const String &line1){
    //only if the text has actually changed will is use the I2C bus
    if(line0== lastLine0 && line1== lastLine1) return;
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(line0.substring(0, 16));
    lcd.setCursor(0,1);
    lcd.print(line1.substring(0, 16));

    lastLine0= line0;
    lastLine1= line1;
}

void Display::showTile(FloorColor color){;
    showStatus("Tile Detected: ", colorToString(color));
}

String Display::colorToString(FloorColor color){
    switch(color){
        case CYAN: return "CYAN";
        case YELLOW: return "YELLOW";
        case ORANGE: return "ORANGE";
        case MAGENTA: return "MAGENTA";
        case GREEN: return "START";
        case RED: return "END";
        case WHITE: return "WHITE";
        default: return "NONE";
    }
}