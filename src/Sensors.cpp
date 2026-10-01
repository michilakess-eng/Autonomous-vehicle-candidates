#include "Sensors.h"

Sensors robotSensors;

//imu initialize
Sensors::Sensors():imu(IMU_ADDR){}

void Sensors::begin(){
    //start 12c bus
    Wire.begin(I2C_SDA,I2C_SCL);

    //Ir pins as digital inputs
    pinMode(IR_outer_left,INPUT);
    pinMode(IR_outer_right,INPUT);
    pinMode(IR_inner_front,INPUT);
    pinMode(IR_back_corner,INPUT);

    //subsystems
    initToF();
    initIMU();
    initRGBs();
}

//tells multiplexer which channel to open
//and then shifting a bit to flip the swithc
void Sensors::tcaselect(uint8_t channel){
    if(channel>7) return;
    Wire.beginTransmission(TCA9548A_ADDR);
    Wire.write(1<<channel);
    Wire.endTransmission();
}
    //TOF boot
void Sensors::initToF(){
    pinMode(TOF_left_XSHUT,OUTPUT);
    pinMode(TOF_front_XSHUT,OUTPUT);
    pinMode(TOF_right_XSHUT,OUTPUT);
    
    //reset
    digitalWrite(TOF_left_XSHUT,LOW);
    digitalWrite(TOF_front_XSHUT,LOW);
    digitalWrite(TOF_right_XSHUT,LOW);
    delay(10);
    
    // the rest assigns the unique adddress
    digitalWrite(TOF_left_XSHUT,HIGH);
    delay(10);
    tofLeft.begin(TOF_left_ADDR);
    //front
    digitalWrite(TOF_front_XSHUT,HIGH);
    delay(10);
    tofFront.begin(TOF_front_ADDR);
//right
    digitalWrite(TOF_right_XSHUT,HIGH);
    delay(10);
    tofRight.begin(TOF_right_ADDR);
}

//rgb boot, channel multiplexer
void Sensors::initRGBs(){
    rgbSensor= Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_24MS,TCS34725_GAIN_4X);
    
    tcaselect(RGB_CHAN_LEFT);
    rgbSensor.begin();
    
    tcaselect(RGB_CHAN_CENTER);
    rgbSensor.begin();
    
    tcaselect(RGB_CHAN_RIGHT);
    rgbSensor.begin();
}

//imu boot KEEP STILL
void Sensors::initIMU(){
    imu.init();
    imu.autoOffsets(); 
}


//second part, data reading funcs

//read all tofs
ToFDistances Sensors::readToF(){
    ToFDistances dist;
    dist.left= tofLeft.readRange();
    dist.front= tofFront.readRange();
    dist.right= tofRight.readRange();
    return dist;
}

//reads raw data from a specific rgb
RGBValues Sensors::readRGB(uint8_t channel){
    tcaselect(channel); //open speifici channel
    RGBValues vals;
    rgbSensor.getRawData(&vals.r,&vals.g,&vals.b,&vals.c);
    return vals;
}

//which obstacle depending on tilt. ask about speedbumps, im a little stumped
 float Sensors::getMaxTilt(){
    xyzFloat angles= imu.getAngles();
    return max(abs(angles.x),abs(angles.y));
}

//current z axis from imu
float Sensors::getYaw(){
    xyzFloat angles= imu.getAngles();
    return angles.z;
}
/*
IR sensors.
High (1) must mean black or empty air.
low(0) must mean white tape or standard floor
*/
bool Sensors::isLeftOuterBlack(){
    return digitalRead(IR_outer_left) == HIGH;
}

bool Sensors::isRightOuterBlack(){
    return digitalRead(IR_outer_right) == HIGH;
}

bool Sensors::isInnerFrontBlack(){
    return digitalRead(IR_inner_front) == HIGH;
}

bool Sensors::isBackCornerBlack(){
    return digitalRead(IR_back_corner) == HIGH;
}