#include "Sensors.h"
 Sensors::Sensors()
    : rgbSensor(TCS34725_INTEGRATIONTIME_24MS, TCS34725_GAIN_4X),
    imu(IMU_ADDR),
    lastColorScoreTime(0){}

bool Sensors::begin(){
    Wire.begin(I2C_SDA, I2C_SCL);

    //XSHUT must give unique I2C to not talk over each other
    // shutdown, wakeup and change addressess
    pinMode(TOF_left_XSHUT, OUTPUT);
    pinMode(TOF_right_XSHUT, OUTPUT);
    pinMode(TOF_front_XSHUT, OUTPUT);

    digitalWrite(TOF_left_XSHUT, LOW);
    digitalWrite(TOF_right_XSHUT, LOW);
    digitalWrite(TOF_front_XSHUT, LOW);
    delay(10);

    //wake assign above order
    digitalWrite(TOF_left_XSHUT, HIGH);
    delay(10);
    if(!tofLeft.begin(TOF_left_ADDR)) return false;

    digitalWrite(TOF_right_XSHUT, HIGH);
    delay(10);
    if(!tofRight.begin(TOF_right_ADDR)) return false;

    digitalWrite(TOF_front_XSHUT, HIGH);
    delay(10);
    if(!tofFront.begin(TOF_front_ADDR)) return false;

    //imu
    if(!imu.init()) return false;
    imu.autoOffsets();//calibration KEEP VERY STILL;

    //RGB color sensor
    if(!rgbSensor.begin()) return false;

    return true;
}

    //TOF METHODS

int Sensors::getLeftDist(){
    VL53L0X_RangingMeasurementData_t measure;
    tofLeft.rangingTest(&measure, false);
    if(measure.RangeStatus != 4) return measure.RangeMilliMeter;
    return 8000; //max value corridor
}

int Sensors::getRightDist(){
    VL53L0X_RangingMeasurementData_t measure;
    tofRight.rangingTest(&measure, false);
    if(measure.RangeStatus != 4) return measure.RangeMilliMeter;
    return 8000;
}

int Sensors::getFrontDist(){
    VL53L0X_RangingMeasurementData_t measure;
    tofFront.rangingTest(&measure, false);
    if(measure.RangeStatus != 4) return measure.RangeMilliMeter;
    return 8000;
}

bool Sensors::isWallAhead(){
    return (getFrontDist() < wall_stop_dist);
}

    //IMU (MPU6500) METHODS

float Sensors::getPitch(){
    xyzFloat angles= imu.getAngles();
    return angles.y; //tilt on stair and ramps
}

MPU6500_WE& Sensors::getIMU(){
    return imu;
}

    //RGB TCS34725 METHODS

FloorColor Sensors::readFloorColor(bool useCooldown){
    //if in Pista A
    if(useCooldown && (millis() - lastColorScoreTime < color_cooldown_ms)){
        return NONE;
    }

    uint16_t rRaw, gRaw, bRaw, cRaw;
    rgbSensor.getRawData(&rRaw, &gRaw, &bRaw, &cRaw);

    if(cRaw== 0) return NONE; //no /0

    //normalize for detection through shadows
    float r= (float)rRaw/cRaw;
    float g= (float)gRaw/cRaw;
    float b= (float)bRaw/cRaw;

    FloorColor detected= classifyColor(r, g, b, cRaw);

    //cooldown if color
    if(useCooldown && detected != WHITE && detected != NONE){
        lastColorScoreTime= millis();
    }
    return detected;
}

FloorColor Sensors::classifyColor(float r, float g, float b, uint16_t c){
    //white
    if(c>2500 && r>0.28 && g>0.28 &&b>0.25) return WHITE;

    //subtractive logic. TEST AND CALLIBRATE
    if(r>0.45 && g<0.28 && b<0.28) return RED; //final casilla
    if(r>0.42 && g<0.32 && b<0.22) return ORANGE;
    if(r>0.36 && g<0.38 && b<0.22) return YELLOW;
    if(r>0.28 && g<0.42 && b<0.28) return GREEN; //initial casilla
    if(r>0.25 && g<0.35 && b<0.35) return CYAN;
    if(r>0.35 && g<0.25 && b<0.35) return MAGENTA;

    return NONE;
}
