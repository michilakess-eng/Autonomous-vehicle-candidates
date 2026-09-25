#ifndef CONFIG_H
#define CONFIG_H
//herein will lie the code with important variables and constants.
//Such as, pinout, the state of the robot, and the color-coded direction of Pista B
//UNDETERMINED pinout. discuss with electrical. I put numbers to visualize how the pinout is fairing.
//Technically, the h bridge can have the PWMA and PWMB wired together.
//left motors
const int M_left_PWM= 25;
const int M_left_IN1= 26;
const int M_left_IN2= 27;

//right motors
const int M_right_PWM= 14;
const int M_right_IN1= 12;
const int M_right_IN2= 13;

//turning, variables I asked Gemini to estimate for me
const int speed_norm= 150;
const int speed_max= 255;
const int speed_turn= 130;

//IMU
const float pitch_ramp= 12.0; //pitch= y axis
const float turn_tolerance= 2.0;//most tolerance in overshoot

//I2C BusPins
const int I2C_SDA= 21;
const int I2C_SCL= 22;

//ToF XSHUT SPEAK WITH ELECTRICAL ABOUT THIS!!!
const int TOF_left_XSHUT= 32;
const int TOF_front_XSHUT= 33;
const int TOF_right_XSHUT= 4;

//Unique I2C addresses for Tof and IMU
const uint8_t TOF_left_ADDR= 0x30;
const uint8_t TOF_front_ADDR= 0x31;
const uint8_t TOF_right_ADDR= 0x32;
const uint8_t IMU_ADDR= 0x68;

//TCRT5000 IR sensor pins
const int IR_left_PIN= 34;
const int IR_right_PIN= 35;

//threshold, timers TEST ALL
const int wall_stop_dist= 150; //test 15 cm
const unsigned long ir_debounce_ms=  250; //min ms between grid line triggers
const unsigned long color_cooldown_ms= 3000; //tile cooldown Pista A. maybe less time.

//colors of both Pistas. RGB sensors research due.
enum FloorColor{
    WHITE,
    GREEN,
    RED,
    CYAN,
    YELLOW,
    ORANGE,
    MAGENTA,
    NONE
};

/*
//core states 
enum RobotState{
    CALIBRATION,
    PISTA_A,
    PISTA_B_BALL,
    PISTA_B_WDODGE, //dodging the white lines
    PISTA_B_WGAP_SEEK,//seeking the gap between white lines
    PISTA_B_COLOR_NAV,
    EXECUTE_RIGHT,
    EXECUTE_LEFT,
    EXECUTE_180_TURN,
    EXECUTE_FORWARD,
    LACK_OF_PROGESS //va aqu[i??]
};

*/
#endif