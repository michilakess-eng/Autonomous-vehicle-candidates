#ifndef CONFIG_H
#define CONFIG_H
//herein will lie the code with important variables and constants.
//Such as, pinout, the state of the robot, and the color-coded direction of Pista B
//UNDETERMINED pinout. discuss with electrical
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

//colors of both Pistas. RGB sensors research due.
enum FloorColor{
    CYAN,
    YELLOW,
    ORANGE,
    MAGENTA,
    NONE
};
*/
#endif