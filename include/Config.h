#ifndef CONFIG_H
#define CONFIG_H
//herein will lie the code with important variables and constants.
//Such as, pinout, the state of the robot, and the color-coded direction of Pista B
//pinout to be determined

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
    LACK_OF_PROGESs //va aqu[i??]
};

//colors of both Pistas. RGB sensors research due.
enum FloorColor{
    CYAN,
    YELLOW,
    ORANGE,
    MAGENTA,
    NONE
};

#endif