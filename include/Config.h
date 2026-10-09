#pragma once
#include <Arduino.h>

//UNDETERMINED pinout. discuss with electrical. I put numbers to visualize how the pinout is fairing.
//Technically, the h bridge can have the PWMA and PWMB wired together.
constexpr int M_LEFT_PWM= 25, M_LEFT_IN1= 26, M_LEFT_IN2= 27;   // left, A+B  
constexpr int M_RIGHT_PWM= 14, M_RIGHT_IN1= 13, M_RIGHT_IN2= 33;   // dercha A+B

constexpr int I2C_SDA= 21, I2C_SCL= 22;

constexpr int TOF_LEFT_XSHUT = 32, TOF_FRONT_XSHUT = 4, TOF_RIGHT_XSHUT = 5;

constexpr int IR_OUTER_LEFT= 34; 
constexpr int IR_OUTER_RIGHT= 35;   
constexpr int IR_INNER_FRONT= 36;   
constexpr int IR_BACK_CORNER= 39;  

constexpr int ENC_LEFT_A= 18, ENC_RIGHT_A= 19;
constexpr int SERVO_CLAW= 23;
constexpr int PI_RX= 16, PI_TX= 17;//uart2
constexpr long PI_BAUD= 115200;

constexpr int PIN_TRACK_SWITCH= 2;   // switch para det. pista. GND es PistaB, other es PistaA
constexpr int PIN_BUTTON= 15;  // Si en GND en boot, calibrar


const int speed_norm= 150;
const int speed_max= 255;
const int speed_turn= 130;

//I2C
constexpr uint8_t TOF_ADDR_L= 0x30, TOF_ADDR_F= 0x31, TOF_ADDR_R= 0x32;
constexpr uint8_t IMU_ADDR= 0x68, LCD_ADDR= 0x27, TCA_ADDR= 0x70;
constexpr uint8_t RGB_CH_LEFT= 0, RGB_CH_CENTER= 1, RGB_CH_RIGHT= 2;

// INVENTADO, VARIABLES REALES DEL ROBOT. 
constexpr float CELL_MM= 300.0f;
constexpr float SENSOR_AHEAD_MM= 135.0f;  // dist. centro rueda a barra de sensor
constexpr float REAR_IR_BEHIND_MM= 115.0f;  // dist. demtrp rueda a los IR traseros
constexpr float WHEEL_DIAMETER_MM= 40.0f;   
constexpr float ENC_TICKS_PER_REV= 300.0f;  //pulsos per wheel turn
constexpr float MM_PER_TICK= (WHEEL_DIAMETER_MM * 3.14159265f)/ENC_TICKS_PER_REV;

//speed, 255 max
constexpr int SPEED_CRUISE= 150, SPEED_MAX= 255, SPEED_PROBE= 90, SPEED_DESCENT= 110;
constexpr int SPEED_REVERSE= 110, SPEED_TURN_MAX= 130, SPEED_TURN_MIN= 60;
constexpr int PWM_FREQ= 20000;

//control
constexpr float HEADING_KP= 4.0f;//pwm grado error
constexpr float HEADING_KD= 0.5f; //pwm grado yaw r
constexpr float STEER_MAX= 120.0f;
constexpr float TURN_KP= 2.5f, TURN_TOL_DEG= 1.5f, TURN_LEAD_S= 0.05f;
constexpr float WALL_CENTER_KP= 0.03f;  
constexpr float WALL_CENTER_MAX= 4.0f;
constexpr float WALL_TARGET_MM= 50.0f; 
constexpr float BRAKE_COMP_MM= 10.0f;  //brake distance
constexpr bool  YAW_RESYNC= true;

//imu TIENE QUE SER: x forward. y left. z up.
constexpr float IMU_YAW_SIGN= -1.0f; 
constexpr float IMU_PITCH_SIGN= 1.0f;
constexpr float IMU_ROLL_SIGN = 1.0f;
constexpr float IMU_ALPHA= 0.98f;//giroscopio weight v acc

//obstaculos
constexpr float TILT_ON_DEG= 1.5f;
constexpr float TILT_OFF_DEG= 0.8f;
constexpr float RAMP_PEAK_DEG= 10.0f;
constexpr float EPISODE_FLAT_END_MM= 90.0f;//posible cambio dependiendo del chassis
constexpr float BUMP_MAX_SPAN_MM= 120.0f;

//ToF
constexpr uint16_t TOF_FAR_MM= 2000;
constexpr uint16_t WALL_SIDE_MM= 170, WALL_FRONT_MM= 200, OPEN_MM= 250, FRONT_STOP_MM= 40;
constexpr int IR_BLACK_ANALOG_MIN= 3200;//black
constexpr int IR_WHITE_ANALOG_MAX= 700;//not black likely white
constexpr int IR_DIGITAL_BLACK= HIGH;

//Color
constexpr float HUE_RED_MIN= 345, HUE_RED_MAX= 12, HUE_ORANGE_MAX= 38, HUE_YELLOW_MAX= 75;
constexpr float HUE_GREEN_MAX= 165, HUE_CYAN_MAX= 215, HUE_MAGENTA_MIN= 290;
constexpr float RGB_SAT_WHITE_MAX= 0.20f;
constexpr uint16_t RGB_BLACK_MAX_CLEAR= 300, RGB_WHITE_MIN_CLEAR= 2000;
constexpr float RGB_GAIN_R= 1.0f, RGB_GAIN_G= 1.0f, RGB_GAIN_B= 1.0f;

//mm despues de tile
constexpr float COLOR_WIN_START_MM= 60.0f, COLOR_WIN_END_MM= 130.0f;
constexpr float LINE_MIN_SPACING_MM= 120.0f;
constexpr uint32_t RGB_PERIOD_MS= 30;

//round
constexpr uint32_t ROUND_MS= 360000;
constexpr uint32_t T_CELL_MS= 1900;//MEDIR tiempo estimado por cell
constexpr uint32_t T_BLIND_CELL_MS= 1100;//unexplored estimate
constexpr float ENDGAME_SAFETY= 1.5f;
constexpr bool BONUS_RETURN= true;//Pista A bonus
constexpr bool USE_ARUCO= true;//Pista A bonus 2

//Pista B
constexpr bool S2_USE_3RGB= true;
constexpr float S2_STEP_MM= 80.0f; //est.
constexpr float S2_REVERSE_MM= 20.0f;
constexpr float S1_BALL_APPROACH_MM= 150.0f; //centro a garra
constexpr int S1_EXIT_IDX= 5; //2,1 este medio
constexpr float S1_EXIT_HDG= 90.0f; //rumbo salida
constexpr float CLAW_OPEN_DEG= 10, CLAW_CLOSED_DEG= 130;
constexpr uint32_t CLAW_MOVE_MS= 450;

//others, helpers
enum FloorColor : uint8_t { COL_NONE, COL_WHITE, COL_BLACK, COL_GREEN, COL_RED,
                            COL_CYAN, COL_YELLOW, COL_ORANGE, COL_MAGENTA, COL_COUNT };

inline const char* colorName(FloorColor c) {
  switch (c) {
    case COL_WHITE: return "WHITE"; 
    case COL_BLACK: return "BLACK";
    case COL_GREEN: return "GREEN";
    case COL_RED: return "RED";
    case COL_CYAN: return "CYAN";
    case COL_YELLOW: return "YELLOW";
    case COL_ORANGE: return "ORANGE";
    case COL_MAGENTA: return "MAGENTA";
    default: return "NONE";
  }
}
inline bool isScoringColor(FloorColor c) {
  return c== COL_CYAN || c== COL_YELLOW || c== COL_ORANGE || c== COL_MAGENTA;
}
inline float wrap180(float a) { while (a > 180.0f) a -= 360.0f; while (a <= -180.0f) a += 360.0f; return a; } //normalizador
inline float clampf(float v, float lo, float hi) { return v < lo?lo: (v> hi?hi:v); }

//direcciones: N=0, E=1, S=2, W=3, N es +y
constexpr int8_t DX[4] = {0, 1, 0, -1};
constexpr int8_t DY[4] = {1, 0, -1, 0};
