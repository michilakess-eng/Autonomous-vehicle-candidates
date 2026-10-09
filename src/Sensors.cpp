#include "Sensors.h"

//Encoders (un canal). Direccion por comando de motor.
namespace Enc {
  static volatile long ticksL= 0, ticksR= 0;
  static volatile int8_t dirL= 1, dirR= 1;
  static long baseL= 0, baseR= 0;

  static void IRAM_ATTR isrL() { ticksL+= dirL; }
  static void IRAM_ATTR isrR() { ticksR+= dirR; }

  void begin() {
    pinMode(ENC_LEFT_A, INPUT_PULLUP);
    pinMode(ENC_RIGHT_A, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENC_LEFT_A), isrL, FALLING);
    attachInterrupt(digitalPinToInterrupt(ENC_RIGHT_A), isrR, FALLING);
  }
  long left()  { noInterrupts(); long v= ticksL; interrupts(); return v - baseL; }
  long right() { noInterrupts(); long v= ticksR; interrupts(); return v - baseR; }
  void reset() { noInterrupts(); baseL= ticksL; baseR= ticksR; interrupts(); }
  float travelMm() { return 0.5f * (float)(left() + right()) * MM_PER_TICK; }
  void setDirection(int l, int r) {
    if(l!= 0) dirL= (l> 0)?1:-1;
    if(r!= 0) dirR= (r> 0)?1:-1;
  }
}

//IMU directo por registros (sin lib)
//yaw= integral de giro Z. pitch/roll= filtro complementario
static void imuWrite(uint8_t reg, uint8_t v) {
  Wire.beginTransmission(IMU_ADDR); Wire.write(reg); Wire.write(v); Wire.endTransmission();
}
static bool imuRead(uint8_t reg, uint8_t* buf, uint8_t n) {
  Wire.beginTransmission(IMU_ADDR); Wire.write(reg);
  if(Wire.endTransmission(false)!= 0) return false;
  if(Wire.requestFrom((uint8_t)IMU_ADDR, (uint8_t)n)!= n) return false;
  for(uint8_t i= 0; i< n; i++) buf[i]= Wire.read();
  return true;
}

bool Sensors::initIMU() {
  imuWrite(0x6B, 0x80); delay(100); //reset
  imuWrite(0x6B, 0x01); delay(50);  //wake, reloj PLL
  uint8_t who= 0;
  if(!imuRead(0x75, &who, 1) || who== 0x00 || who== 0xFF) return false;
  imuWrite(0x1A, 0x03); //gyro DLPF ~41Hz
  imuWrite(0x19, 0x00); //1kHz sample rate
  imuWrite(0x1B, 0x08); //gyro +-500 dps
  imuWrite(0x1C, 0x00); //accel +-2g
  imuWrite(0x1D, 0x03); //accel DLPF ~45Hz
  return true;
}

bool Sensors::readIMURaw(float& ax, float& ay, float& az, float& gx, float& gy, float& gz) {
  uint8_t b[14];
  if(!imuRead(0x3B, b, 14)) return false;
  auto s16= [&](int i) { return (int16_t)((b[i]<< 8) | b[i+1]); };
  ax= s16(0)/16384.0f; ay= s16(2)/16384.0f; az= s16(4)/16384.0f;
  gx= s16(8)/65.5f;    gy= s16(10)/65.5f;   gz= s16(12)/65.5f;
  return true;
}

void Sensors::calibrateGyro(uint32_t maxWaitMs) {
  uint32_t start= millis();
  while(true) {
    double sx= 0, sy= 0, sz= 0, sp= 0, sr= 0; int n= 0; float maxAbs= 0;
    uint32_t t0= millis();
    while(millis() - t0 < 600) {
      float ax, ay, az, gx, gy, gz;
      if(readIMURaw(ax, ay, az, gx, gy, gz)) {
        sx+= gx; sy+= gy; sz+= gz;
        sp+= atan2f(ax, sqrtf(ay*ay + az*az)) * 57.29578f;
        sr+= atan2f(ay, az) * 57.29578f;
        maxAbs= fmaxf(maxAbs, fmaxf(fabsf(gx), fmaxf(fabsf(gy), fabsf(gz))));
        n++;
      }
      delay(2);
    }
    //aceptar solo si robot esta inmovil
    if(millis() - start > maxWaitMs && n<= 50) break; //timeout o falla
    if(n> 50 && (maxAbs< 3.0f || millis() - start > maxWaitMs)) {
      biasGx= sx/n; biasGy= sy/n; biasGz= sz/n;
      pitch0= IMU_PITCH_SIGN * (float)(sp/n);
      roll0= IMU_ROLL_SIGN * (float)(sr/n);
      break;
    }
  }
  yawDeg= 0; pitchDeg= 0; rollDeg= 0; lastImuUs= micros();
}

void Sensors::updateIMU() {
  uint32_t now= micros();
  float dt= (now - lastImuUs) * 1e-6f;
  if(dt< 0.003f) return;
  lastImuUs= now;
  if(dt> 0.05f) dt= 0.05f;
  float ax, ay, az, gx, gy, gz;
  if(!readIMURaw(ax, ay, az, gx, gy, gz)) return;
  gx-= biasGx; gy-= biasGy; gz-= biasGz;

  yawRateDps= IMU_YAW_SIGN * gz;
  yawDeg+= yawRateDps * dt;

  float pitchRate= IMU_PITCH_SIGN * (-gy); //nose up pos
  float rollRate= IMU_ROLL_SIGN * gx;
  float accP= IMU_PITCH_SIGN * atan2f(ax, sqrtf(ay*ay + az*az)) * 57.29578f - pitch0;
  float accR= IMU_ROLL_SIGN * atan2f(ay, az) * 57.29578f - roll0;
  float mag= sqrtf(ax*ax + ay*ay + az*az);
  if(fabsf(mag - 1.0f)< 0.12f) { //sin accelerar, confiar en gravedad 2%
    pitchDeg= IMU_ALPHA * (pitchDeg + pitchRate*dt) + (1.0f - IMU_ALPHA)*accP;
    rollDeg= IMU_ALPHA * (rollDeg + rollRate*dt) + (1.0f - IMU_ALPHA)*accR;
  } else {
    pitchDeg+= pitchRate*dt;
    rollDeg+= rollRate*dt;
  }
}

//ToF boot secuencial
uint8_t Sensors::initToF() {
  const int xs[3]= {TOF_LEFT_XSHUT, TOF_FRONT_XSHUT, TOF_RIGHT_XSHUT};
  const uint8_t addr[3]= {TOF_ADDR_L, TOF_ADDR_F, TOF_ADDR_R};
  Adafruit_VL53L0X* t[3]= {&tofL, &tofF, &tofR};
  for(int i= 0; i< 3; i++) { pinMode(xs[i], OUTPUT); digitalWrite(xs[i], LOW); }
  delay(15);
  uint8_t failed= 0;
  for(int i= 0; i< 3; i++) {
    digitalWrite(xs[i], HIGH); delay(15);
    if(!t[i]->begin(addr[i], false, &Wire)) failed|= (1<<i);
    else t[i]->startRangeContinuous(33);
  }
  return failed;
}

static uint16_t tofRead(Adafruit_VL53L0X& t, uint16_t last, bool& fresh) {
  if(t.isRangeComplete()) {
    uint16_t r= t.readRangeResult();
    fresh= true;
    return (r>= TOF_FAR_MM)?TOF_FAR_MM:r;
  }
  return last;
}

void Sensors::pollToF() {
  if(millis() - lastTofMs < 8) return;
  lastTofMs= millis();
  bool fL= false, fF= false, fR= false;
  dL= tofRead(tofL, dL, fL);
  dF= tofRead(tofF, dF, fF);
  dR= tofRead(tofR, dR, fR);
  if(fF) tofFrames++;
}

ToF Sensors::avgToF(uint8_t frames) {
  uint32_t target= tofFrames + frames, t0= millis();
  uint32_t sl= 0, sf= 0, sr= 0; uint8_t n= 0; uint32_t seen= tofFrames;
  while(tofFrames< target && millis() - t0 < 500) {
    update();
    if(tofFrames!= seen) { seen= tofFrames; sl+= dL; sf+= dF; sr+= dR; n++; }
    delay(1);
  }
  if(n== 0) return ToF{dL, dF, dR};
  return ToF{(uint16_t)(sl/n), (uint16_t)(sf/n), (uint16_t)(sr/n)};
}

//RGB por I2C multiplexer
void Sensors::tcaSelect(uint8_t ch) {
  Wire.beginTransmission(TCA_ADDR); Wire.write(1<<ch); Wire.endTransmission();
}

uint8_t Sensors::initRGB() {
  Wire.beginTransmission(TCA_ADDR);
  if(Wire.endTransmission()!= 0) return 0x07; //mux fail
  uint8_t failed= 0;
  for(uint8_t i= 0; i< 3; i++) {
    tcaSelect(i);
    if(!rgb[i].begin(0x29, &Wire)) failed|= (1<<i);
  }
  return failed;
}

RGBRaw Sensors::readRGB(uint8_t ch) {
  tcaSelect(ch);
  RGBRaw v;
  rgb[ch].getRawData(&v.r, &v.g, &v.b, &v.c);
  return v;
}

void Sensors::hsv(const RGBRaw& v, float& h, float& s, float& val) {
  float r= v.r * RGB_GAIN_R, g= v.g * RGB_GAIN_G, b= v.b * RGB_GAIN_B;
  float mx= fmaxf(r, fmaxf(g, b)), mn= fminf(r, fminf(g, b)), d= mx - mn;
  val= mx; s= (mx> 0)?d/mx:0; h= 0;
  if(d> 0) {
    if(mx== r) h= 60.0f * fmodf((g - b)/d, 6.0f);
    else if(mx== g) h= 60.0f * ((b - r)/d + 2.0f);
    else h= 60.0f * ((r - g)/d + 4.0f);
    if(h< 0) h+= 360.0f;
  }
}

FloorColor Sensors::classify(const RGBRaw& v) {
  if(v.c< RGB_BLACK_MAX_CLEAR) return COL_BLACK;
  float h, s, val; hsv(v, h, s, val);
  if(s< RGB_SAT_WHITE_MAX) return (v.c>= RGB_WHITE_MIN_CLEAR)?COL_WHITE:COL_NONE;
  if(h>= HUE_RED_MIN || h< HUE_RED_MAX) return COL_RED;
  if(h< HUE_ORANGE_MAX) return COL_ORANGE;
  if(h< HUE_YELLOW_MAX) return COL_YELLOW;
  if(h< HUE_GREEN_MAX) return COL_GREEN;
  if(h< HUE_CYAN_MAX) return COL_CYAN;
  if(h>= HUE_MAGENTA_MIN) return COL_MAGENTA;
  return COL_NONE;
}

//IR
bool Sensors::frontLine() {
  return digitalRead(IR_INNER_FRONT)== IR_DIGITAL_BLACK ||
         analogRead(IR_OUTER_LEFT)>= IR_BLACK_ANALOG_MIN ||
         analogRead(IR_OUTER_RIGHT)>= IR_BLACK_ANALOG_MIN;
}
bool Sensors::backLine() { return digitalRead(IR_BACK_CORNER)== IR_DIGITAL_BLACK; }

//UART Pi ArUco
void Sensors::pollPi() {
  while(Serial2.available()) {
    char ch= (char)Serial2.read();
    if(ch== '\n') {
      piBuf[piLen]= 0; piLen= 0;
      if(piBuf[0]== 'A' && piBuf[1]== ',') {
        int id= atoi(piBuf + 2);
        if(id>= 0 && id< 50 && arucoLocked< 0) {
          if(id== arucoCand) { if(++arucoCount>= 2) { arucoLocked= id; arucoFresh= true; } }
          else { arucoCand= id; arucoCount= 1; }
        }
      }
    } else if(ch!= '\r' && piLen< sizeof(piBuf) - 1) {
      piBuf[piLen++]= ch;
    }
  }
}

//Main HAL Begin
uint8_t Sensors::begin() {
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000); //bajar a 100000 si level shifter falla
  pinMode(IR_INNER_FRONT, INPUT);
  pinMode(IR_BACK_CORNER, INPUT);
  pinMode(IR_OUTER_LEFT, INPUT);
  pinMode(IR_OUTER_RIGHT, INPUT);
  Enc::begin();
  Serial2.begin(PI_BAUD, SERIAL_8N1, PI_RX, PI_TX);

  uint8_t fail= 0;
  if(!initIMU()) fail|= 0x01;
  fail|= (uint8_t)(initToF()<< 1);
  fail|= (uint8_t)(initRGB()<< 4);
  lastImuUs= micros();
  return fail;
}

void Sensors::update() {
  updateIMU();
  pollToF();
  pollPi();
}