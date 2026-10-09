#include <Arduino.h>
#include "Robot.h"
#include "PistaA.h"
#include "PistaB.h"

Sensors sensors;
Display display;
Motion motion;

static PistaA pistaA;

void tick() {
  sensors.update();
  display.update();
  if(sensors.takeArucoNew()) {
    char b[17]; snprintf(b, sizeof(b), "ID: %d", sensors.arucoId());
    display.hold("ARUCO MARKER", b, 3000);
  }
}

void waitMs(uint32_t ms) {
  uint32_t t0= millis();
  while(millis() - t0 < ms) { tick(); delay(1); }
}

//Modo prueba/calibracion (mantener boton en boot)
//Cmds serial: o/c garra, f/b 300mm, r/l giro 90, z cero yaw, h test color
static void testMode() {
  display.status("TEST MODE", "see Serial");
  Serial.println("TEST: f b r l z h");
  uint32_t last = 0;
  while (true) {
    tick();
    if (Serial.available()) {
      char c = Serial.read();
      float y = sensors.yaw();
      if (c == 'f') { Enc::reset(); motion.driveDistance(300, SPEED_CRUISE, y);
                           Serial.printf("moved: ticks L=%ld R=%ld  est=%.0f mm (measure the real distance!)\n", Enc::left(), Enc::right(), Enc::travelMm()); }
      else if (c == 'b') motion.driveDistance(-300, SPEED_REVERSE, y);
      else if (c == 'r') { motion.turnTo(y + 90); Serial.printf("yaw after turn: %.1f\n", sensors.yaw()); }
      else if (c == 'l') { motion.turnTo(y - 90); Serial.printf("yaw after turn: %.1f\n", sensors.yaw()); }
      else if (c == 'z') sensors.shiftYaw(-sensors.yaw());
      else if (c == 'h') {
        for (uint8_t ch = 0; ch < 3; ch++) {
          RGBRaw v = sensors.readRGB(ch); float h, s, val; sensors.hsv(v, h, s, val);
          Serial.printf("RGB%u r%u g%u b%u c%u  H%.0f S%.2f -> %s\n", ch, v.r, v.g, v.b, v.c, h, s, colorName(sensors.classify(v)));
        }
      }
    }
    if (millis() - last > 250) {
      last = millis();
      Serial.printf("yaw %.1f pitch %.1f roll %.1f | ToF L%u F%u R%u | IRfront %d IRback %d outer %d/%d | enc %ld/%ld | aruco %d\n",
                    sensors.yaw(), sensors.pitch(), sensors.roll(), sensors.left(), sensors.front(), sensors.right(),
                    sensors.frontLine(), sensors.backLine(), sensors.outerLeftAnalog(), sensors.outerRightAnalog(),
                    Enc::left(), Enc::right(), sensors.arucoId());
      char a[17], b[17];
      snprintf(a, sizeof(a), "Y%4.0f P%3.0f R%3.0f", sensors.yaw(), sensors.pitch(), sensors.roll());
      snprintf(b, sizeof(b), "%4u%4u%4u", sensors.left(), sensors.front(), sensors.right());
      display.status(a, b);
    }
    delay(1);
  }
}
void setup() {
  Serial.begin(115200);
  pinMode(PIN_BUTTON, INPUT_PULLUP);

  Wire.begin(I2C_SDA, I2C_SCL); //bus antes de LCD
  motion.begin();
  display.begin();

  uint8_t fail= sensors.begin();
  if(fail) {
    char b[17]; snprintf(b, sizeof(b), "fail mask %02X", fail);
    display.status("SENSOR ERROR", b);
    display.update();
    Serial.printf("Sensor init fail, mask=0x%02X (b0 IMU, b1-3 ToF, b4-6 RGB)\n", fail);
    if(fail & 0x0F) while(true) delay(100); //IMU o ToF fallaron: no arrancar
  }

  display.status("Calibrando", "NO MOVER");
  display.update();
  sensors.calibrateGyro(); //espera a que el robot este inmovil
  display.status("Ready", "");

  if(digitalRead(PIN_BUTTON)== LOW) testMode(); //loop infinito

  bool pistaB_selected= (digitalRead(PIN_TRACK_SWITCH)== LOW);
  Serial.printf("Track: %s\n", pistaB_selected?"B":"A");
  waitMs(300);

  pistaA.run();
  motion.brake();
}

void loop() {
  tick(); //ronda terminada: mantener LCD vivo
  delay(5);
}