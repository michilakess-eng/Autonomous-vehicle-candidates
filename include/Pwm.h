#pragma once
#include <Arduino.h>

//API LEDC varia entre core 2.x (canales) y 3.x (pines). No usar analogWrite()
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR>= 3
  inline void pwmAttach(int pin, int /*ch*/, int freq, int bits) { ledcAttach(pin, freq, bits); }
  inline void pwmWrite(int pin, int /*ch*/, uint32_t duty) { ledcWrite(pin, duty); }
#else
  inline void pwmAttach(int pin, int ch, int freq, int bits) { ledcSetup(ch, freq, bits); ledcAttachPin(pin, ch); }
  inline void pwmWrite(int /*pin*/, int ch, uint32_t duty) { ledcWrite(ch, duty); }
#endif

//Canales: 0-1 motores (20kHz), 2 servo (50Hz)
constexpr int PWM_CH_LEFT= 0, PWM_CH_RIGHT= 1, PWM_CH_SERVO= 2;