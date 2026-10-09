#include "Display.h"

void Display::copy16(char* dst, const char* src) {
  uint8_t i= 0;
  for(; i< 16 && src[i]; i++) dst[i]= src[i];
  for(; i< 16; i++) dst[i]= ' '; //pad: evita lcd.clear() flicker
  dst[16]= 0;
}

void Display::begin() {
  lcd.init();
  lcd.backlight();
  status("Booting", "Keep still");
  update();
}

void Display::draw(const char* a, const char* b) {
  if(strcmp(a, cur0)!= 0) { lcd.setCursor(0, 0); lcd.print(a); strcpy(cur0, a); }
  if(strcmp(b, cur1)!= 0) { lcd.setCursor(0, 1); lcd.print(b); strcpy(cur1, b); }
}

void Display::status(const char* l0, const char* l1) { copy16(st0, l0); copy16(st1, l1); }

void Display::hold(const char* l0, const char* l1, uint32_t ms) {
  if(qCount== QN) { qHead= (qHead + 1) % QN; qCount--; } //descarta msg mas antiguo
  Msg& m= q[(qHead + qCount) % QN];
  copy16(m.a, l0); copy16(m.b, l1); m.ms= ms;
  qCount++;
}

void Display::update() {
  uint32_t now= millis();
  if(holding && (int32_t)(now - holdUntil)>= 0) holding= false;
  if(holding) return;
  
  if(qCount) {
    Msg& m= q[qHead];
    draw(m.a, m.b);
    holdUntil= now + m.ms; holding= true;
    qHead= (qHead + 1) % QN; qCount--;
  } else {
    draw(st0, st1);
  }
}