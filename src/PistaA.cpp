#pragma once
#include <Arduino.h>
#include "Config.h"
#include "Navigator.h"

/*Laberinto 5x5 en matriz 9x9. Inicio en 4,4
Bits: 0-3 pared N,E,S,W
4-7 pared no vista por cam
8 visitado
9-11 terreno
12-14 color id 
15 scored*/
class PistaA {
public:
  void run();

private:
  static constexpr int N= 9;
  static constexpr uint16_t VISITED= 1<< 8, SCORED= 1<< 15;
  enum Terrain : uint8_t { T_FLAT, T_BUMP, T_STAIRS, T_RAMP, T_RED };

  struct Bfs { int8_t dist[N][N]; int8_t first[N][N]; };

  uint16_t m[N][N];
  int x= 4, y= 4; uint8_t dir= 0; float north0= 0;
  int xmin= 4, xmax= 4, ymin= 4, ymax= 4;
  bool redKnown= false; int redX= 0, redY= 0;
  int colorsFound= 0, bumpHalves= 0, stairsDone= 0, rampsDone= 0;
  uint8_t trailX[255], trailY[255]; int trailLen= 0;
  uint32_t t0= 0;

  //helpers mapa
  bool wall(int cx, int cy, int d) const { return m[cy][cx] & (1<< d); }
  void setWall(int cx, int cy, int d, bool on);
  bool inBounds(int cx, int cy) const { return cx>= 0 && cx< N && cy>= 0 && cy< N; }
  bool inBox(int cx, int cy) const;
  void setTerrain(int cx, int cy, Terrain t) { m[cy][cx]= (m[cy][cx] & ~(7<< 9)) | ((uint16_t)t<< 9); }
  float headingOf(uint8_t d) const { return north0 + 90.0f * d; }

  //comportamiento
  void senseCell(bool firstTime);
  void scanBackAtStart();
  void faceDir(uint8_t d);
  MoveResult step(uint8_t d, bool allowRed, bool readColor);
  void commit(const MoveResult& r, uint8_t d, int ox, int oy);
  void scoreColor(int cx, int cy, FloorColor c);
  bool canStopAt(int cx, int cy) const { uint8_t t= (m[cy][cx]>> 9) & 7; return t!= T_STAIRS && t!= T_RAMP; }
  void bfs(bool allowRed, Bfs& o);
  bool pickFrontier(int& tx, int& ty, int8_t& firstDir);
  bool gotoCell(int tx, int ty);
  bool scanTourStep();
  bool endgameDue();
  uint32_t remainingMs() const;
  void sprintToRed();
  void returnToStart();
};