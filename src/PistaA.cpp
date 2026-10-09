#include "PistaA.h"
#include "Robot.h"
#include <string.h>

/*Laberinto 5x5 en matriz 9x9. Inicio en 4,4
Bits: 0-3 pared N,E,S,W
4-7 pared no vista por cam
8 visitado
9-11 terreno
12-14 color id 
15 scored*/
#include "PistaA.h"
#include "Robot.h"
#include <string.h>

//Map helpers
void PistaA::setWall(int cx, int cy, int d, bool on) {
  if(on) m[cy][cx]|= (1<< d); else m[cy][cx]&= ~(1<< d);
  int nx= cx + DX[d], ny= cy + DY[d], od= (d + 2) & 3; //doble via
  if(inBounds(nx, ny)) { if(on) m[ny][nx]|= (1<< od); else m[ny][nx]&= ~(1<< od); }
}

bool PistaA::inBox(int cx, int cy) const { //limite 5x5
  return (max(xmax, cx) - min(xmin, cx)<= 4) && (max(ymax, cy) - min(ymin, cy)<= 4);
}

uint32_t PistaA::remainingMs() const {
  uint32_t el= millis() - t0;
  return el>= ROUND_MS?0:ROUND_MS - el;
}

//Sensing y giros
void PistaA::senseCell(bool /*firstTime*/) {
  bool first= !(m[y][x] & VISITED);
  ToF t= sensors.avgToF(4); //inmovil centro
  setWall(x, y, dir, t.front< WALL_FRONT_MM);
  setWall(x, y, (dir + 3) & 3, t.left< WALL_SIDE_MM);
  setWall(x, y, (dir + 1) & 3, t.right< WALL_SIDE_MM);
  if(first) for(int d= 0; d< 4; d++) if(wall(x, y, d)) m[y][x]|= (1<< (4 + d)); //camara no ha visto esta pared
  m[y][x]|= VISITED;
  Serial.printf("[A] cell(%d,%d) dir=%d walls=%X L%u F%u R%u\n", x, y, dir, m[y][x] & 15, t.left, t.front, t.right);
}

void PistaA::scanBackAtStart() { //unico lado sin explorar en inicio
  float h= headingOf(dir);
  motion.turnTo(h + 180.0f);
  ToF t= sensors.avgToF(4);
  setWall(x, y, (dir + 2) & 3, t.front< WALL_FRONT_MM);
  motion.turnTo(h);
}

void PistaA::faceDir(uint8_t d) {
  if(d!= dir) { motion.turnTo(headingOf(d)); dir= d; }
}

//Step logica
MoveResult PistaA::step(uint8_t d, bool allowRed, bool readColor) {
  faceDir(d);
  int ox= x, oy= y;
  MoveOpts o; o.heading= headingOf(d); o.allowRed= allowRed; o.readColor= readColor;
  MoveResult r= nav.moveOneCell(o);
  
  if(r.abortedRed) { //regreso a origen
    redKnown= true; redX= x + DX[d]; redY= y + DY[d];
    setTerrain(redX, redY, T_RED); setWall(x, y, d, false);
    display.status("RED FOUND", "avoiding");
    Serial.printf("[A] red tile en (%d,%d)\n", redX, redY);
    return r;
  }
  
  if(!r.ok) { setWall(x, y, d, true); Serial.println("[A] mov fallo -> marca pared"); return r; }
  commit(r, d, ox, oy);
  return r;
}

void PistaA::scoreColor(int cx, int cy, FloorColor c) {
  if(!isScoringColor(c) || !inBounds(cx, cy) || (m[cy][cx] & SCORED)) return;
  m[cy][cx]|= SCORED | ((uint16_t)(c - COL_CYAN + 1)<< 12);
  colorsFound++;
  display.hold("COLOR:", colorName(c), 3000);
}

void PistaA::commit(const MoveResult& r, uint8_t d, int ox, int oy) {
  int fx= ox + DX[d], fy= oy + DY[d]; //celda inicial pisada (color muestra)
  m[oy][ox]&= ~(1<< (4 + ((d + 1) & 3))); //camara vio pared der origen
  for(int i= 1; i<= r.cells; i++) {
    setWall(x, y, d, false);
    x+= DX[d]; y+= DY[d];
    xmin= min(xmin, x); xmax= max(xmax, x); ymin= min(ymin, y); ymax= max(ymax, y);
    if(i< r.cells) { //zonas ciegas rampa/stairs
      m[y][x]|= VISITED;
      setTerrain(x, y, r.ramp?T_RAMP:T_STAIRS);
      setWall(x, y, (d + 1) & 3, true); setWall(x, y, (d + 3) & 3, true);
    }
    if(trailLen< 255) { trailX[trailLen]= x; trailY[trailLen]= y; trailLen++; }
  }
  if(r.ramp) rampsDone++;
  if(r.stairs) stairsDone++;
  bumpHalves+= r.bumpHalves;
  if(r.cells== 1 && r.bumpHalves) setTerrain(x, y, T_BUMP);

  scoreColor(fx, fy, r.color);
  if(r.cells> 1) scoreColor(x, y, r.colorEnd); //aterrizaje
  
  m[y][x]&= ~(1<< (4 + ((d + 1) & 3)));
  if(!(m[y][x] & VISITED)) senseCell(true);
  Serial.printf("[A] en(%d,%d) colores=%d bumps=%d stairs=%d ramp=%d t=%lus\n",
                x, y, colorsFound, bumpHalves, stairsDone, rampsDone, (unsigned long)((millis() - t0)/1000));
}

//Pathfinding BFS
void PistaA::bfs(bool allowRed, Bfs& o) {
  memset(o.dist, -1, sizeof(o.dist));
  memset(o.first, -1, sizeof(o.first));
  uint8_t qx[N * N], qy[N * N]; int qh= 0, qt= 0;
  o.dist[y][x]= 0; qx[qt]= x; qy[qt++]= y;
  
  while(qh< qt) {
    int cx= qx[qh], cy= qy[qh++];
    if(!(m[cy][cx] & VISITED) && !(cx== x && cy== y)) continue; //hoja inexplorada, stop
    for(int d= 0; d< 4; d++) {
      if(wall(cx, cy, d)) continue;
      int nx= cx + DX[d], ny= cy + DY[d];
      if(!inBounds(nx, ny) || !inBox(nx, ny)) continue;
      if(!allowRed && redKnown && nx== redX && ny== redY) continue;
      if(o.dist[ny][nx]>= 0) continue;
      
      o.dist[ny][nx]= o.dist[cy][cx] + 1;
      o.first[ny][nx]= (cx== x && cy== y)?d:o.first[cy][cx];
      qx[qt]= nx; qy[qt++]= ny;
    }
  }
}

bool PistaA::pickFrontier(int& tx, int& ty, int8_t& fd) {
  Bfs o; bfs(false, o);
  float best= 1e9f; bool found= false;
  for(int cy= 0; cy< N; cy++) for(int cx= 0; cx< N; cx++) {
    if((m[cy][cx] & VISITED) || o.dist[cy][cx]<= 0) continue;
    float cost= o.dist[cy][cx] + (o.first[cy][cx]== dir?0.0f:0.5f); //penalidad giro
    if(cost< best) { best= cost; tx= cx; ty= cy; fd= o.first[cy][cx]; found= true; }
  }
  return found;
}

bool PistaA::gotoCell(int tx, int ty) {
  for(int guard= 0; guard< 40 && !(x== tx && y== ty); guard++) {
    Bfs o; bfs(false, o);
    int8_t d= o.first[ty][tx];
    if(d< 0) return false;
    if(!step(d, false, false).ok) return false;
  }
  return x== tx && y== ty;
}

//Recorrido de camara paredes pendientes (pase pared por la der)
bool PistaA::scanTourStep() {
  Bfs o; bfs(false, o);
  int best= 1000, bx= -1, by= -1, bh= 0, wx= 0, wy= 0, ww= 0;
  for(int cy= 0; cy< N; cy++) for(int cx= 0; cx< N; cx++) {
    if(!(m[cy][cx] & VISITED)) continue;
    for(int w= 0; w< 4; w++) {
      if(!(m[cy][cx] & (1<< (4 + w)))) continue;
      int h= (w + 3) & 3, px= cx - DX[h], py= cy - DY[h];
      if(!canStopAt(cx, cy)) { m[cy][cx]&= ~(1<< (4 + w)); continue; } //stairs/ramp
      if(inBounds(px, py) && !canStopAt(px, py)) continue; //llegada no safe
      if(!inBounds(px, py) || !(m[py][px] & VISITED) || wall(px, py, h) || o.dist[py][px]< 0) {
        if(!inBounds(px, py) || !(m[py][px] & VISITED) || wall(px, py, h)) m[cy][cx]&= ~(1<< (4 + w)); //imposible scan
        continue;
      }
      if(o.dist[py][px]< best) { best= o.dist[py][px]; bx= px; by= py; bh= h; wx= cx; wy= cy; ww= w; }
    }
  }
  if(bx< 0) return false;
  display.status("PISTA A", "ArUco search");
  if(gotoCell(bx, by)) step(bh, false, false);
  m[wy][wx]&= ~(1<< (4 + ww)); //marcar
  return true;
}

bool PistaA::endgameDue() {
  if(!redKnown) return false; //no hay target rojo
  Bfs o; bfs(true, o);
  int dd= o.dist[redY][redX];
  if(dd< 0) return false;
  uint32_t need= (uint32_t)(dd * T_CELL_MS * ENDGAME_SAFETY);
  if(BONUS_RETURN) need+= (uint32_t)trailLen * T_BLIND_CELL_MS;
  return remainingMs()<= need;
}

void PistaA::sprintToRed() {
  display.status("SPRINT", "to RED");
  for(int guard= 0; guard< 60 && !(x== redX && y== redY); guard++) {
    Bfs o; bfs(true, o);
    int8_t d= o.first[redY][redX];
    if(d< 0) break;
    step(d, true, false);
  }
}

//Bonus 1: retorno sobre los pasos
void PistaA::returnToStart() {
  display.status("RETURNING", "bonus 1");
  int i= trailLen - 1;
  for(int guard= 0; i> 0 && guard< 200; guard++) {
    int tx= trailX[i - 1], ty= trailY[i - 1], d= -1;
    for(int k= 0; k< 4; k++) if(x + DX[k]== tx && y + DY[k]== ty) d= k;
    if(d< 0) break;
    faceDir((uint8_t)d);
    MoveOpts o; o.heading= headingOf((uint8_t)d); o.readColor= false; o.allowRed= true;
    MoveResult r= nav.moveOneCell(o);
    if(!r.ok) break;
    for(int k= 0; k< r.cells; k++) { x+= DX[d]; y+= DY[d]; }
    i-= r.cells;
    if(i< 0 || x!= trailX[i] || y!= trailY[i]) { Serial.println("[A] retorno desvio"); break; }
  }
}

void PistaA::run() {
  memset(m, 0, sizeof(m));
  t0= millis(); north0= sensors.yaw(); dir= 0; x= y= 4;
  xmin= xmax= ymin= ymax= 4;
  trailLen= 1; trailX[0]= 4; trailY[0]= 4;
  display.status("PISTA A", "exploring");

  scanBackAtStart();
  senseCell(true);

  //Fase 1 exploracion
  while(!endgameDue()) {
    int tx, ty; int8_t fd;
    if(!pickFrontier(tx, ty, fd)) break;
    step((uint8_t)fd, false, true);
  }
  //Fase 2 busqueda aruco muros ciegos
  if(USE_ARUCO)
    while(sensors.arucoId()< 0 && !endgameDue() && scanTourStep()) {}

  if(!redKnown) { motion.brake(); display.status("NO RED FOUND", "check map"); return; }

  //Fase 3 final
  sprintToRed();
  display.hold("FINISH", "red tile", 3000);
  if(BONUS_RETURN && remainingMs()> (uint32_t)trailLen * T_BLIND_CELL_MS) returnToStart();
  motion.brake();
  display.status("DONE", "");
}