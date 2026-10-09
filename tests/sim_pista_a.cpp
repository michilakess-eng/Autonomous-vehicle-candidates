// Host-side fuzz test of the REAL PistaA.cpp logic against simulated 5x5 mazes.
// Build: see tests/README.txt. Only the world (walls, moves, ToF) is simulated.
#include <vector>
#include <cstdio>
#include <cstdlib>
#define private public            // let the test read PistaA's counters
#include "Robot.h"
#include "PistaA.h"
#undef private

uint32_t g_ms = 0;
SerialStub Serial, Serial2; TwoWire Wire;
Sensors sensors; Display display; Motion motion; Navigator nav;

struct World {
  bool w[5][5][4];                 // true walls, N E S W
  int sx, sy, rx, ry;              // start, red
  FloorColor col[5][5];
  bool stairs[5][5];
  bool seen[5][5];
  int px, py, dir;                 // robot truth
  int moves, msPerMove; uint32_t tRed;
} W;
static const int DX4[4] = {0,1,0,-1}, DY4[4] = {1,0,-1,0};

static bool inb(int x,int y){ return x>=0&&x<5&&y>=0&&y<5; }
static void carve(int x,int y,int d){ int nx=x+DX4[d], ny=y+DY4[d]; W.w[x][y][d]=false; if(inb(nx,ny)) W.w[nx][ny][(d+2)&3]=false; }

static void build(unsigned seed, int msPerMove, bool withStairs) {
  srand(seed);
  for(int x=0;x<5;x++) for(int y=0;y<5;y++){ for(int d=0;d<4;d++) W.w[x][y][d]=true; W.col[x][y]=COL_WHITE; W.stairs[x][y]=false; W.seen[x][y]=false; }
  // random perfect maze (DFS) + extra openings (loops)
  bool vis[5][5]={}; std::vector<int> st; st.push_back(0); vis[0][0]=true;
  while(!st.empty()){
    int c=st.back(), x=c/5, y=c%5; int opts[4], n=0;
    for(int d=0;d<4;d++){ int nx=x+DX4[d], ny=y+DY4[d]; if(inb(nx,ny)&&!vis[nx][ny]) opts[n++]=d; }
    if(!n){ st.pop_back(); continue; }
    int d=opts[rand()%n]; carve(x,y,d); int nx=x+DX4[d], ny=y+DY4[d]; vis[nx][ny]=true; st.push_back(nx*5+ny);
  }
  for(int k=0;k<4;k++){ int x=rand()%5,y=rand()%5,d=rand()%4; if(inb(x+DX4[d],y+DY4[d])) carve(x,y,d); }
  W.sx=rand()%5; W.sy=rand()%5;
  // the real final tile is the far end of the maze: pick a dead end (exactly one opening)
  { std::vector<int> dead; for(int x=0;x<5;x++) for(int y=0;y<5;y++){ int open=0; for(int d=0;d<4;d++) if(!W.w[x][y][d]) open++; if(open==1 && !(x==W.sx&&y==W.sy)) dead.push_back(x*5+y); }
    if(dead.empty()){ W.rx=(W.sx+2)%5; W.ry=W.sy; } else { int c=dead[rand()%dead.size()]; W.rx=c/5; W.ry=c%5; } }
  FloorColor cs[4]={COL_CYAN,COL_YELLOW,COL_ORANGE,COL_MAGENTA};
  for(int k=0;k<4;k++){ int x,y; do{ x=rand()%5; y=rand()%5; }while((x==W.sx&&y==W.sy)||(x==W.rx&&y==W.ry)||W.col[x][y]!=COL_WHITE); W.col[x][y]=cs[k]; }
  if(withStairs){  // a straight corridor cell with closed sides: the robot cannot stop on it
    for(int tries=0;tries<200;tries++){
      int x=rand()%5,y=rand()%5; if((x==W.sx&&y==W.sy)||(x==W.rx&&y==W.ry)||W.col[x][y]!=COL_WHITE) continue;
      for(int d=0;d<2;d++){ bool straight=!W.w[x][y][d]&&!W.w[x][y][d+2]&&W.w[x][y][(d+1)&3]&&W.w[x][y][(d+3)&3];
        if(straight && inb(x+DX4[d],y+DY4[d]) && inb(x-DX4[d],y-DY4[d]) && !(x+DX4[d]==W.rx&&y+DY4[d]==W.ry) && !(x-DX4[d]==W.rx&&y-DY4[d]==W.ry)){ W.stairs[x][y]=true; tries=999; break; } }
    }
  }
  W.px=W.sx; W.py=W.sy; W.dir=0; W.moves=0; W.tRed=0; W.msPerMove=msPerMove; W.seen[W.px][W.py]=true; g_ms=0;
}

// ---- simulated hardware ----------------------------------------------------------------------
ToF Sensors::avgToF(uint8_t) {
  int d=W.dir; bool f=W.w[W.px][W.py][d], l=W.w[W.px][W.py][(d+3)&3], r=W.w[W.px][W.py][(d+1)&3];
  return ToF{(uint16_t)(l?60:400),(uint16_t)(f?60:500),(uint16_t)(r?60:400)};
}
bool Motion::turnTo(float h){ W.dir=((int)lroundf(h/90.0f))&3; return true; }
void Motion::brake(){}
MoveResult Navigator::moveOneCell(const MoveOpts& o){
  MoveResult R; if(++W.moves>3000){ printf("RUNAWAY\n"); exit(2); }
  g_ms += o.readColor ? W.msPerMove : (W.msPerMove*11/19);   // blind moves are faster
  int d=((int)lroundf(o.heading/90.0f))&3; if(d!=W.dir){ printf("heading/dir mismatch\n"); exit(3); }
  if(W.w[W.px][W.py][d]){ R.blocked=true; return R; }
  int nx=W.px+DX4[d], ny=W.py+DY4[d];
  if(!o.allowRed && nx==W.rx && ny==W.ry){ R.abortedRed=true; R.color=COL_RED; return R; }
  R.color=W.col[nx][ny]; R.ok=true; if(nx==W.rx&&ny==W.ry&&!W.tRed) W.tRed=g_ms; R.cells=1; W.seen[nx][ny]=true; W.px=nx; W.py=ny;
  if(W.stairs[nx][ny]){ int mx=nx+DX4[d], my=ny+DY4[d]; R.stairs=true; R.cells=2; W.px=mx; W.py=my; W.seen[mx][my]=true; R.colorEnd=W.col[mx][my]; }
  if(!o.readColor){ R.color=COL_NONE; R.colorEnd=COL_NONE; }
  return R;
}
// ----------------------------------------------------------------------------------------------

int main(int argc,char**argv){
  int fails=0, total=0, inTime=0; int msPerMove = argc>1 ? atoi(argv[1]) : 1900;
  unsigned only = argc>2 ? atoi(argv[2]) : 0;
  for(unsigned seed=1; seed<=300; seed++){ if(only && seed!=only) continue;
    build(seed, msPerMove, seed%2==0);
    PistaA a; a.run(); total++;
    int cov=0; for(int x=0;x<5;x++) for(int y=0;y<5;y++) cov+=W.seen[x][y];
    bool atStart = (W.px==W.sx && W.py==W.sy);
    if(W.tRed && W.tRed<=ROUND_MS) inTime++;
    bool full = (cov==25);
    bool redFound = a.redKnown;
    bool colorsOk = (a.colorsFound==4);
    if(!(full && redFound && colorsOk && atStart)){ /* details below */
      fails++; if(fails<=8) printf("seed %u: coverage %d/25 red=%d colors=%d/4 finalAtStart=%d stairs=%d moves=%d t=%lus trail=%d\n",
          seed,cov,redFound,a.colorsFound,atStart,(int)(seed%2==0),W.moves,(unsigned long)(g_ms/1000),a.trailLen);
    }
  }
  printf("PistaA fuzz (%d ms/move): %d/%d mazes fully OK (all cells covered, red found, 4 colours, returned to start)\n", msPerMove, total-fails, total);
  printf("   reached the red tile before the %us limit: %d/%d\n",(unsigned)(ROUND_MS/1000),inTime,total);
  return fails?1:0;
}
