#include <vector>
#include <Arduino.h>
#include "Obstacles.h"
SerialStub Serial, Serial2; uint32_t g_ms = 0;
struct Seg{ float len; float pitch; };
static void run(const char* name, std::vector<Seg> segs){
  ObstacleTracker t; float d=0; printf("--- %s\n", name);
  for(auto&s:segs) for(float x=0;x<s.len;x+=2){ d+=2; ObstacleEvent e=t.update(s.pitch,0,d); if(e) printf("   event %d at %.0f mm\n", e, d); }
}
int main(){
  run("speedbump (2.3deg x 25mm, gap 130, -2.3 x 25, flat)", {{50,0},{25,2.3f},{130,0},{25,-2.3f},{250,0}});
  run("stairs (1.9/3.8/1.9 x 200mm, plateau 50, down 200, flat)", {{50,0},{50,1.9f},{100,3.8f},{50,1.9f},{50,0},{50,-1.9f},{100,-3.8f},{50,-1.9f},{250,0}});
  run("ramp (15deg x155, plateau 150, -15 x155, flat)", {{50,0},{155,15},{150,0},{155,-15},{250,0}});
  run("vibration (2deg x 3mm)", {{100,0},{3,2},{200,0}});
}
