// Host test of the hue classifier with idealised raw values (real tiles need TEST-mode tuning).
#include "Sensors.h"
SerialStub Serial, Serial2; TwoWire Wire; uint32_t g_ms = 0;
int main() {
  Sensors s; struct T { const char* n; RGBRaw v; FloorColor want; } t[] = {
    {"red",     {2000, 300, 300, 2800}, COL_RED},     {"orange",  {2400, 1000, 250, 3800}, COL_ORANGE},
    {"yellow",  {2500, 2300, 300, 4800}, COL_YELLOW}, {"green",   {400, 2000, 700, 3000}, COL_GREEN},
    {"cyan",    {500, 2200, 2400, 4600}, COL_CYAN},   {"magenta", {2400, 700, 1800, 4500}, COL_MAGENTA},
    {"white",   {3000, 3000, 2900, 9000}, COL_WHITE}, {"black",   {40, 40, 40, 120}, COL_BLACK}};
  int bad = 0;
  for (auto& x : t) { FloorColor g = s.classify(x.v); if (g != x.want) { bad++; printf("MISMATCH %s -> %s\n", x.n, colorName(g)); } }
  printf("colour classifier: %d/8 ideal samples correct\n", 8 - bad); return bad;
}
