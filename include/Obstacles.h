#pragma once
#include <Arduino.h>
#include "Config.h"

//classifying obstacles through imu and distance tracked
//episode starts with tilt and continues depending on flat surface or pleateau  
//todo config tiene que ser configurado a los valores reales
enum ObstacleEvent: uint8_t { OBS_NONE, OBS_BUMP_HALF, OBS_STAIRS, OBS_RAMP_UP, OBS_RAMP_DOWN };

class ObstacleTracker{
public:
  void reset() { inEpisode= false; }

  ObstacleEvent update(float pitch, float roll, float travelMm) {
    float tilt= fmaxf(fabsf(pitch), fabsf(roll));
    if (!inEpisode) {
      if (tilt> TILT_ON_DEG) {
        inEpisode= true; startMm= lastTiltMm= travelMm;
        peakPitch= pitch; peakTilt= tilt;
      }
      return OBS_NONE;
    }
    if (tilt > TILT_OFF_DEG) {
      lastTiltMm= travelMm;
      if (fabsf(pitch) > fabsf(peakPitch)) peakPitch= pitch;
      peakTilt= fmaxf(peakTilt, tilt);
      return OBS_NONE;
    }
    if (fabsf(travelMm - lastTiltMm) < EPISODE_FLAT_END_MM) return OBS_NONE;

    inEpisode= false;
    float span= fabsf(lastTiltMm - startMm);
    ObstacleEvent ev;
    if (fabsf(peakPitch) >= RAMP_PEAK_DEG) ev= (peakPitch >= 0) ? OBS_RAMP_UP : OBS_RAMP_DOWN;
    else if (span >= BUMP_MAX_SPAN_MM) ev= OBS_STAIRS;
    else if (span < 6.0f) ev= OBS_NONE;  // vibracion ignore
    else ev= OBS_BUMP_HALF;
    Serial.printf("[OBS] ev=%d span=%.0fmm peakPitch=%.1f peakTilt=%.1f\n", ev, span, peakPitch, peakTilt);
    return ev;
  }

  bool active() const { return inEpisode; }
  //plateau
  bool bigActive(float travelMm) const {
    return inEpisode && (fabsf(peakPitch) >= RAMP_PEAK_DEG || fabsf(lastTiltMm - startMm) >= BUMP_MAX_SPAN_MM
                         || fabsf(travelMm - startMm) >= BUMP_MAX_SPAN_MM + EPISODE_FLAT_END_MM);
  }

private:
  bool inEpisode= false;
  float startMm= 0, lastTiltMm= 0, peakPitch= 0, peakTilt= 0;
};
