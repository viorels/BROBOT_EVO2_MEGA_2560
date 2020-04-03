#include <Arduino.h>
#include "utils.h"

bool wait_time(unsigned long &timer, int duration) {
  bool finished = false;
  
  if (millis() - timer > duration) {
    finished = true;
    timer = millis();
  }
  
  return finished;
}

static inline int8_t sign(float val) {
  if (val < 0) return -1;
  if (val == 0) return 0;
  return 1;
}

static inline bool isclose(float x, float y) {
  return fabs(x-y) < 0.001;
}
