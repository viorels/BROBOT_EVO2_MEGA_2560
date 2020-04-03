#pragma once

#include <Arduino.h>

bool wait_time(unsigned long &timer, int duration);
extern int8_t sign(float val);
extern bool isclose(float a, float b);
