#include "FrontHeadlights.h"

#include <Arduino.h>

FrontHeadlights::FrontHeadlights(const int pin) : _pin(pin) {
  pinMode(_pin, OUTPUT);
};

void FrontHeadlights::turn(int on) {
  digitalWrite(_pin, on);
};