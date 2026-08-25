#include <Arduino.h>

#include "gateway.h"

void setup() {
  run_hwt901b_gateway();  // Does not return
}

void loop() {
  // Not reached — run_hwt901b_gateway() runs its own event loop.
}
