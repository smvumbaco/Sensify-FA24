#include <Wire.h>
#include "Adafruit_DRV2605.h"

Adafruit_DRV2605 drv;
uint8_t effect = 16;

void setup() {
  Serial.begin(9600);
  Serial.println("Begin");
  if (! drv.begin()) {
    Serial.println("Could not find DRV2605");
    while (1) delay(10);
  }

  drv.selectLibrary(1);

  drv.setMode(DRV2605_MODE_INTTRIG);
}

void loop() {

  // 16 is basic 1000ms alert effect
  // need to figure out how to do conditionals - connect to drill?
  Serial.println("Effect");
  drv.setWaveform(0, effect);
  drv.setWaveform(1, 0);
  drv.go();
  delay(1000);
}
