#include <Wire.h>
#include "Adafruit_DRV2605.h"

Adafruit_DRV2605 drv;
uint8_t effect;

void setup() {
  Serial.begin(9600);

  if (!drv.begin()) {
    Serial.println("Could not find DRV2605");
    while (1) delay(10);
  }

  drv.selectLibrary(1);
  drv.setMode(DRV2605_MODE_INTTRIG);

}

// drill vibrations
// attachment being removed/added
// vise being fully squeezed - resistance

void loop() {
  // simulateDrill();
  // delay(5000);
  // attachments();
  // delay(5000);
  // viceResistance();
  // delay(5000);

}

void simulateDrill() {
  Serial.println("Drill");
  drv.setWaveform(0, 16);
  drv.setWaveform(1, 0);
  drv.go();
  delay(1000);
}

void attachments() {
  Serial.println("Attachment added!");
  drv.setWaveform(0, 17);
  drv.setWaveform(1, 0);
  drv.go();
  delay(10000);

  Serial.println("Attachment removed!");
  drv.setWaveform(0, 17);
  drv.setWaveform(1, 0);
  drv.go();
  delay(1000);
}

void viceResistance() {
  Serial.println("Vise");
  drv.setWaveform(0,47);
  drv.go();
  delay(100);
}
