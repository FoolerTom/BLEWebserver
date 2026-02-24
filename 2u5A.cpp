// Ohne UVLO an D8

#include "SdFat.h"
#include "Adafruit_SPIFlash.h"
#include <bluefruit.h>

Adafruit_FlashTransport_QSPI flashTransport;

void setup() {
  /*pinMode(D8, INPUT_PULLUP);
  pinMode(D2, INPUT_PULLDOWN);
  pinMode(D3, INPUT_PULLDOWN);*/

  Bluefruit.begin();

  flashTransport.begin();
  flashTransport.runCommand(0xB9);
  flashTransport.end();

  sd_power_system_off();
}

void loop() {
  // nothing to do
}
