#include "Arduino.h"

#include "KlipperCommander.h"

KlipperCommander k_commander = KlipperCommander(Serial);


void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);

}

void loop() {

  k_commander.handle();

}
