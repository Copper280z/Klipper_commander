#include "Arduino.h"

#include "KlipperCommander.h"
#include "pins_arduino.h"
#include "stdint.h"

HardwareSerial Serial2(USART2);

KlipperCommander k_commander = KlipperCommander(Serial);

bool led_state;
uint32_t led_timer;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(200000);
  Serial2.begin(2000000);

  digitalWrite(LED_BUILTIN, LOW);
  led_state = false;
  led_timer = micros();
  delay(1000);
}

void loop() {

    k_commander.handle();
    
    if (micros() - led_timer > (uint32_t) 1e6) {
        led_state = !led_state;

        digitalWrite(LED_BUILTIN,led_state);
        led_timer = micros();
    }
}
