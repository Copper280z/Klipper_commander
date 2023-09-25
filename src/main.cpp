#include "Arduino.h"

#ifdef USE_TINYUSB
  #include "tusb_config_rp2040.h"
  #include <Adafruit_TinyUSB.h>
#endif

#include "KlipperCommander_fifo.h"

#define LED LED_BUILTIN

#ifdef USE_TINYUSB
  Adafruit_USBD_CDC USBSer1;
  KlipperCommander k_commander = KlipperCommander(USBSer1);
#else
  KlipperCommander k_commander = KlipperCommander(Serial);
#endif

void setup() {
  pinMode(LED, OUTPUT);
  Serial.begin(115200);

  #ifdef USE_TINYUSB
      // check to see if multiple CDCs are enabled
    if ( CFG_TUD_CDC < 2 ) {
      digitalWrite(LED, HIGH); // LED on for error indicator

      while(1) {
        Serial.printf("CFG_TUD_CDC must be at least 2, current value is %u\n", CFG_TUD_CDC);
        Serial.println("  Config file is located in Adafruit_TinyUSB_Arduino/src/arduino/ports/{platform}/tusb_config_{platform}.h");
        Serial.println("  where platform is one of: nrf, rp2040, samd");
        delay(1000);
      }
    }
    // initialize 2nd CDC interface
    USBSer1.begin(115200);

    while (!Serial || !USBSer1) {
      // if (Serial) {
      //   Serial.println("Waiting for other USB ports");
      // }

      delay(1);
    }

  #endif



}

int t0=0;
int i=0;
int dur=0;
void loop() {
    k_commander.recieve_serial();
    k_commander.parse_message();
    k_commander.send_serial();
    
    // dur = micros()-t0;
    // if (dur > 1e6) {
    //     Serial.printf("Loops per sec: %u\n", i);
    //     i=0;
    //     t0=micros();
    // }
    // i+=1;



}
