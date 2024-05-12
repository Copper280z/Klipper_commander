#ifndef STM32G4xx
#include "Arduino.h"

#include "KlipperCommander.h"
#include "pins_arduino.h"
#include "stdint.h"


HardwareSerial Serial2 = HardwareSerial(USART2);

KlipperCommander k_commander = KlipperCommander(Serial2);

bool led_state;
uint32_t led_timer;
uint32_t print_timer;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(PB0, OUTPUT);
  pinMode(PB1, OUTPUT);
  pinMode(PB2, OUTPUT);
  pinMode(PB3, OUTPUT);
  Serial.begin(250000);
  Serial2.setTx(PA2);
  Serial2.setRx(PA3);
  Serial2.begin(250000);

  digitalWrite(LED_BUILTIN, LOW);
  led_state = false;
  led_timer = micros();
  // while (!Serial.available()){
  //   if (micros() - led_timer > (uint32_t) 1e6) {
  //     led_state = !led_state;

  //     digitalWrite(LED_BUILTIN,led_state);
  //     led_timer = micros();
  //   }
  //   delay(100);
  // }
  // Serial.println("Hello Klipper!");
}

void loop() {
    digitalWrite(PB0, HIGH);
    // delayMicroseconds(2);

    k_commander.handle();
    
    if (micros() - led_timer > (uint32_t) 1e6) {
        led_state = !led_state;

        digitalWrite(LED_BUILTIN,led_state);
        led_timer = micros();
    }
    
    if (micros() - print_timer > (uint32_t) 5e5) {
      // float pos = k_commander.move_queue.position;
      // float velocity = k_commander.move_queue.velocity;
      // float accel = k_commander.move_queue.accel;

      // Serial.printf("pos: %.3f - vel: %.3f - accel: %.3f\n",pos,velocity,accel);
      print_timer = micros();
    }
    digitalWrite(PB0, LOW);
}

#endif

#ifdef STM32G4xx
#include "Arduino.h"

#include "KlipperCommander.h"
#include "pins_arduino.h"
#include "stdint.h"

#include <SPI.h>
#include <SimpleFOC.h>
#include <SimpleFOCDrivers.h>
#include "encoders/mt6701/MagneticSensorMT6701SSI.h"
#include "encoders/smoothing/SmoothingSensor.h"
#include "stm32g431xx.h"
#include "drv_reset.h"
#include "aioli-board.h"
#include "drv_reset.c"

// Motor specific parameters.
#define POLEPAIRS 7
#define RPHASE 5.3
#define MOTORKV 140

uint8_t useDFU = 0;


#ifdef LED_BUILTIN
#undef LED_BUILTIN
#define LED_BUILTIN USER_LED
#endif
HardwareSerial Serial2 = HardwareSerial(USART2);

KlipperCommander k_commander = KlipperCommander(Serial2);

// simpleFOC constructors
BLDCDriver3PWM driver = BLDCDriver3PWM(U_PWM, V_PWM, W_PWM, U_EN, V_EN, W_EN);
BLDCMotor motor = BLDCMotor(POLEPAIRS, RPHASE, MOTORKV);
MagneticSensorMT6701SSI enc = MagneticSensorMT6701SSI(ENC_CS);
// SmoothingSensor enc = SmoothingSensor(encoder, motor);
// Commander commander = Commander(Serial);
bool led_state;
uint32_t led_timer;
uint32_t print_timer;

uint8_t configureFOC(void);
uint8_t configureCAN(void);
uint8_t configureDFU(void);



void setup()
{
  pinMode(USER_LED, OUTPUT);
	pinMode(USER_BUTTON, INPUT);
	
	if (digitalRead(USER_BUTTON) == HIGH){
		jump_to_bootloader();
	}
  // pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(250000);
  Serial2.setTx(PA2);
  Serial2.setRx(PA3);
  Serial2.begin(250000);  // Serial1.begin(250000);

  digitalWrite(USER_LED, LOW);

	configureFOC();
	configureCAN();
	configureDFU() ;

}

void loop()
{
  k_commander.handle();
	motor.loopFOC();
	motor.move(k_commander.move_queue.position);
	// commander.run();

    
    if (micros() - led_timer > (uint32_t) 1e6) {
        led_state = !led_state;

        digitalWrite(USER_LED,led_state);
        led_timer = micros();
    }
    
    if (micros() - print_timer > (uint32_t) 5e5) {
      float pos = k_commander.move_queue.position;
      float velocity = k_commander.move_queue.velocity;
      float accel = k_commander.move_queue.accel;

      Serial.printf("pos: %.3f - vel: %.3f - accel: %.3f\n",pos,velocity,accel);
      print_timer = micros();
    }
}

uint8_t configureFOC(){
	// commander.add('M', doMotor, "motor");
	// commander.verbose = VerboseMode::machine_readable;
	
	#ifdef SIMPLEFOC_STM_DEBUG
	SimpleFOCDebug::enable(&Serial);
	#endif

	// Encoder initialization.
	// Encoder on SPI1
	enc.init();

	// Driver initialization.
	driver.pwm_frequency = 32000;
	driver.voltage_power_supply = 5;
	driver.voltage_limit = 4.5;
	driver.init();

	// Motor PID parameters.
	motor.PID_velocity.P = 0.02;
	motor.PID_velocity.I = 0.5;
	motor.PID_velocity.D = 0.00;
	motor.voltage_limit = 4;
	motor.PID_velocity.output_ramp = 1000;
	motor.LPF_velocity.Tf = 1/(100*_2PI); // 1/(6.28*250);
	motor.LPF_angle.Tf = 1/(100*_2PI); // try to avoid

	// Motor initialization.
	motor.voltage_sensor_align = 2;
	motor.current_limit = 0.5;
	motor.velocity_limit = 20;
	motor.controller = MotionControlType::angle;
	motor.foc_modulation = FOCModulationType::SpaceVectorPWM;

	motor.sensor_offset = 2.43;
	motor.sensor_direction = Direction::CCW;

	motor.linkSensor(&enc);
	motor.linkDriver(&driver);
	motor.init();

	motor.target = 0;

	motor.initFOC();


	return 0;
}

uint8_t configureCAN(){
	return 0;
}

uint8_t configureDFU(){
	return 1;
}

#endif