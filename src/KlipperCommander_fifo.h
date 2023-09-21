#ifndef KLIPPERCOMMANDER_H
#define KLIPPERCOMMANDER_H

#include "Arduino.h"
#include "compressed_dict.h"
#include "fifo.h"

#ifdef USE_TINYUSB
#include <Adafruit_TinyUSB.h>
#endif

#define COMMAND_QUEUE_LEN 64
#define SEND_QUEUE_LEN 64
#define MAX_MESSAGE_LEN 64
#define MIN_MESSAGE_LEN 5
#define MESSAGE_DATA_LEN (MAX_MESSAGE_LEN-MIN_MESSAGE_LEN)

#define SYNC_BYTE 0x7e

struct VarInt {
	uint32_t value;
	uint8_t length;
};

class KlipperCommander {
	public:
		void handle();
		// KlipperCommander(SerialUART Serial);
		

		#ifdef USE_TINYUSB
			KlipperCommander(Adafruit_USBD_CDC &Serial);
		#else
			KlipperCommander(HardwareSerial &Serial);
		#endif


		void attach(float &position, float &velocity_ff, float &torque_ff);
		void attach(float &position, float &velocity_ff);
		void attach(float &position);

		void recieve_serial();
		void parse_message();

		void send_serial();

	private:
		#ifdef USE_TINYUSB
			Adafruit_USBD_CDC &serial;
		#else
			HardwareSerial &serial;
		#endif

		FIFO incoming_fifo;
		FIFO outgoing_fifo;

		void enqueue_response(uint8_t sequence, uint8_t* msg, uint8_t length);
		void enqueue_config_response(uint8_t sequence, uint32_t offset, uint8_t* msg, uint8_t length);
		void ACK(uint8_t sequence);
		void NACK(uint8_t sequence);

		int32_t command_dispatcher(uint32_t cmd_id,uint8_t sequence, uint8_t *msg, uint8_t length);

		uint16_t crc16(uint8_t *arr, uint8_t length);
		VarInt parse_vlq_int(uint8_t* bytes, uint8_t length);

		uint8_t encode_vlq_int(uint8_t* p, uint32_t value);

		uint16_t parse_crc(uint8_t* msg, uint8_t length);


		// uint8_t output_buffer[SEND_QUEUE_LEN][64];
		// uint8_t out_buf_write_idx = 0;
		// uint8_t out_buf_send_idx = 0;

		CONFIG_DICT

		//command handlers below
		void send_config(uint8_t sequence, uint32_t offset, uint32_t amount) ;

};

void print_byte_array(uint8_t* arr, uint8_t len);



#endif