#ifndef KLIPPERCOMMANDER_H
#define KLIPPERCOMMANDER_H

#include "Arduino.h"
#include "compressed_dict.h"
#include "fifo.h"
#include <stdint.h>
#include "MotionQueue.h"
#include "trsync.h"
#include "EndStop.h"
#include "vlq.h"
#include "parse.h"

#ifdef USE_TINYUSB
#include <Adafruit_TinyUSB.h>
#endif

#define DEBUG_P 0
#define DEBUG_PRINTF if (DEBUG_P) Serial.printf
#define DEBUG_PRINT if (DEBUG_P) Serial.print
#define DEBUG_PRINTLN if (DEBUG_P) Serial.println

#define MAX_ENDSTOPS 4
#define MAX_TRSYNCS 10

#define COMMAND_QUEUE_LEN 64
#define SEND_QUEUE_LEN 64
#define MAX_MESSAGE_LEN 64
#define MIN_MESSAGE_LEN 5
#define MSG_HEADER_LEN 2
#define MESSAGE_DATA_LEN (MAX_MESSAGE_LEN-MIN_MESSAGE_LEN)
#define SUMSQ_BASE 256
#define SYNC_BYTE 0x7e


struct ObjID {
    bool allocated = false;
    uint32_t oid = 0;
};

class KlipperCommander {
	public:
		void handle();
		// KlipperCommander(SerialUART Serial);
		

		// #ifdef USE_TINYUSB
		// 	explicit KlipperCommander(Adafruit_USBD_CDC &Serial);
		// #else
		explicit KlipperCommander(Stream &Serial);
			// KlipperCommander(Stream &Serial, uint32_t (*clock)(void));
        
		// #endif


		void attach(float &position, float &velocity_ff, float &torque_ff);
		void attach(float &position, float &velocity_ff);
		void attach(float &position);

		void recieve_serial();
		void parse_message(msg_location_t msg);
		void update_stats(uint32_t current_time);

		void send_serial();

        uint32_t (*clock)(void) = NULL;
        MotionQueue move_queue;
        // MotionQueue motor_queues[3];


        TrSync trsync_objs[MAX_TRSYNCS];
        EndStop endstop_objs[MAX_TRSYNCS];

        ObjID stepper_obj;
        // ObjID trsync_obj;
        // ObjID endstop_obj;
        uint8_t is_config = 0; 
        uint8_t num_oids = 0;
		int32_t sum_total_steps = 0;
	private:
		#ifdef USE_TINYUSB
			Adafruit_USBD_CDC &serial;
		#else
			Stream &serial;
		#endif

		uint8_t in_buf[256];
		size_t in_buf_idx = 0;
		FIFO outgoing_fifo;

		void enqueue_response(uint8_t sequence, const uint8_t* msg, uint8_t length);
		void enqueue_config_response(uint8_t sequence, uint32_t offset, const uint8_t* msg, uint8_t length);
		// void ACK(uint8_t sequence);
		// void NACK(uint8_t sequence);
		void ACKNACK(uint8_t sequence);

		int32_t command_dispatcher(uint32_t cmd_id,uint8_t sequence, uint8_t *msg, int16_t length);

		uint16_t crc16(uint8_t *arr, uint8_t length);

		uint16_t parse_crc(uint8_t* msg, uint8_t length);

		uint32_t current_time = 0;
		uint32_t loop_start_time = 0;
		uint32_t prev_stats_send = 0;
		uint32_t prev_stats_send_high = 0;
		uint32_t stats_loop_count = 0;
		uint32_t stats_sum = 0;
		uint32_t stats_sumsq = 0;
        uint32_t prev_bytes_remaining = 0;
		uint8_t latest_outgoing_sequence = 0x10 | 0x00;
        uint8_t next_seq = 0x10 | 0x01;
		// uint8_t output_buffer[SEND_QUEUE_LEN][64];
		// uint8_t out_buf_write_idx = 0;
		// uint8_t out_buf_send_idx = 0;

		CONFIG_DICT
		uint32_t host_config_crc = 0;
		//command handlers below
		void send_config(uint8_t sequence, uint32_t offset, uint32_t amount) ;
};


void print_byte_array(uint8_t* arr, uint8_t len);



#endif
