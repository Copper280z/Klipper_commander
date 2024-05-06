#include "parse.h"
#include "stdint.h"
#include "Arduino.h"
msg_location_t find_message(uint8_t *array, uint8_t len, uint8_t next_seq) {
    uint8_t start_b, sync_b;
    uint16_t msg_crc, crc;
    msg_location_t retval = msg_location_t{ParseError::NotEnoughBytes,0,0,0,0}; 

    // wait for more data
    if (len < 5) {
        return retval;
    }

    for (int i = 0; i<len; i++) {
        start_b = array[i];

        // illegal start byte, msg length can't be greater than 64 or less than 5
        if ((start_b > 64) || (start_b < 5)) {
           continue;
        }

        // end of message isn't here yet, return and try again later
        if ((start_b+i) > len) {
            retval.valid_message = ParseError::MsgIncomplete;
            retval.start_cnt = i;
            return retval;
        }
        
        sync_b = array[i+start_b-1];
        if (sync_b != 0x7e) {
            // Serial2.printf("No sync! i: %u\n",i);
            continue;
        }
        
        uint8_t msg_sequence = array[i+1];
        if ((msg_sequence & ~0x0f) != 0x10) {
            // Serial2.printf("seq high byte mismatch! i: %u\n",i);
            continue;
        }

        msg_crc = parse_crc(&array[i], start_b);
        crc = crc16(&array[i], start_b-3);
        if (crc != msg_crc) {
            // Serial2.printf("CRC mismatch! i: %u\n",i);
            continue;
        }

        if (msg_sequence != next_seq) {
            //NAK, drop everything in the buffer and wait for next, correct, message
            retval.valid_message = ParseError::WrongSequence;
            retval.start_cnt = i;
            retval.start = &array[i];
            retval.end = &array[i+start_b-1];
            retval.len = start_b;
            // Serial2.printf("seq low byte mismatch! i: %u\n",i);
            return retval;
        }

        retval.valid_message = ParseError::MsgValid;
        retval.start_cnt = i;
        retval.start = &array[i];
        retval.end = &array[i+start_b-1];
        retval.len = start_b;
        return retval;

    }
    retval.valid_message = -3;
    retval.start_cnt = len;
    return retval;

}

uint16_t parse_crc(uint8_t* msg, uint8_t length) {
    uint16_t crc = (uint16_t) *(msg+length-3) << 8 | (uint16_t) *(msg+length-2);
    return crc;
}

uint16_t crc16(uint8_t* arr, uint8_t length) {
    uint16_t crc = 0xFFFF;
    for (int i = 0; i < length; i++) {
        uint8_t b = *(arr+i) ^ (crc & 0xFF);
        b = b ^ (b << 4);
        crc = ((uint16_t)b << 8 | crc >> 8) ^ ((uint16_t)b >> 4) ^ ((uint16_t)b << 3);
    }
    return crc;
}
