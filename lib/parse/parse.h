#ifndef PARSE_H
#define PARSE_H

#include <stdint.h>

typedef struct {
    int valid_message;
    int start_cnt;
    uint8_t *start;
    uint8_t *end;
} msg_location_t; 

// take raw byte stream 
// assumes the first byte is the first byte in a message
// checks if this byte can be a valid length
// checks if there's a sync byte however many bytes away

msg_location_t find_message(uint8_t *array, uint8_t len, uint8_t next_seq);
uint16_t parse_crc(uint8_t* msg, uint8_t length);
uint16_t crc16(uint8_t *arr, uint8_t length);


#endif
