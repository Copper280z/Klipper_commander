#include "KlipperCommander_fifo.h"

#ifdef USE_TINYUSB
    KlipperCommander::KlipperCommander(Adafruit_USBD_CDC &Serial) : serial(Serial) {
        serial = Serial;
    }
#else
    KlipperCommander::KlipperCommander(arduino::HardwareSerial &Serial) : serial(Serial) {
        serial = Serial;
    }
#endif

void KlipperCommander::recieve_serial() {
    Pointer curr_write =  incoming_fifo.getWritePointer();

    uint8_t bytes_read = serial.read(curr_write.ptr, curr_write.len);
    if (bytes_read == 0) return;
    print_byte_array(curr_write.ptr, bytes_read);

    uint8_t msg_length = incoming_fifo.currentWriteMsgGetByteAt(0);
    uint8_t msg_bytes_already_recvd = incoming_fifo.getCurrentWriteMsgLength();

    uint8_t bytes_available = msg_bytes_already_recvd + bytes_read;
    // Serial.printf("Now have %u bytes available to do stuff with\n", bytes_available);
    
    if ((bytes_available)  < msg_length){
        incoming_fifo.advanceWriteCursorN(bytes_read);
        return;
    }

    // if we have enough bytes to complete the message
    while ( bytes_available  >= msg_length && bytes_available != 0 ) {

        uint8_t msg_sync = incoming_fifo.currentWriteMsgGetByteAt(msg_length-1);
        uint8_t* msg_ptr = incoming_fifo.getCurrentWriteMsgStart();
        uint16_t msg_crc = parse_crc(msg_ptr, msg_length);
        uint16_t calc_crc = crc16(msg_ptr, msg_length-3);
        Serial.printf("msg length: 0x%x\n", msg_length);
        Serial.printf("msg sync:    0x%x\n", msg_sync);
        Serial.printf("calc'd crc: 0x%x\n", calc_crc);
        Serial.printf("msg crc:    0x%x\n", msg_crc);
        
        if (msg_sync == SYNC_BYTE && msg_crc == calc_crc && msg_length <= 64 && msg_length > 0) {
            Serial.println("Valid Message!");
            uint8_t sequence = incoming_fifo.currentWriteMsgGetByteAt(1);
            ACK(sequence);
            incoming_fifo.advanceWriteCursorN(msg_length);
            incoming_fifo.finalizeMessage();
            bytes_available -= msg_length;
            msg_length = incoming_fifo.currentWriteMsgGetByteAt(0);
        } else {
            Serial.println("Invalid Message!");
            print_byte_array(msg_ptr, bytes_available);
            // out of sync - look for a SYNC_BYTE and copy 
            // everything after that back to the start of current msg, then start over
            bool sync=false;
            for (int i=0;i<bytes_available;i++) {
                uint8_t curr_byte = incoming_fifo.currentWriteMsgGetByteAt(i);
                Serial.printf("0x%x\n", curr_byte);
                if (curr_byte == SYNC_BYTE) {
                    memcpy(msg_ptr, msg_ptr+i+1, bytes_available-i+1);
                    bytes_available -= i;
                    sync=true;
                    break;
                }
            }
            if (sync==false) {
                bytes_available = 0;
            }
        }
    }
}


void KlipperCommander::parse_message() {
    // only try to parse up to command_idx-1, command_idx may be incomplete
    uint8_t commands_to_parse = incoming_fifo.getNumMsgToRead();

    while (commands_to_parse > 0) {
        Serial.printf("\n%d commands in queue\n", commands_to_parse);
        Pointer read_ptr = incoming_fifo.getReadPointer();

        Serial.printf("Length of command being parsed: %d\n", read_ptr.len);

        for (int i=0; i<read_ptr.len;i++){
            Serial.print(*(read_ptr.ptr+i), HEX);
            Serial.print(" ");
        }
        Serial.print("\n");

        uint8_t sequence = *(read_ptr.ptr+1);
        Serial.printf("sequence low bytes: %u\n", sequence & 0b00001111 );

        int16_t command_bytes_available = read_ptr.len-MIN_MESSAGE_LEN;
        int32_t bytes_consumed=0;
        while (command_bytes_available>0) {
            Serial.printf("Command bytes available: %u\n", command_bytes_available);
            VarInt cmd_id_var = parse_vlq_int(read_ptr.ptr+2+bytes_consumed, command_bytes_available);
            Serial.printf("Command ID:  %u\n", (int32_t) cmd_id_var.value);
            bytes_consumed = command_dispatcher(cmd_id_var.value, sequence, read_ptr.ptr+2+bytes_consumed, command_bytes_available);
            if (bytes_consumed < 0) {
                Serial.printf("bytes_consumed is < 0: %u\n", bytes_consumed);
                // not sure what that command was, so maybe we might be lost
                // quit and try again next time
                // if this leg is ever taken, something is wrong, probably related to the data dict
                break;
            }
            command_bytes_available -= bytes_consumed;
        }
        incoming_fifo.advanceReadCursor();
        commands_to_parse = incoming_fifo.getNumMsgToRead();
    }
}

int32_t KlipperCommander::command_dispatcher(uint32_t cmd_id, uint8_t sequence, uint8_t *msg, uint8_t length) {
    uint8_t bytes_consumed = 1;
    switch (cmd_id) {
        case 1:{
            VarInt offset_var = parse_vlq_int((msg+1), length-1);
            VarInt amount_var = parse_vlq_int((msg+1+offset_var.length), length-1-offset_var.length);
            Serial.printf("Offset: %u - len: %u\n", (int32_t) offset_var.value, offset_var.length);
            Serial.printf("Amount: %u - len: %u\n", amount_var.value, amount_var.length);
            send_config(sequence, offset_var.value, amount_var.value);
            bytes_consumed = 1+offset_var.length+amount_var.length;
            break;
        }
        default:{
            bytes_consumed = -1;
            break;
        }
    }
    return bytes_consumed;
}

void KlipperCommander::send_serial(){
    uint8_t msgs_to_send = outgoing_fifo.getNumMsgToRead();
    if (msgs_to_send > 0) {
        Serial.printf("\nStarting serial send to host: %u messages\n", msgs_to_send);
    }
    for (int i=0; i<msgs_to_send; i++){
        Pointer read_ptr = outgoing_fifo.getReadPointer();
        Serial.printf("msg sent: ");
        print_byte_array(read_ptr.ptr, read_ptr.len);
        serial.write(read_ptr.ptr, read_ptr.len);
        outgoing_fifo.advanceReadCursor();
    }

}

void KlipperCommander::ACK(uint8_t sequence) {
    Pointer write_ptr = outgoing_fifo.getWritePointer();
    uint8_t new_sequence = ((sequence +1 ) & 0b00001111) | 0x10;
    // Serial.printf("ACK sequence high bytes: 0x%x\n", new_sequence & 0b11110000 );
    // Serial.printf("ACK sequence low bytes: %u\n", new_sequence & 0b00001111 );

    write_ptr.ptr[0] = 5;
    write_ptr.ptr[1] = new_sequence;
    uint16_t crc = crc16(write_ptr.ptr,2);
    write_ptr.ptr[2] = (uint8_t) (crc >> 8);
    write_ptr.ptr[3] = (uint8_t) (crc & 0xFF);
    write_ptr.ptr[4] = SYNC_BYTE;
    
    outgoing_fifo.advanceWriteCursorN(5);
    outgoing_fifo.finalizeMessage();

}

void KlipperCommander::NACK(uint8_t sequence) {
    Pointer write_ptr = outgoing_fifo.getWritePointer();
    uint8_t new_sequence = ((sequence -1 ) & 0b00001111) | 0x10;

    write_ptr.ptr[0] = 5;
    write_ptr.ptr[1] = new_sequence;
    uint16_t crc = crc16(write_ptr.ptr,2);
    write_ptr.ptr[2] = (uint8_t) (crc >> 8);
    write_ptr.ptr[3] = (uint8_t) (crc & 0xFF);
    write_ptr.ptr[4] = SYNC_BYTE;
    
    outgoing_fifo.advanceWriteCursorN(5);
    outgoing_fifo.finalizeMessage();
}

uint16_t KlipperCommander::crc16(uint8_t* arr, uint8_t length) {
    uint16_t crc = 0xFFFF;
    for (int i = 0; i < length; i++) {
        uint8_t b = *(arr+i) ^ (crc & 0xFF);
        b = b ^ (b << 4);
        crc = ((uint16_t)b << 8 | crc >> 8) ^ ((uint16_t)b >> 4) ^ ((uint16_t)b << 3);
    }
    return crc;
}

VarInt KlipperCommander::parse_vlq_int(uint8_t* bytes, uint8_t length) {
    uint32_t c = *(bytes);
    uint32_t v = 0; //c & 0x7F;

    //if vlq is negative(?)
    if ((c & 0x60) == 0x60) {
        v |= (uint32_t) ((int32_t) -0x20);
    }
    uint8_t j=0;
    for (int i=0; i<length; i++) {
        c = *(bytes+i);
        v = (v << 7) | (c & 0x7F);
        j+=1;

        if ((c & 0x80) != 0x80){
            break;
        }

    }
    VarInt var = VarInt{v,j};
    return var;
}

uint16_t KlipperCommander::parse_crc(uint8_t* msg, uint8_t length) {
    uint16_t crc = (uint16_t) *(msg+length-3) << 8 | (uint16_t) *(msg+length-2);
    return crc;
}

void KlipperCommander::send_config(uint8_t sequence, uint32_t offset, uint32_t amount) {

    if ((offset+amount) < CONFIG_DICT_LENGTH) {
        enqueue_config_response(sequence, offset, &config[offset], amount);
    } else if(offset==CONFIG_DICT_LENGTH){
        // uint8_t msg[] = {0,CONFIG_DICT_LENGTH,0};
        enqueue_config_response(sequence, offset, &config[offset], CONFIG_DICT_LENGTH-offset);

    } else {
        enqueue_config_response(sequence, offset, &config[offset], CONFIG_DICT_LENGTH-offset);
    }
}

void KlipperCommander::enqueue_response(uint8_t sequence, uint8_t* msg, uint8_t length) {
    Pointer write_ptr = outgoing_fifo.getWritePointer();

    uint8_t send_cmd_len = 5+length;

    uint8_t new_sequence = ((sequence + 1 ) & 0b00001111) | 0x10;

    write_ptr.ptr[0] = send_cmd_len;
    write_ptr.ptr[1] = new_sequence;
    for (int i=0;i<length;i++) {
        write_ptr.ptr[i+2] = msg[i];
    }
    uint16_t crc = crc16(write_ptr.ptr,send_cmd_len-3);
    write_ptr.ptr[2+length] = (uint8_t) (crc >> 8);
    write_ptr.ptr[3+length] = (uint8_t) (crc & 0xFF);
    write_ptr.ptr[4+length] = SYNC_BYTE;

    outgoing_fifo.advanceWriteCursorN(send_cmd_len);
    outgoing_fifo.finalizeMessage();
}

void KlipperCommander::enqueue_config_response(uint8_t sequence, uint32_t offset, uint8_t* msg, uint8_t count) {
    Pointer write_ptr = outgoing_fifo.getWritePointer();
    uint8_t new_sequence = ((sequence +1 ) & 0b00001111) | 0x10;
    uint8_t offset_bytes = encode_vlq_int(&write_ptr.ptr[3], offset);
    uint8_t count_bytes = encode_vlq_int(&write_ptr.ptr[3+offset_bytes], count);
    uint8_t vlq_bytes = offset_bytes+count_bytes;
    uint8_t send_cmd_len = 6+count+vlq_bytes;

    write_ptr.ptr[0] = send_cmd_len;
    write_ptr.ptr[1] = new_sequence;
    write_ptr.ptr[2] = 0; // command id - "identify_response"
    
    for (int i=0;i<count;i++) {
        write_ptr.ptr[i+3+vlq_bytes] = *(msg+i);
    }

    uint16_t crc = crc16(write_ptr.ptr,send_cmd_len-3);
    write_ptr.ptr[3+vlq_bytes+count] = (uint8_t) (crc >> 8);
    write_ptr.ptr[4+vlq_bytes+count] = (uint8_t) (crc & 0xFF);
    write_ptr.ptr[5+vlq_bytes+count] = SYNC_BYTE;

    Serial.printf("command length: %u - 0x%x\n", send_cmd_len,send_cmd_len);
    Serial.printf("Parsed crc: 0x%x\n", parse_crc(write_ptr.ptr,send_cmd_len));
    Serial.printf("Calc'd crc: 0x%x\n", crc);

    outgoing_fifo.advanceWriteCursorN(send_cmd_len);
    outgoing_fifo.finalizeMessage();
}

void print_byte_array(uint8_t* arr, uint8_t len){
    for (int i=0; i<len;i++){
        Serial.print(*(arr+i), HEX);
        Serial.print(" ");
    }
    Serial.print("\n");
}

// copy+paste from klipper, rewrite without goto
uint8_t KlipperCommander::encode_vlq_int(uint8_t *p, uint32_t v) {
    int32_t sv = v;
    uint8_t bytes=0;
    if (sv < (3L<<5)  && sv >= -(1L<<5))  goto f4;
    if (sv < (3L<<12) && sv >= -(1L<<12)) goto f3;
    if (sv < (3L<<19) && sv >= -(1L<<19)) goto f2;
    if (sv < (3L<<26) && sv >= -(1L<<26)) goto f1;
    *p++ = (v>>28) | 0x80;
    bytes+=1;
f1: *p++ = ((v>>21) & 0x7f) | 0x80;
    bytes+=1;
f2: *p++ = ((v>>14) & 0x7f) | 0x80;
    bytes+=1;
f3: *p++ = ((v>>7) & 0x7f) | 0x80;
    bytes+=1;
f4: *p++ = v & 0x7f;
    bytes+=1;
    return bytes;
}