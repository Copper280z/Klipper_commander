#include "KlipperCommander.h"


#ifdef USE_TINYUSB
    KlipperCommander::KlipperCommander(Adafruit_USBD_CDC &Serial) : serial(Serial) {
        serial = Serial;
    }
#else
    KlipperCommander::KlipperCommander(Stream &Serial) : serial(Serial) {
        serial = Serial; 
        loop_start_time = micros();
        move_queue = MotionQueue();

    }
#endif

void KlipperCommander::handle() {
    recieve_serial();
    parse_message();
    send_serial();

    uint32_t current_time = micros();
    update_stats(current_time);
    move_queue.update();
}

void KlipperCommander::attach(float &position_p) {
    move_queue.attach(position_p);
}

void KlipperCommander::attach(float &position_p, float &velocity_p) {
    move_queue.attach(position_p, velocity_p);
}

void KlipperCommander::attach(float &position_p, float &velocity_p, float &acceleration_p) {
    move_queue.attach(position_p, velocity_p, acceleration_p);
}

void KlipperCommander::recieve_serial() {
    Pointer curr_write =  incoming_fifo.getWritePointer();


    uint8_t bytes_read = 0;
    while (serial.available() > 0) {
        if (bytes_read >= (curr_write.len-1)) break;
        curr_write.ptr[bytes_read] = serial.read();
        bytes_read +=1;
    }

    if (bytes_read == 0) return;
    print_byte_array(curr_write.ptr, bytes_read);

    uint8_t msg_length = incoming_fifo.currentWriteMsgGetByteAt(0);
    uint8_t msg_bytes_already_recvd = incoming_fifo.getCurrentWriteMsgLength();

    uint8_t bytes_available = msg_bytes_already_recvd + bytes_read;
    // DEBUG_PRINTF("Now have %u bytes available to do stuff with\n", bytes_available);
    
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
        DEBUG_PRINTF("msg length: 0x%x\n", msg_length);
        DEBUG_PRINTF("msg sync:    0x%x\n", msg_sync);
        DEBUG_PRINTF("calc'd crc: 0x%x\n", calc_crc);
        DEBUG_PRINTF("msg crc:    0x%x\n", msg_crc);
        
        if (msg_sync == SYNC_BYTE && msg_crc == calc_crc && msg_length <= 64 && msg_length > 0) {
            DEBUG_PRINTLN("Valid Message!");
            uint8_t sequence = incoming_fifo.currentWriteMsgGetByteAt(1);
            ACK(sequence);
            incoming_fifo.advanceWriteCursorN(msg_length);
            incoming_fifo.finalizeMessage();
            bytes_available -= msg_length;
            msg_length = incoming_fifo.currentWriteMsgGetByteAt(0);
        } else {
            DEBUG_PRINTLN("Invalid Message!");
            print_byte_array(msg_ptr, bytes_available);
            // out of sync - look for a SYNC_BYTE and copy 
            // everything after that back to the start of current msg, then start over
            bool sync=false;
            for (int i=0;i<bytes_available;i++) {
                uint8_t curr_byte = incoming_fifo.currentWriteMsgGetByteAt(i);
                DEBUG_PRINTF("0x%x\n", curr_byte);
                if (curr_byte == SYNC_BYTE) {
                    memcpy(msg_ptr, msg_ptr+i+1, bytes_available-i+1);
                    bytes_available -= i;
                    sync=true;
                    msg_length = incoming_fifo.currentWriteMsgGetByteAt(0);
                    break;
                }
            }
            if (sync==false) {
                bytes_available = 0;
                incoming_fifo.setWriteCursorToStart();
            }
        }
    }
}


void KlipperCommander::parse_message() {
    // only try to parse up to command_idx-1, command_idx may be incomplete
    uint8_t commands_to_parse = incoming_fifo.getNumMsgToRead();

    while (commands_to_parse > 0) {
        DEBUG_PRINTF("\n%d commands in queue\n", commands_to_parse);
        Pointer read_ptr = incoming_fifo.getReadPointer();

        DEBUG_PRINTF("Length of command being parsed: %d\n", read_ptr.len);

        for (int i=0; i<read_ptr.len;i++){
            DEBUG_PRINT(*(read_ptr.ptr+i), HEX);
            DEBUG_PRINT(" ");
        }
        DEBUG_PRINT("\n");

        uint8_t sequence = *(read_ptr.ptr+1);
        DEBUG_PRINTF("sequence low bytes: %u\n", sequence & 0b00001111 );

        int16_t command_bytes_available = read_ptr.len-MIN_MESSAGE_LEN;
        int32_t bytes_consumed=0;
        while (command_bytes_available>0) {
            DEBUG_PRINTF("Command bytes available: %u\n", command_bytes_available);
            VarInt cmd_id_var = parse_vlq_int(read_ptr.ptr+2+bytes_consumed, command_bytes_available);
            DEBUG_PRINTF("Command ID:  %u\n", (int32_t) cmd_id_var.value);
            bytes_consumed = command_dispatcher(cmd_id_var.value, sequence, read_ptr.ptr+2+bytes_consumed, command_bytes_available);
            if (bytes_consumed < 0) {
                DEBUG_PRINTF("bytes_consumed is < 0: %u\n", bytes_consumed);
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
            DEBUG_PRINTF("Offset: %u - len: %u\n", (int32_t) offset_var.value, offset_var.length);
            DEBUG_PRINTF("Amount: %u - len: %u\n", amount_var.value, amount_var.length);
            send_config(sequence, offset_var.value, amount_var.value);
            bytes_consumed = 1+offset_var.length+amount_var.length;
            break;
        }
        case 4:{ //Uptime
            current_time = micros();

            uint8_t new_msg[64];
            uint8_t resp_id = 79;
            uint8_t offset = encode_vlq_int(new_msg, resp_id);
            offset+= encode_vlq_int(new_msg+offset, prev_stats_send_high + (current_time < prev_stats_send));
            offset+= encode_vlq_int(new_msg+offset, current_time);
            enqueue_response(sequence, new_msg, offset);
        }
        case 5: { //get_clock
            current_time = micros();
            uint8_t new_msg[64];
            uint8_t resp_id = 80;
            uint8_t offset = encode_vlq_int(new_msg, resp_id);
            offset+= encode_vlq_int(new_msg+offset, current_time);
            enqueue_response(sequence, new_msg, offset);     
        }
        case 6: { // finalize config
            VarInt config_crc_var = parse_vlq_int((msg+1), length-1);
            host_config_crc = config_crc_var.value;
            // needs response with size of move queue
            // need to decide how to implement move queue
        }
        case 7: { // get config - return config crc to host

        }
        case 8: { // allocate_oids count=%c
        // maybe this can be a nop, since I'd rather everything be static anyhow
        }
        case 9: { //debug_nop
        }
        case 10: { // debug_ping data=%*s
        }
        case 11: { // debug_write order=%c, addr=%u, val=%u
        }
        case 12: { // debug_read order=%c, addr=%u
        }
        // 13-17 digital out pins
        case 18: { // stepper_stop_on_trigger oid=%c, trsync_oid=%c

        }
        case 19: { //stepper_get_position oid=%c

        }
        case 20: { // reset_step_clock oid=%c clock=%u
            VarInt oid_var = parse_vlq_int((msg+1), length-1);
            VarInt clock_var = parse_vlq_int((msg+1), length-1+oid_var.length);
            move_queue.previous_time = clock_var.value;
            
        }
        case 21: { // set_next_step_dir oid=%c dir=%c
            VarInt oid_var = parse_vlq_int((msg+1), length-1);
            VarInt dir_var = parse_vlq_int((msg+1), length-1+oid_var.length);
            move_queue.host_dir = (int8_t) dir_var.value;
        
        }
        case 22: { // queue_step oid=%c interval=%u count=%hu add=%hi
            uint8_t offset=0;
            VarInt oid_var = parse_vlq_int((msg+1), length-1);
            offset += oid_var.length;
            VarInt interval_var = parse_vlq_int((msg+1), length-1+offset);
            offset += interval_var.length;
            VarInt count_var = parse_vlq_int((msg+1), length-1+offset);
            offset += count_var.length;
            VarInt add_var = parse_vlq_int((msg+1), length-1+offset);
            
            MoveData new_move = MoveData{interval_var.value, count_var.value, (int32_t) add_var.value, move_queue.host_dir}; 
            move_queue.push(new_move);
        }
        case 23: { // config_stepper oid=%c step_pin=%c dir_pin=%c invert_step=%c step_pulse_ticks=%ku
            // can ignore everything except oid
            VarInt oid_var = parse_vlq_int((msg+1), length-1);
            stepper_obj = ObjID{true, oid_var.value};
            
        }
        case 24: { // endstop_query_state oid=%c
            uint32_t next_clock = micros();
            uint8_t new_msg[64];
            uint8_t resp_id = 85;

            VarInt oid_var = parse_vlq_int((msg+1), length-1);
            uint8_t endstop_state = move_queue.endstop_state;

            uint8_t offset = encode_vlq_int(new_msg, resp_id);
            offset += encode_vlq_int(new_msg+offset, oid_var.value);            // oid
            offset += encode_vlq_int(new_msg+offset, move_queue.homing);        // homing
            offset += encode_vlq_int(new_msg+offset, next_clock);               // next_clock --TODO: Check implementation
            offset += encode_vlq_int(new_msg+offset, move_queue.endstop_state); // pin_state
            enqueue_response(sequence, new_msg, offset);
            
            
        }
        case 25: { // endstop_home oid=%c clock=%u sample_ticks=%u sample_count=%c rest_ticks=%u pin_value=%c trsync_oid=%c trigger_reason=%c         
        }
        case 26: { // config_endstop oid=%c pin=%c pull_up=%c
            VarInt oid_var = parse_vlq_int((msg+1), length-1);
            endstop_obj = ObjID{true, oid_var.value};
        }
        case 27: { // trsync_trigger oid=%c reason=%c
        }
        case 28: { // trsync_set_timeout oid=%c clock=%u
        }
        case 29: { // trsync_start oid=%c report_clock=%u report_ticks=%u expire_reason=%c
        }
        case 30: { // config_trsync oid=%c
            VarInt oid_var = parse_vlq_int((msg+1), length-1);
            trsync_obj = ObjID{true, oid_var.value};
        }
        case 74: { // reset
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
        DEBUG_PRINTF("\nStarting serial send to host: %u messages\n", msgs_to_send);
    }
    for (int i=0; i<msgs_to_send; i++){
        Pointer read_ptr = outgoing_fifo.getReadPointer();
        DEBUG_PRINTF("msg sent: ");
        print_byte_array(read_ptr.ptr, read_ptr.len);
        serial.write(read_ptr.ptr, read_ptr.len);
        outgoing_fifo.advanceReadCursor();
    }

}

void KlipperCommander::ACK(uint8_t sequence) {
    Pointer write_ptr = outgoing_fifo.getWritePointer();
    uint8_t new_sequence = ((sequence +1 ) & 0b00001111) | 0x10;
    // DEBUG_PRINTF("ACK sequence high bytes: 0x%x\n", new_sequence & 0b11110000 );
    // DEBUG_PRINTF("ACK sequence low bytes: %u\n", new_sequence & 0b00001111 );

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
    latest_outgoing_sequence = new_sequence;

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

    DEBUG_PRINTF("command length: %u - 0x%x\n", send_cmd_len,send_cmd_len);
    DEBUG_PRINTF("Parsed crc: 0x%x\n", parse_crc(write_ptr.ptr,send_cmd_len));
    DEBUG_PRINTF("Calc'd crc: 0x%x\n", crc);

    outgoing_fifo.advanceWriteCursorN(send_cmd_len);
    outgoing_fifo.finalizeMessage();
}

void print_byte_array(uint8_t* arr, uint8_t len){
    for (int i=0; i<len;i++){
        DEBUG_PRINT(*(arr+i), HEX);
        DEBUG_PRINT(" ");
    }
    DEBUG_PRINT("\n");
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

void KlipperCommander::update_stats(uint32_t current_time) {
    uint32_t looptime = current_time - loop_start_time;
    stats_loop_count++;

    stats_sum += looptime;
    uint32_t next_sumsq;
    if (looptime <= 0xffff) {
        next_sumsq = stats_sumsq + (looptime * looptime+SUMSQ_BASE-1)/SUMSQ_BASE;
    } else if (looptime <= 0xfffff) {
        next_sumsq = stats_sumsq + looptime * (looptime+SUMSQ_BASE-1)/SUMSQ_BASE;
    } else {
        next_sumsq = 0xffffffff;
    }
    if (next_sumsq < stats_sumsq)
        next_sumsq = 0xffffffff;
    stats_sumsq = next_sumsq;

    loop_start_time = current_time;
    // send stats update every 5sec
    // if not time yet, return now
    if (current_time < prev_stats_send+5000000) {
        return;
    }

    // not sure how this is ever reached, maybe when the clock rolls over?
    if (current_time < prev_stats_send){
        prev_stats_send_high++;
    }

    uint8_t msg[64];
    uint8_t resp_id = 78;
    uint8_t offset = encode_vlq_int(msg, resp_id);
    offset+= encode_vlq_int(msg+offset, stats_loop_count);
    offset+= encode_vlq_int(msg+offset, stats_sum);
    offset+= encode_vlq_int(msg+offset, stats_sumsq);
    enqueue_response(latest_outgoing_sequence-1, msg, offset);

    prev_stats_send = current_time;
    stats_loop_count = 0;
    stats_sum = 0;
    stats_sumsq = 0;
}
