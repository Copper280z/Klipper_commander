#include "KlipperCommander.h"
#include "parse.h"
#include <cstdint>


#ifdef USE_TINYUSB
    KlipperCommander::KlipperCommander(Adafruit_USBD_CDC &Serial) : serial(Serial) {
        serial = Serial;
    }
#else
    KlipperCommander::KlipperCommander(Stream &Serial) : serial(Serial) {
        serial = Serial; 
        loop_start_time = micros();
        move_queue = MotionQueue();

        for (size_t i=0;i<sizeof(in_buf);i++){
            in_buf[i]=0;
        }

    }
#endif

void KlipperCommander::handle() {
    recieve_serial();
    send_serial();

    uint32_t current_time = micros();

    if (is_config) {
        update_stats(current_time);
        move_queue.update();
    }
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
    // desired state machine behavior
    //
    // If we had no message previously, or had a complete message with 
    // no extra bytes at the end of the read
    //  - validate/finalize message then advance write pointer to the end of message
    //
    // If we have a complete message with a few more bytes at the end.
    //  - validate/finalize message.
    //  - Move write pointer to end of bytes
    //
    // If we have previously unused bytes and recieve some more.
    //  - Put new bytes into the buffer after the previously recieved bytes
    //  - Start message search/validation at first unused byte, not at the start of new bytes
    if (serial.available()) {
        DEBUG_PRINTLN("New Bytes!");
    }
    while (serial.available())
    {
        uint8_t b = serial.read();
        
        in_buf[in_buf_idx++] = b;
        msg_location_t msg = find_message(in_buf, in_buf_idx, next_seq);
        
        switch (msg.valid_message) {
            case ParseError::MsgValid: 
                {
                    DEBUG_PRINTLN("  Valid Message!");
                    next_seq = 0x10 | ((next_seq+1) & 0x0f);
                    ACKNACK(next_seq);

                    parse_message(msg);
                    
                    // byte after this message to the top of the buffer
                    size_t bytes_left = in_buf_idx-msg.len;
                    memmove(in_buf, msg.end+1, bytes_left);
                    in_buf_idx = bytes_left;
                    
                    break;
                }
            case ParseError::NotEnoughBytes: 
            case ParseError::MsgIncomplete: 
                {
                    // DEBUG_PRINTF("search idx at return: %d\n",msg.start_cnt);
                    // DEBUG_PRINTLN("  not enough data, need to get more and try again");
                    break;
                }
            case ParseError::WrongSequence: 
                {
                    DEBUG_PRINTLN("  Got valid message, but wrong sequence byte, NACK");
                    DEBUG_PRINTF("    Expected %u, got %u\n", next_seq&0xf, msg.start[1]&0xf);
                    // seq is same as last seq, already handled that so drop it and try again
                    // if (msg.start[1]&0xf == (next_seq&0xf-1)) {
                    //     size_t bytes_left = in_buf_idx-msg.len;
                    //     memmove(in_buf, msg.end, bytes_left);
                    //     in_buf_idx = bytes_left;
                    //     break;
                    // }
                    ACKNACK(next_seq);
                    // next_seq = 0x10 | ((next_seq-1) & 0x0f);
                    in_buf_idx = 0;
                    for (size_t i=0; i<(sizeof(in_buf)/sizeof(in_buf[0])); i++) {
                        in_buf[i]=0;
                    }
                     while (serial.available()) {
                        serial.read();
                     }
                    break;
                }
            case -3: 
                {
                    DEBUG_PRINTLN("  Have data but nothing that looks like a message?!");                                
                }
        } // switch (msg.valid_message)


        if (in_buf_idx == (sizeof(in_buf)/sizeof(in_buf[0]))) {
            in_buf_idx = 0;
        }
    } // while serial.available

    return;
}


void KlipperCommander::parse_message(msg_location_t msg) {


        DEBUG_PRINTF("Length of command being parsed: %d\n", msg.len);

        for (int i=0; i<msg.len;i++){
            DEBUG_PRINT(msg.start[i], HEX);
            DEBUG_PRINT(" ");
        }
        DEBUG_PRINT("\n");

        uint8_t sequence = msg.start[1];
        DEBUG_PRINTF("sequence low bytes: %u\n", sequence & 0xf );

        int16_t command_bytes_available = msg.len-MIN_MESSAGE_LEN;
        int32_t bytes_consumed=0;
        while ((command_bytes_available - bytes_consumed)>0) {
            DEBUG_PRINTF("Command bytes available: %u\n", command_bytes_available);
            VarInt cmd_id_var = parse_vlq_int(&msg.start[bytes_consumed+MSG_HEADER_LEN], command_bytes_available);
            bytes_consumed += cmd_id_var.length;
            DEBUG_PRINTF("Command ID:  %u\n", cmd_id_var.value);
            bytes_consumed += command_dispatcher(cmd_id_var.value, sequence, &msg.start[bytes_consumed+MSG_HEADER_LEN], command_bytes_available-bytes_consumed);
            DEBUG_PRINTF("Bytes left in msg after cmd: %d\n", command_bytes_available-bytes_consumed); 

            if (bytes_consumed <= 0) {
                DEBUG_PRINTF("bytes_consumed is < 0: %u\n", bytes_consumed);
                // not sure what that command was, so maybe we might be lost
                // quit and try again next time
                // if this leg is ever taken, something is wrong, probably related to the data dict
                break;
            }
        }
        if ((command_bytes_available - bytes_consumed)>0) {
            DEBUG_PRINTF("&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&\n");
            DEBUG_PRINTF("WARNING LOSING BYTES IN VALID MESSAGE\n");
            DEBUG_PRINTF("&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&\n");
            while (1) {
                // Serial.send_now();
                delay(1000);
            };
        }

}

int32_t KlipperCommander::command_dispatcher(uint32_t cmd_id, uint8_t sequence, uint8_t *msg, int16_t length) {
    uint8_t bytes_consumed = 0;
    switch (cmd_id) {
        case 1:{
            VarInt offset_var = parse_vlq_int((msg), length);
            VarInt amount_var = parse_vlq_int((msg+offset_var.length), length-offset_var.length);
            DEBUG_PRINTF("Offset: %u - len: %u\n", (int32_t) offset_var.value, offset_var.length);
            DEBUG_PRINTF("Amount: %u - len: %u\n", amount_var.value, amount_var.length);
            send_config(sequence, offset_var.value, amount_var.value);
            bytes_consumed = offset_var.length+amount_var.length;
            break;
        }
        case 4:{ //Uptime
            DEBUG_PRINTLN("Serving Uptime");
            current_time = micros();

            uint8_t new_msg[64];
            uint8_t resp_id = 79;
            uint8_t offset = encode_vlq_int(new_msg, resp_id);
            offset+= encode_vlq_int(new_msg+offset, prev_stats_send_high + (current_time < prev_stats_send));
            offset+= encode_vlq_int(new_msg+offset, current_time);
            enqueue_response(sequence, new_msg, offset);
            break;
        }
        case 5: { // get_clock
            current_time = micros();
            uint8_t new_msg[64];
            uint8_t resp_id = 80;
            uint8_t offset = encode_vlq_int(new_msg, resp_id);
            offset+= encode_vlq_int(new_msg+offset, current_time);
            enqueue_response(sequence, new_msg, offset);   
            DEBUG_PRINTF("sum_total_steps: %d\n",sum_total_steps);

            break;
        }
        case 6: { // finalize_config crc=%u
            DEBUG_PRINTF("********************************************************\n");
            VarInt config_crc_var = parse_vlq_int((msg), length);
            host_config_crc = config_crc_var.value;
            DEBUG_PRINTF("finalize_config crc: %u\n", host_config_crc);
            DEBUG_PRINTF("********************************************************\n");
            bytes_consumed+=config_crc_var.length;
            is_config = 1;
            break;    
        }
        case 7: { // get config - return config crc to host
            DEBUG_PRINTF("********************************************************\n");
            uint8_t new_msg[64];
            uint8_t resp_id = 81;
            // needs response with size of move queue
            uint8_t offset = encode_vlq_int(new_msg, resp_id);
            offset += encode_vlq_int(new_msg+offset, is_config);
            offset += encode_vlq_int(new_msg+offset, host_config_crc);
            offset += encode_vlq_int(new_msg+offset, 0); // is_shutdown
            offset += encode_vlq_int(new_msg+offset, (uint16_t)move_queue.getCapacity());
            DEBUG_PRINTF("Stored Host config crc: %u\n", host_config_crc);
            DEBUG_PRINTF("********************************************************\n");
            enqueue_response(sequence, new_msg, offset);    
            break;
        }
        case 8: { // allocate_oids count=%c
            VarInt oids = parse_vlq_int((msg), length);
            bytes_consumed += oids.length;
            num_oids = oids.value;
            DEBUG_PRINTF("Allocating %u oids\n", num_oids);
            
        break;
        }
        case 9: { //debug_nop
        break;
        }
        case 10: { // debug_ping data=%*s
        break;
        }
        case 11: { // debug_write order=%c, addr=%u, val=%u
        break;
        }
        case 12: { // debug_read order=%c, addr=%u
        break;
        }
        // 13-17 digital out pins
        case 13: { //  "set_digital_out pin=%u value=%c": 13,
            uint8_t offset=0;
            VarInt pin_var = parse_vlq_int((msg), length);
            offset += pin_var.length;
            VarInt val_var = parse_vlq_int((msg+offset), length);
            offset += val_var.length;
            bytes_consumed+=offset;
        
        break;
        }
        case 14: { // "update_digital_out oid=%c value=%c": 14, 
            uint8_t offset=0;
            VarInt pin_var = parse_vlq_int((msg), length);
            offset += pin_var.length;
            VarInt val_var = parse_vlq_int((msg+offset), length);
            offset += val_var.length;
            bytes_consumed+=offset;
        
        break;
        }
        case 15: { // "queue_digital_out oid=%c clock=%u on_ticks=%u": 15,
            uint8_t offset=0;
            VarInt oid_var = parse_vlq_int((msg), length);
            offset += oid_var.length;
            VarInt clock_var = parse_vlq_int((msg+offset), length-offset);
            offset += clock_var.length;
            VarInt tick_var = parse_vlq_int((msg+offset), length-offset);
            offset += tick_var.length;
            bytes_consumed+=offset;
        
        break;
        }
        case 17: { // "config_digital_out oid=%c pin=%u value=%c default_value=%c max_duration=%u": 17, 
            uint8_t offset=0;
            VarInt oid_var = parse_vlq_int((msg), length);
            offset += oid_var.length;
            VarInt pin_var = parse_vlq_int((msg+offset), length-offset);
            offset += pin_var.length;
            VarInt val_var = parse_vlq_int((msg+offset), length-offset);
            offset += val_var.length;
            VarInt default_var = parse_vlq_int((msg+offset), length-offset);
            offset += default_var.length;
            VarInt max_dur_var = parse_vlq_int((msg+offset), length-offset);
            offset += max_dur_var.length;
            bytes_consumed+=offset;
        
        break;
        }
        case 18: { // stepper_stop_on_trigger oid=%c, trsync_oid=%c

        break;
        }
        case 19: { //stepper_get_position oid=%c
            uint8_t new_msg[64];
            VarInt oid_var = parse_vlq_int((msg), length);
            bytes_consumed+=oid_var.length;
            // float pos = *move_queue.position_var;
            uint8_t resp_id = 84;
            uint8_t offset = encode_vlq_int(new_msg, resp_id);
            // TODO - Use the specific motor oid
            offset += encode_vlq_int(new_msg+offset, 0);
            // TODO - Get the accumulated motor position
            offset += encode_vlq_int(new_msg+offset, 0);
            enqueue_response(sequence, new_msg, offset);    
            
        break;
        }
        case 20: { // reset_step_clock oid=%c clock=%u
            uint8_t offset=0;
            VarInt oid_var = parse_vlq_int((msg), length);
            offset += oid_var.length;
            VarInt clock_var = parse_vlq_int((msg+offset), length-oid_var.length);
            move_queue.previous_time = clock_var.value;
            bytes_consumed+=oid_var.length + clock_var.length;
            
        break;
        }
        case 21: { // set_next_step_dir oid=%c dir=%c
            uint8_t offset=0;
            VarInt oid_var = parse_vlq_int((msg), length);
            offset += oid_var.length;
            VarInt dir_var = parse_vlq_int((msg+offset), length-offset);
            offset += dir_var.length;
            if ((uint8_t) dir_var.value==1){
                move_queue.host_dir = 1;
            } else {
                move_queue.host_dir = -1;
            }
            bytes_consumed+=offset;
            DEBUG_PRINTF("Setting dir to: %d\n", move_queue.host_dir);
        break;
        }
        case 22: { // queue_step oid=%c interval=%u count=%hu add=%hi

            int8_t offset=0;
            VarInt oid_var = parse_vlq_int((msg), length);
            offset += oid_var.length;
            // DEBUG_PRINTF("q step offset: %u - curr byte: %u - next byte: %u\n", offset,msg[1], msg[1+offset]);
            VarInt interval_var = parse_vlq_int((msg+offset), length-offset);
            offset += interval_var.length;
            VarInt count_var = parse_vlq_int((msg+offset), length-offset);
            offset += count_var.length;
            VarInt add_var = parse_vlq_int((msg+offset), length-offset);
            offset += add_var.length;
            DEBUG_PRINTF("Move queued: oid: %u - interval: %u - count: %u - add: %d\n", oid_var.value, interval_var.value, (uint16_t) count_var.value, (int16_t)add_var.value);
            

            MoveData new_move = MoveData{}; 
            new_move.interval = interval_var.value;
            new_move.count = (uint16_t) count_var.value;
            new_move.add = (int16_t) add_var.value;
            new_move.dir = move_queue.host_dir;

            move_queue.push(new_move);
            bytes_consumed += offset;
            sum_total_steps += move_queue.host_dir * (uint16_t) count_var.value;

        break;
        }
        case 23: { // config_stepper oid=%c step_pin=%c dir_pin=%c invert_step=%c step_pulse_ticks=%ku
            uint8_t offset=0;
            VarInt oid_var = parse_vlq_int((msg), length);
            offset += oid_var.length;
            VarInt step_pin_var = parse_vlq_int((msg+offset), length+offset);
            offset += step_pin_var.length;
            VarInt dir_pin_var = parse_vlq_int((msg+offset), length+offset);
            offset += dir_pin_var.length;
            VarInt invert_step_var = parse_vlq_int((msg+offset), length+offset);
            offset += invert_step_var.length;
            VarInt step_pulse_ticks_var = parse_vlq_int((msg+offset), length+offset);
            offset += step_pulse_ticks_var.length;
            bytes_consumed+=offset;
            stepper_obj = ObjID{true, oid_var.value};
            DEBUG_PRINTF("Allocated Motor with oid %u\n", oid_var.value);
            
        break;
        }
        // endstop state contains the precise time the pin satisfied the trigger conditions
        case 24: { // endstop_query_state oid=%c
            uint32_t next_clock = micros();
            uint8_t new_msg[64];
            uint8_t resp_id = 85;

            VarInt oid_var = parse_vlq_int((msg), length);
            uint8_t endstop_state = move_queue.endstop_state;

            uint8_t offset = encode_vlq_int(new_msg, resp_id);
            offset += encode_vlq_int(new_msg+offset, oid_var.value);            // oid
            offset += encode_vlq_int(new_msg+offset, move_queue.homing);        // homing
            offset += encode_vlq_int(new_msg+offset, next_clock);               // next_clock - Precise time pin satisfies trigger condition
            offset += encode_vlq_int(new_msg+offset, move_queue.endstop_state); // pin_state
            enqueue_response(sequence, new_msg, offset);
            
            
        break;
        }
        case 25: { // endstop_home oid=%c clock=%u sample_ticks=%u sample_count=%c rest_ticks=%u pin_value=%c trsync_oid=%c trigger_reason=%c         
        break;
        }
        case 26: { // config_endstop oid=%c pin=%c pull_up=%c
            uint8_t offset = 0;
            VarInt oid_var = parse_vlq_int((msg), length);
            offset += oid_var.length;
            VarInt pin_var = parse_vlq_int((msg+offset), length+offset);
            offset += pin_var.length;
            VarInt pull_up_var = parse_vlq_int((msg+offset), length+offset);
            offset += pull_up_var.length;
            bytes_consumed += offset;
            DEBUG_PRINTF("Allocating endstop with oid: %u, pin: %u, pull_up: %u\n",oid_var.value, pin_var.value, pull_up_var.value);
        break;
        }
        // trsync returns a message when it is either triggered or the time runs out.
        // this returned message causes the host to proceed and get the endstop state
        // REASON_ENDSTOP_HIT = 1
        // REASON_COMMS_TIMEOUT = 2
        // REASON_HOST_REQUEST = 3
        // REASON_PAST_END_TIME = 4
        case 27: { // trsync_trigger oid=%c reason=%c
            uint8_t offset=0;
            VarInt oid_var = parse_vlq_int((msg), length);
            offset += oid_var.length;
            VarInt reason_var = parse_vlq_int((msg+offset), length+offset);
            offset += reason_var.length;
            bytes_consumed += offset;
            for (int i=0;i<MAX_TRSYNCS;i++) {
                if (!trsync_objs[i].allocated) {
                    continue;
                }
                if (trsync_objs[i].oid == oid_var.value) {
                    trsync_objs[i].triggered = true;
                    trsync_objs[i].expire_reason = reason_var.value;
                    // TODO - if use callbacks, call them here?
                }
            }
        break;
        }
        case 28: { // trsync_set_timeout oid=%c clock=%u
            uint8_t offset=0;
            VarInt oid_var = parse_vlq_int((msg), length);
            offset += oid_var.length;
            VarInt expire_clock_var = parse_vlq_int((msg), length+offset);
            offset += expire_clock_var.length;
            bytes_consumed += offset;
            for (int i=0;i<MAX_TRSYNCS;i++) {
                if (!trsync_objs[i].allocated) {
                    continue;
                }
                if (trsync_objs[i].oid == oid_var.value) {
                    trsync_objs[i].expire_time = expire_clock_var.value;
                }
            }
        break;
        }
        case 29: { // trsync_start oid=%c report_clock=%u report_ticks=%u expire_reason=%c
            uint8_t offset=0;
            VarInt oid_var = parse_vlq_int((msg), length);
            offset += oid_var.length;
            VarInt report_clock_var = parse_vlq_int((msg), length+offset);
            offset += report_clock_var.length;
            VarInt report_ticks_var = parse_vlq_int((msg), length+offset);
            offset += report_ticks_var.length;
            VarInt expire_reason_var = parse_vlq_int((msg), length+offset);
            offset += report_ticks_var.length;
            bytes_consumed += offset;
            for (int i=0;i<MAX_TRSYNCS;i++) {
                if (!trsync_objs[i].allocated) {
                    continue;
                }
                if (trsync_objs[i].oid == oid_var.value) {
                    trsync_objs[i].report_time = report_clock_var.value;
                    trsync_objs[i].expire_reason = expire_reason_var.value;
                }
            }
        break;
        }
        case 30: { // config_trsync oid=%c
            VarInt oid_var = parse_vlq_int((msg), length);
            DEBUG_PRINTF("Allocating trsync with oid: %u\n",oid_var.value);
            for (int i=0;i<MAX_TRSYNCS;i++) {
                if (!trsync_objs[i].allocated) {
                    trsync_objs[i].allocated = true;
                    trsync_objs[i].oid = oid_var.value;
                    break;
                }
            }
            bytes_consumed+= oid_var.length;
        break;
        }
        case 74: { // reset
            host_config_crc = 0;
            next_seq = 0x10 | 0x01;
            is_config = 0;
            break;
        }
        default:{
            DEBUG_PRINTF("&&&&&&&&&&&&&&&&&&&&&&\n");
            DEBUG_PRINTF("WARNING COMMAND NOT FOUND\n");
            DEBUG_PRINTF("&&&&&&&&&&&&&&&&&&&&&&\n");
            while (1) {
                // Serial.send_now();
                delay(1000);
            };
            bytes_consumed = -1;
            break;
        }
    }
    return bytes_consumed;
}

void KlipperCommander::send_serial(){
    uint8_t msgs_to_send = outgoing_fifo.getNumMsgToRead();
    if (msgs_to_send > 0) {
        // DEBUG_PRINTF("\nStarting serial send to host: %u messages\n", msgs_to_send);
    }
    for (int i=0; i<msgs_to_send; i++){
        Pointer read_ptr = outgoing_fifo.getReadPointer();
        // DEBUG_PRINTF("msg sent: ");
        // print_byte_array(read_ptr.ptr, read_ptr.len);
        // for (int i=0;i<read_ptr.len;i++){
            // serial.write(read_ptr.ptr[i]);
        // }
        serial.write(read_ptr.ptr, read_ptr.len);
        serial.flush();
        outgoing_fifo.advanceReadCursor();
    }

}

// void KlipperCommander::ACK(uint8_t sequence) {
//     Pointer write_ptr = outgoing_fifo.getWritePointer();
//     uint8_t new_sequence = ((sequence +1 ) & 0b00001111) | 0x10;
//     // DEBUG_PRINTF("ACK sequence high bytes: 0x%x\n", new_sequence & 0b11110000 );
//     // DEBUG_PRINTF("ACK sequence low bytes: %u\n", new_sequence & 0b00001111 );

//     write_ptr.ptr[0] = 5;
//     write_ptr.ptr[1] = new_sequence;
//     uint16_t crc = crc16(write_ptr.ptr,2);
//     write_ptr.ptr[2] = (uint8_t) (crc >> 8);
//     write_ptr.ptr[3] = (uint8_t) (crc & 0xFF);
//     write_ptr.ptr[4] = SYNC_BYTE;
    
//     outgoing_fifo.advanceWriteCursorN(5);
//     outgoing_fifo.finalizeMessage();

// }

// void KlipperCommander::ACK(uint8_t sequence) {
//     static uint8_t msg_arr[5];
//     uint8_t new_sequence = ((sequence +1 ) & 0xf) | 0x10;

//     msg_arr[0] = 5;
//     msg_arr[1] = new_sequence;
//     uint16_t crc = crc16(msg_arr,2);
//     msg_arr[2] = (uint8_t) (crc >> 8);
//     msg_arr[3] = (uint8_t) (crc & 0xFF);
//     msg_arr[4] = SYNC_BYTE;
//     serial.write(msg_arr,5);
//     serial.flush();
// }

// void KlipperCommander::NACK(uint8_t sequence) {
//     Pointer write_ptr = outgoing_fifo.getWritePointer();
//     uint8_t new_sequence = ((sequence -1 ) & 0b00001111) | 0x10;

//     write_ptr.ptr[0] = 5;
//     write_ptr.ptr[1] = new_sequence;
//     uint16_t crc = crc16(write_ptr.ptr,2);
//     write_ptr.ptr[2] = (uint8_t) (crc >> 8);
//     write_ptr.ptr[3] = (uint8_t) (crc & 0xFF);
//     write_ptr.ptr[4] = SYNC_BYTE;
    
//     outgoing_fifo.advanceWriteCursorN(5);
//     outgoing_fifo.finalizeMessage();
// }

void KlipperCommander::ACKNACK(uint8_t sequence) {
    Pointer write_ptr = outgoing_fifo.getWritePointer();

    write_ptr.ptr[0] = 5;
    write_ptr.ptr[1] = sequence;
    uint16_t crc = crc16(write_ptr.ptr,2);
    write_ptr.ptr[2] = (uint8_t) (crc >> 8);
    write_ptr.ptr[3] = (uint8_t) (crc & 0xFF);
    write_ptr.ptr[4] = SYNC_BYTE;
    
    outgoing_fifo.advanceWriteCursorN(5);
    outgoing_fifo.finalizeMessage();
}

// void KlipperCommander::NACK(uint8_t sequence) {
//     static uint8_t msg_arr[5];
//     uint8_t new_sequence = ((sequence-1) & 0xf) | 0x10;

//     msg_arr[0] = 5;
//     msg_arr[1] = new_sequence;
//     uint16_t crc = crc16(msg_arr,2);
//     msg_arr[2] = (uint8_t) (crc >> 8);
//     msg_arr[3] = (uint8_t) (crc & 0xFF);
//     msg_arr[4] = SYNC_BYTE;
//     serial.write(msg_arr,5);
//     serial.flush();
// }

uint16_t KlipperCommander::crc16(uint8_t* arr, uint8_t length) {
    uint16_t crc = 0xFFFF;
    for (int i = 0; i < length; i++) {
        uint8_t b = *(arr+i) ^ (crc & 0xFF);
        b = b ^ (b << 4);
        crc = ((uint16_t)b << 8 | crc >> 8) ^ ((uint16_t)b >> 4) ^ ((uint16_t)b << 3);
    }
    return crc;
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

void KlipperCommander::enqueue_response(uint8_t sequence, const uint8_t* msg, uint8_t length) {
    Pointer write_ptr = outgoing_fifo.getWritePointer();

    uint8_t send_cmd_len = 5+length;

    uint8_t new_sequence = ((sequence + 1 ) & 0xf) | 0x10;
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

// void KlipperCommander::enqueue_response(uint8_t sequence, const uint8_t* msg, uint8_t length) {
//     uint8_t msg_arr[64];

//     uint8_t send_cmd_len = 5+length;

//     uint8_t new_sequence = ((sequence + 1 ) & 0b00001111) | 0x10;
//     latest_outgoing_sequence = new_sequence;

//     msg_arr[0] = send_cmd_len;
//     msg_arr[1] = new_sequence;
//     for (int i=0;i<length;i++) {
//         msg_arr[i+2] = msg[i];
//     }
//     uint16_t crc = crc16(msg_arr,send_cmd_len-3);
//     msg_arr[2+length] = (uint8_t) (crc >> 8);
//     msg_arr[3+length] = (uint8_t) (crc & 0xFF);
//     msg_arr[4+length] = SYNC_BYTE;

//     serial.write(msg_arr,send_cmd_len);
//     serial.flush();
// }

void KlipperCommander::enqueue_config_response(uint8_t sequence, uint32_t offset, const uint8_t* msg, uint8_t count) {
    Pointer write_ptr = outgoing_fifo.getWritePointer();
    uint8_t new_sequence = ((sequence +1 ) & 0xf) | 0x10;
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

// void KlipperCommander::enqueue_config_response(uint8_t sequence, uint32_t offset, const uint8_t* msg, uint8_t count) {
//     uint8_t msg_arr[64];
//     uint8_t new_sequence = ((sequence +1 ) & 0b00001111) | 0x10;
//     uint8_t offset_bytes = encode_vlq_int(&msg_arr[3], offset);
//     uint8_t count_bytes = encode_vlq_int(&msg_arr[3+offset_bytes], count);
//     uint8_t vlq_bytes = offset_bytes+count_bytes;
//     uint8_t send_cmd_len = 6+count+vlq_bytes;

//     msg_arr[0] = send_cmd_len;
//     msg_arr[1] = new_sequence;
//     msg_arr[2] = 0; // command id - "identify_response"
    
//     for (int i=0;i<count;i++) {
//         msg_arr[i+3+vlq_bytes] = *(msg+i);
//     }

//     uint16_t crc = crc16(msg_arr,send_cmd_len-3);
//     msg_arr[3+vlq_bytes+count] = (uint8_t) (crc >> 8);
//     msg_arr[4+vlq_bytes+count] = (uint8_t) (crc & 0xFF);
//     msg_arr[5+vlq_bytes+count] = SYNC_BYTE;

//     DEBUG_PRINTF("command length: %u - 0x%x\n", send_cmd_len,send_cmd_len);
//     DEBUG_PRINTF("Parsed crc: 0x%x\n", parse_crc(msg_arr,send_cmd_len));
//     DEBUG_PRINTF("Calc'd crc: 0x%x\n", crc);

//     serial.write(msg_arr,send_cmd_len);
//     serial.flush();
// }

void print_byte_array(uint8_t* arr, uint8_t len){
    for (int i=0; i<len;i++){
        DEBUG_PRINT(*(arr+i), HEX);
        DEBUG_PRINT(" ");
    }
    DEBUG_PRINT("\n");
}

// // copy+paste from klipper, rewrite without goto
// uint8_t KlipperCommander::encode_vlq_int(uint8_t *p, uint32_t v) {
//     int32_t sv = v;
//     uint8_t bytes=0;
//     if (sv < (3L<<5)  && sv >= -(1L<<5))  goto f4;
//     if (sv < (3L<<12) && sv >= -(1L<<12)) goto f3;
//     if (sv < (3L<<19) && sv >= -(1L<<19)) goto f2;
//     if (sv < (3L<<26) && sv >= -(1L<<26)) goto f1;
//     *p++ = (v>>28) | 0x80;
//     bytes+=1;
// f1: *p++ = ((v>>21) & 0x7f) | 0x80;
//     bytes+=1;
// f2: *p++ = ((v>>14) & 0x7f) | 0x80;
//     bytes+=1;
// f3: *p++ = ((v>>7) & 0x7f) | 0x80;
//     bytes+=1;
// f4: *p++ = v & 0x7f;
//     bytes+=1;
//     return bytes;
// }

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
    enqueue_response(latest_outgoing_sequence, msg, offset);

    prev_stats_send = current_time;
    stats_loop_count = 0;
    stats_sum = 0;
    stats_sumsq = 0;
}
