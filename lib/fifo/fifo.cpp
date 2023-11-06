#include "fifo.h"

FIFO::FIFO() {
    for (int i=0;i<MAX_N_MSGS;i++){
        message_length[i] = 0;
    }
    read_message_length_p = &message_length[0];
    write_message_length_p = &message_length[0];
    write_cursor = &array[0];
    read_cursor = &array[0];
    start_current_write = &array[0];
    // current_write_msg_length = 0;
    array_start = &array[0];
    array_end = &array[FIFO_LENGTH];
}

Pointer FIFO::getWritePointer() {
    available_length = &array[FIFO_LENGTH-1]-write_cursor;

    if (available_length > (uint32_t) MAX_MESSAGE_LEN) {
        available_length = MAX_MESSAGE_LEN;
    }

    int cursor_diff = read_cursor - write_cursor;
    if (cursor_diff > 0 && cursor_diff < MAX_MESSAGE_LEN) {
        available_length = cursor_diff;
    }

    Pointer ptr = Pointer{write_cursor, (uint8_t) available_length, 0};
    return ptr;
}

void FIFO::advanceWriteCursorN(uint8_t n_bytes) {
    // This function has the opportunity to increment the cursor outside of the array in some cases
    // Needs to be managed, possibly outside of this function?

    uint8_t *new_cursor = &write_cursor[n_bytes];
    if (new_cursor <= array_end && new_cursor >= array_start) {
        write_cursor = new_cursor;
        *write_message_length_p += n_bytes;
    }
    
}

void FIFO::setWriteCursorToStart() {
    write_cursor = start_current_write;

}

void FIFO::setWriteCursorOffsetFromStart(uint8_t offset) {
    uint8_t *new_cursor = &start_current_write[offset];
    if (new_cursor <= array_end && new_cursor >= array_start) {
        write_cursor = new_cursor;
        *write_message_length_p = offset;
    }
}

uint8_t FIFO::finalizeMessage() {
    available_length = &array[FIFO_LENGTH-1]-write_cursor;

    uint8_t error = 1;

    if (available_length < (uint32_t) MAX_MESSAGE_LEN) {
        // Serial.println("Reset write cursor to beginning of array");
        write_cursor = &array[0];
        if ((read_cursor - &array[0]) > MAX_MESSAGE_LEN) {
            error = 0;
        }
    } else {
        error = 0;
    }

    start_current_write = write_cursor;

    advance_msg_len_write_ptr();
    *write_message_length_p = 0;
    return error;
}

Pointer FIFO::getReadPointer() {
    Pointer ptr = Pointer{read_cursor, *read_message_length_p, 0};
    return ptr;
}

void FIFO::advanceReadCursor() {
    uint32_t available_read_length = &array[FIFO_LENGTH-1]-(read_cursor+read_message_length_p[0]);
    
    if (available_read_length < MAX_MESSAGE_LEN) {
        // Serial.println("Reset read cursor to beginning of array");
        read_cursor = &array[0];
    } else {
        uint8_t* new_read_cursor = read_cursor+ *read_message_length_p;
        if (new_read_cursor > write_cursor && read_cursor < write_cursor) {
            illegal_read_pointer_overtake = 1;
            read_cursor = write_cursor;
        } else {
            read_cursor = new_read_cursor;
        }
        
    }
    advance_msg_len_read_ptr();
}

uint8_t FIFO::currentWriteMsgGetByteAt(uint8_t idx) {
    uint8_t current_byte = start_current_write[idx];
    return current_byte;
}

void FIFO::advance_msg_len_write_ptr() {
    if (write_message_length_p == &message_length[MAX_N_MSGS-1]) {
        write_message_length_p = &message_length[0];
    } else {
        write_message_length_p += 1;
    }
}

void FIFO::advance_msg_len_read_ptr() {
    if (read_message_length_p == &message_length[MAX_N_MSGS-1]) {
        read_message_length_p = &message_length[0];
    } else {
        read_message_length_p += 1;
    }
}

uint8_t* FIFO::getCurrentWriteMsgStart() {
    return start_current_write;
}

uint8_t FIFO::getCurrentWriteMsgLength() {
    return *write_message_length_p;
}

uint8_t FIFO::getNumMsgToRead() {
    uint8_t n_messages = 0;
    int ptr_diff = write_message_length_p - read_message_length_p;
    
    if (ptr_diff >= 0) {
        n_messages = ptr_diff;
    } else {
        n_messages = MAX_N_MSGS + ptr_diff;
    }
    return n_messages;
}

uint8_t FIFO::confirm_msg(uint8_t msg_len) {
    
    uint8_t *new_start = &start_current_write[msg_len-1];

    if (new_start == write_cursor) {
        return finalizeMessage();
    } else if (((&start_current_write[0] < write_cursor) && (new_start > write_cursor)) || (new_start > array_end)){
        return 1; //error condition
    } 

    start_current_write = new_start;

    *write_message_length_p = msg_len;
    advance_msg_len_write_ptr();
    *write_message_length_p = 0;
    return 0;
}
