#ifndef _FIFO_H
#define _FIFO_H

#ifndef _TEST
    #include "Arduino.h"
#else
    #include "ArduinoFake.h"
#endif

#ifndef MAX_MESSAGE_LEN
#define MAX_MESSAGE_LEN 64
#endif

#define FIFO_LENGTH 256 // 64*64 for now, probably needs to be much less in real world usage
#define MAX_N_MSGS 6 // most messages are much shorter than the 64 byte max

struct Pointer {
    uint8_t* ptr;
    uint8_t len;
    uint8_t err;
};


class FIFO {
    public:

        FIFO();
        /**************** 
         * returns a Pointer object, which contains a pointer to the start of writable memory
         * and the length of the available memory to write to
        *****************/
        Pointer getWritePointer();

        /**************** 
         * Advances the write cursor by the specified number of bytes and also
         * adds this value to the internal message length counter.
         * This is useful for the case where you finish recieving a message and also recieve a portion of another.
        *****************/
        void advanceWriteCursorN(uint8_t n_bytes);
        
         /**************** 
          * Sets the write cursor back to the start of the current write       
        *****************/
        void setWriteCursorToStart();
        /**************** 
         * Maybe don't use?
         * Adds n_bytes to the internal counter for the current message being written
         * once advanceWriteCursor is called, the cursor is advanced by the sum of all calls to this function.
         * The sum is also saved in the array of message lengths to be provided during a read call.
        *****************/  
        // void numBytesWritten(uint8_t n_bytes);

        /**************** 
         * Saves the current message length in the array of message lengths.
         * Ensures at least MAX_MSG_LEN bytes are available in the underlying array for the next message
         * if MAX_MSG_LEN bytes are not available between the current cursor
         * position and the end of the array, but the read pointer has advanced at least 
         * MAX_MSG_LEN bytes from the start, the write cursor will move to the beginning of the array.
         * If no memory is available, there will be a non-zero return.
         * Returns:
         *  0 = Success
         *  1 = Error
        *****************/        
        uint8_t finalizeMessage();
        // uint8_t finalizeMessage(uint8_t len_from_start);

        /**************** 
         * returns a struct containing the pointer to the location of the message in the array
         * and the length of the message. reading beyond the specified length may read outside the bounds of the array
        *****************/
        Pointer getReadPointer();
        
        /**************** 
         * advances the read cursor to the start of the next valid message
        *****************/  
        void advanceReadCursor();

        /**************** 
         * returns the byte at the specified index in the current message
        *****************/          
        uint8_t currentWriteMsgGetByteAt(uint8_t idx);

        uint8_t* getCurrentWriteMsgStart();

        uint8_t getCurrentWriteMsgLength();
        
        uint8_t getNumMsgToRead();

        uint8_t* array_end;
        uint8_t* array_start;
        uint32_t available_length;

        uint8_t illegal_read_pointer_overtake=0;

        void advance_msg_len_write_ptr();

        uint8_t confirm_msg(uint8_t msg_len);
        void setWriteCursorOffsetFromStart(uint8_t offset);
        // void advance_msg_len_write_ptr(int set_val);

    private:

        uint8_t message_length[MAX_N_MSGS]; // each value is the number of bytes contained in a single, valid, message
        uint8_t* read_message_length_p; // pointer to the length of the message that starts at the read_cursor position
        uint8_t* write_message_length_p; // pointer to the length of the message being written

        uint8_t array[FIFO_LENGTH]; // underlying array containing all message data

        uint8_t* write_cursor; // pointer to the current write cursor location

        uint8_t* read_cursor; // pointer to the current read cursor location

        uint8_t* start_current_write; // pointer to the start of the current working message
        // uint8_t current_write_msg_length; // counter for length of current message

        
        void advance_msg_len_read_ptr();
};

#endif