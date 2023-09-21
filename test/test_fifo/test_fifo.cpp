#include <unity.h>
#include <ArduinoFake.h>
#include <unordered_map>  
#include "fifo.h"



using namespace fakeit;

void setUp(void) {
    // set stuff up here
    // When(OverloadedMethod(ArduinoFake(Serial), printf, void(unsigned long))).AlwaysReturn();
    When(OverloadedMethod(ArduinoFake(Serial), println, size_t())).AlwaysReturn();
}

void tearDown(void) {
    // clean stuff up here
}

void test_msg_write() {
    FIFO fifo = FIFO();
    uint8_t msg_len = 40;
    uint8_t n_messages = 64;
    uint8_t write_char = 0x7e;

    uint32_t nbytes= (uint32_t)msg_len * (uint32_t)n_messages;
    TEST_ASSERT(nbytes < FIFO_LENGTH);

    for (int i=0; i<n_messages; i++) {
        Pointer ptr = fifo.getWritePointer();
        TEST_ASSERT( ptr.len > msg_len);
        
        for (int j=0; j<msg_len; j++){
            ptr.ptr[j] = write_char;
        }
        
        TEST_ASSERT_EACH_EQUAL_HEX8(ptr.ptr[0], ptr.ptr, msg_len);

        fifo.advanceWriteCursorN(msg_len);
        TEST_ASSERT_EQUAL(msg_len, fifo.getCurrentWriteMsgLength());
        fifo.finalizeMessage();
    }

    TEST_ASSERT(fifo.getNumMsgToRead() == n_messages);
}

void test_msg_write_read() {
    FIFO fifo = FIFO();
    char errmsg[64]; 
    uint8_t msg_len = 33;
    uint8_t n_messages = 120;
    uint8_t write_char_even = 0x7e;
    uint8_t write_char_odd = 0x5f;
    uint8_t* msg_write_start_ptrs[n_messages];
    uint8_t* msg_read_start_ptrs[n_messages];

    uint32_t nbytes= (uint32_t)msg_len * (uint32_t)n_messages;
    TEST_ASSERT_LESS_THAN_MESSAGE(FIFO_LENGTH, nbytes, "Test configured wrong, change msg_len or n_messages");

    for (int i=0; i<n_messages; i++) {
        snprintf(errmsg, sizeof(errmsg), "ptr.ptr[0] Write Message number: %i", i);

        Pointer ptr = fifo.getWritePointer();
        TEST_ASSERT( ptr.len > msg_len);
        msg_write_start_ptrs[i] = ptr.ptr;
        uint8_t write_char=0;
        if (i%2 == 0){
            write_char = write_char_even;
        } else {
            write_char =  write_char_odd;
        }       
        for (int j=0; j<msg_len; j++){
            ptr.ptr[j] = write_char;
            TEST_ASSERT_EQUAL(write_char, ptr.ptr[j]);
        }

        TEST_ASSERT_EACH_EQUAL_HEX8_MESSAGE(ptr.ptr[0], ptr.ptr, msg_len, errmsg);

        // snprintf(errmsg, sizeof(errmsg), "fifo.method Write Message number: %i", i);
        // TEST_ASSERT_EACH_EQUAL_HEX8_MESSAGE(*fifo.getCurrentWriteMsgStart(), ptr.ptr, msg_len, errmsg);

        fifo.advanceWriteCursorN(msg_len);
        TEST_ASSERT(fifo.getCurrentWriteMsgLength() == msg_len);
        fifo.finalizeMessage();
    }
    uint8_t fifo_nmessages = fifo.getNumMsgToRead();
    uint8_t init_fifo_messages = fifo_nmessages;
    TEST_ASSERT(fifo_nmessages == n_messages);

    uint8_t read_counter=0;
    while (fifo_nmessages > 0) {
        TEST_ASSERT_LESS_OR_EQUAL(init_fifo_messages,read_counter);

        Pointer read_ptr = fifo.getReadPointer();
        msg_read_start_ptrs[read_counter] = read_ptr.ptr;

        snprintf(errmsg, sizeof(errmsg), "Read Loop number: %u", read_counter);
        TEST_ASSERT_EQUAL_MESSAGE(msg_len, read_ptr.len, errmsg);

        // READING
        TEST_ASSERT_EQUAL_PTR_MESSAGE(msg_write_start_ptrs[read_counter], msg_read_start_ptrs[read_counter], errmsg);
        TEST_ASSERT_EACH_EQUAL_HEX8_MESSAGE(read_ptr.ptr[0], read_ptr.ptr, read_ptr.len, errmsg);
        read_counter+=1;
        fifo.advanceReadCursor();
        fifo_nmessages = fifo.getNumMsgToRead();
    }

    TEST_ASSERT(read_counter == n_messages);
}

void test_msg_write_read_loop() {
    FIFO fifo = FIFO();
    char errmsg[64]; 
    uint8_t msg_len = 40;
    uint8_t n_messages = 80;
    uint8_t write_char_even = 0x7e;
    uint8_t write_char_odd = 0x5f;
    uint8_t* msg_write_start_ptrs[n_messages];
    uint8_t* msg_read_start_ptrs[n_messages];

    uint32_t nbytes= (uint32_t)msg_len * (uint32_t)n_messages;
    TEST_ASSERT_LESS_THAN_MESSAGE(FIFO_LENGTH, nbytes, "Test configured wrong, change msg_len or n_messages");

    for (int k=0; k<10; k++) {

        for (int i=0; i<n_messages; i++) {
            snprintf(errmsg, sizeof(errmsg), "ptr.ptr[0] Write Message number: %i", i);

            Pointer ptr = fifo.getWritePointer();
            TEST_ASSERT( ptr.len > msg_len);
            msg_write_start_ptrs[i] = ptr.ptr;
            uint8_t write_char=0;
            if ((i+k)%2 == 0){
                write_char = write_char_even;
            } else {
                write_char =  write_char_odd;
            }       
            for (int j=0; j<msg_len; j++){
                ptr.ptr[j] = write_char;
                TEST_ASSERT_EQUAL(write_char, ptr.ptr[j]);
            }

            TEST_ASSERT_EACH_EQUAL_HEX8_MESSAGE(ptr.ptr[0], ptr.ptr, msg_len, errmsg);

            // snprintf(errmsg, sizeof(errmsg), "fifo.method Write Message number: %i", i);
            // TEST_ASSERT_EACH_EQUAL_HEX8_MESSAGE(*fifo.getCurrentWriteMsgStart(), ptr.ptr, msg_len, errmsg);

            fifo.advanceWriteCursorN(msg_len);
            TEST_ASSERT(fifo.getCurrentWriteMsgLength() == msg_len);
            fifo.finalizeMessage();
        }
        uint8_t fifo_nmessages = fifo.getNumMsgToRead();
        uint8_t init_fifo_messages = fifo_nmessages;
        TEST_ASSERT_EQUAL(n_messages, fifo_nmessages);
        printf("starting readout\n");
        uint8_t read_counter=0;
        while (fifo_nmessages > 0) {
            TEST_ASSERT_LESS_OR_EQUAL(init_fifo_messages,read_counter);

            Pointer read_ptr = fifo.getReadPointer();
            msg_read_start_ptrs[read_counter] = read_ptr.ptr;

            snprintf(errmsg, sizeof(errmsg), "Read Loop number: %u", read_counter);
            TEST_ASSERT_EQUAL_MESSAGE(msg_len, read_ptr.len, errmsg);

            // READING
            TEST_ASSERT_EQUAL_PTR_MESSAGE(msg_write_start_ptrs[read_counter], msg_read_start_ptrs[read_counter], errmsg);
            TEST_ASSERT_EACH_EQUAL_HEX8_MESSAGE(read_ptr.ptr[0], read_ptr.ptr, read_ptr.len, errmsg);
            read_counter+=1;
            fifo.advanceReadCursor();
            fifo_nmessages = fifo.getNumMsgToRead();
        }

        TEST_ASSERT(read_counter == n_messages);
    }
}

int main( int argc, char **argv) {
    UNITY_BEGIN();

    RUN_TEST(test_msg_write);
    RUN_TEST(test_msg_write_read);
    RUN_TEST(test_msg_write_read_loop);
    UNITY_END();
}
