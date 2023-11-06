
#include <unity.h>
#include <ArduinoFake.h>
#include <unordered_map>  
#include <cstdio>
#include <stdlib.h>
#include <math.h>
#include <algorithm>
#include <unistd.h>

#include "parse.h"


using namespace fakeit;

void setUp(void) {
    // set stuff up here

}

void tearDown(void) {
    // clean stuff up here
}

void test_encode_decode() {
    char errmsg[256]; 
    
    FILE *fp = fopen("/home/bob/Documents/PlatformIO/Projects/Klipper_commander/test/test_parse/out.txt", "rb");
    TEST_PRINTF("fp: %i\n", fp);
    fseek(fp, 0, SEEK_END);
    long flen = ftell(fp);
    rewind(fp);
    uint8_t *file_bytes = (uint8_t *)malloc(flen * sizeof(uint8_t));
    fread(file_bytes,1,flen, fp);
    fclose(fp);

    int bytes_remaining = flen;
    uint8_t next_seq = 0x10 | 0x01;
    int i = 0;
    while (bytes_remaining >5) {
        uint8_t len = std::min(bytes_remaining,(int) 64);
        TEST_PRINTF("len: %i\n", len);

        msg_location_t ret = find_message(&file_bytes[i], len, next_seq);

        TEST_PRINTF("valid: %i, cnt: %i, start: %p, end: %p", ret.valid_message, ret.start_cnt, ret.start, ret.end);
        if (ret.valid_message == 0 || ret.valid_message == -1) {
            i += ret.start_cnt;
            bytes_remaining -= i;

        }

    }

    free(file_bytes);

}



int main( int argc, char **argv) {
    UNITY_BEGIN();

    RUN_TEST(test_encode_decode);

    UNITY_END();
}
