#include <unity.h>
#include <ArduinoFake.h>
#include <unordered_map>  
#include "vlq.h"



using namespace fakeit;

void setUp(void) {
    // set stuff up here

}

void tearDown(void) {
    // clean stuff up here
}

void test_encode_decode() {
    char errmsg[256]; 
    
    int32_t nums_to_test[12] = {50002,5006,502, 100, 5, 1, 0, -1, -5000, -500, -100};

    for (int i=0; i<sizeof(nums_to_test); i++) {
        
        uint8_t buf[5];

        uint32_t val;

        if (nums_to_test[i] < 0) {
            memcpy(&val, &nums_to_test[i], 4);
        } else {
            val = nums_to_test[i];
        }

        uint8_t n_bytes = encode_vlq_int(buf, val);
        TEST_ASSERT_LESS_OR_EQUAL(5,n_bytes);
        VarInt out = parse_vlq_int(buf, n_bytes);
        TEST_ASSERT_EQUAL(n_bytes, out.length);
        snprintf(errmsg, sizeof(errmsg), "Original value: %i, val into encoder: %u, value out of decoder: %u", nums_to_test[i], val, out.value);
        TEST_ASSERT_EQUAL_MESSAGE(val, out.value, errmsg);

    }

}



int main( int argc, char **argv) {
    UNITY_BEGIN();

    RUN_TEST(test_encode_decode);

    UNITY_END();
}
