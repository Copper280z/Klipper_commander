#ifndef _TRSYNC
#define _TRSYNC

#include "Arduino.h"

class TrSync {
    public:
        TrSync();

        bool allocated=false;
        bool triggered=false;
        uint8_t oid = 0;
        uint8_t reason = 1;
        uint32_t expire_time = 0;
        uint32_t report_time = 0;
        uint8_t flags = 0;
        uint8_t trigger_reason = 0;
        uint8_t expire_reason = 0;
        
        //callback?
};

#endif
