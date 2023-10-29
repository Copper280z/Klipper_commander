#ifndef _TRSYNC
#define _TRSYNC

#include "Arduino.h"

class TrSync {
    public:
        TrSync();

        bool allocated=false;
        bool triggered=false;
        uint8_t oid;
        uint8_t reason=1;
        uint32_t expire_time;
        uint32_t report_time;
        uint8_t flags, trigger_reason, expire_reason;
        
        //callback?
};

#endif
