#include "vlq.h"

VarInt parse_vlq_int(uint8_t* bytes, uint8_t length) {
    uint32_t c = bytes[0];
    uint32_t v = c & 0x7F;


    uint8_t j=1;
    if ((c & 0x60) == 0x60)
        v |= -0x20;
    while (c & 0x80) {
        c = bytes[j];
        v = (v<<7) | (c & 0x7f);
        j++;
    }

    VarInt var = VarInt{v,j};
    return var;
}

// copy+paste from klipper, rewrite without goto
uint8_t encode_vlq_int(uint8_t *p, uint32_t v) {
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
