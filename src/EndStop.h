#ifndef _EndStop_H
#define _EndStop_H


class EndStop {
    public:
        bool allocated=false;
        bool torque_error_based=false;
        int pin;
};

#endif
