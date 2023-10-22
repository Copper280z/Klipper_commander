#ifndef _MOTIONQUEUE
#define _MOTIONQUEUE

#include "Arduino.h"

#define MOVE_QUEUE_LEN 128

#define DEBUG 0
#define DEBUG_PRINTF if (DEBUG) Serial.printf
#define DEBUG_PRINT if (DEBUG) Serial.print
#define DEBUG_PRINTLN if (DEBUG) Serial.print

struct MoveData {
    uint32_t interval;
    uint32_t count;
    int32_t add;
    int8_t dir;
};

class MotionQueue {
    public:
        MotionQueue();
        // MotionQueue(float *position);
        // MotionQueue(float *position, float *velocity);
        // MotionQueue(float *position, float *velocity, float *acceleration);
        MotionQueue(unsigned long (*clock)(void));
        // MotionQueue(unsigned long (*clock)(void), float *position);
        // MotionQueue(unsigned long (*clock)(void), float *position, float *velocity);
        // MotionQueue(unsigned long (*clock)(void), float *position, float *velocity, float *acceleration);

		void attach(float &position, float &velocity_ff, float &torque_ff);
		void attach(float &position, float &velocity_ff);
		void attach(float &position);

        // function call in main loop updates attached variables
        void update();
        
        int8_t push(MoveData new_move);
        uint8_t getCapacity();
        uint8_t getSize();

        float *position_var;
        float *velocity_var;
        float *acceleration_var;
        
        
        unsigned long (*clock)(void);
        uint32_t previous_time;

        MoveData current_move;
        float position_coeff;
        float velocity_coeff;
        float acceleration_coeff;

    private:

        MoveData pop();
        MoveData *head;
        MoveData *tail;
        uint8_t queue_size;
        uint8_t queue_capacity;
        MoveData move_array[MOVE_QUEUE_LEN];
};


#endif