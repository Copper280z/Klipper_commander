#include "MotionQueue.h"

MotionQueue::MotionQueue() {
    clock = micros;
    current_move = MoveData{99999999,0,0,0};
    head = &move_array[0];
    tail = &move_array[0];
}

MotionQueue::MotionQueue(unsigned long (*user_clock)(void)){
    clock = user_clock;
    head = &move_array[0];
    tail = &move_array[0];
}

void MotionQueue::attach(float &position) {
    position_var = &position;
}

void MotionQueue::attach(float &position, float &velocity) {
    position_var = &position;
    velocity_var = &velocity;
}

void MotionQueue::attach_trsync(TrSync &new_trsync) {
    trsync = &new_trsync;
}

void MotionQueue::update() {
    uint32_t current_time = clock();
    uint32_t elapsed_time = current_time - previous_time;

    if (trsync != NULL) {
        if (trsync->triggered) {
            current_move = MoveData{9999999,0,0,0};
            while (queue_capacity != 0) {
                pop();
            }
            trsync = NULL;
            if (velocity_var != NULL) {
                *velocity_var = 0;
            }
            
            // update attached acceleration var
            if (acceleration_var != NULL) {
                *acceleration_var = 0;
            }
        }
    }

    if (elapsed_time > current_move.interval) {
        float delta_counts = (float) elapsed_time / (float) current_move.interval; 
        if (delta_counts > 2) {
            // DEBUG_PRINTF("Possibly Meaningful Warning, loop rate slower than step rate: %.2f\n", delta_counts);
        }
        
        // make sure we don't move beyond the commanded move
        if (floor(delta_counts) > current_move.count) {
            delta_counts = float(current_move.count);
        }

        // add counts to position variable
        if (position_var != NULL) {
            *position_var += position_coeff * floor(delta_counts) * (float) current_move.dir;
        
            // subtract added counts from current_move
            current_move.count -= floor(delta_counts);
        
            // increment current_move.interval by add*delta_counts
            current_move.interval += floor(delta_counts)*current_move.add;
        }
        // update attached velocity FF var
        if (velocity_var != NULL) {
            *velocity_var += velocity_coeff * current_move.interval;
        }
        
        // update attached acceleration var
        if (acceleration_var != NULL) {
            *acceleration_var += acceleration_coeff * current_move.add;
        }
    }

    if (current_move.count == 0) {
       current_move = pop(); 
    }
}

int8_t MotionQueue::push(MoveData new_move) {
    if (queue_size<MOVE_QUEUE_LEN) {
        *head = new_move;
        if (head == &move_array[MOVE_QUEUE_LEN-1]) {
            head = &move_array[0];
        } else {
            head+=1;
        }
        queue_size+=1;
        return 0;
    } else {
        return -1;
    }
}

uint8_t MotionQueue::getCapacity() {

    return queue_capacity;
}

uint8_t MotionQueue::getSize() {

    return queue_size;
}

MoveData MotionQueue::pop() {
    if (queue_size > 0) {
        queue_size-=1;
        
        MoveData next_move = *tail;

        if (tail == &move_array[MOVE_QUEUE_LEN-1]) {
            tail = &move_array[0];
        }else {
            tail+=1;
        }
        return next_move;

    } else {
        // Move of large interval with no counts, should mean do nothing and wait.
        return MoveData{9999999,0,0,0};
    }
}

