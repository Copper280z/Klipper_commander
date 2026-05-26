#include "MotionQueue.h"

// #define NO_MOVE_QUEUED 999999999

MotionQueue::MotionQueue() {
    clock = micros;
    // current_move = MoveData{};
    current_move.interval = NO_MOVE_QUEUED;
    current_move.count = 0;
    current_move.add = 0;
    head = 0;
    tail = 0;
    // position_coeff = 32.0f/(200.0f*16.0f);
}

MotionQueue::MotionQueue(unsigned long (*user_clock)(void)){
    current_move.interval = NO_MOVE_QUEUED;
    current_move.count = 0;
    current_move.add = 0;
    clock = user_clock;
    head = 0;
    tail = 0;
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

float MotionQueue::getPosition(){
    return position;
}
float MotionQueue::getVelocity(){
    return velocity;
}
float MotionQueue::getAccel(){
    return accel;
}

void MotionQueue::update() {
    if (trsync != NULL && trsync->triggered) {
        current_move = MoveData{NO_MOVE_QUEUED, 0, 0, 0};
        while (items_in_queue != 0) {
            pop();
        }
        trsync = NULL;
        if (velocity_var != NULL)     *velocity_var = 0;
        if (acceleration_var != NULL) *acceleration_var = 0;
    }

    // Drain every step that is due this iteration using wrap-safe signed comparison.
    // next_step_time is advanced by the scheduled interval (never by "now") so
    // loop-timing overshoot does not accumulate into the step timeline.
    while (current_move.count > 0 &&
           (int32_t)(clock() - next_step_time) >= 0) {

        step_count += current_move.dir;
        position   += position_coeff * current_move.dir;
        if (position_var != NULL) *position_var = position;

        next_step_time        += current_move.interval;
        current_move.interval += current_move.add;
        current_move.count    -= 1;

        velocity = velocity_coeff / current_move.interval;
        if (velocity_var != NULL)     *velocity_var = velocity;
        accel = acceleration_coeff * current_move.add / current_move.interval;
        if (acceleration_var != NULL) *acceleration_var = accel;
    }

    if (current_move.count == 0) {
        current_move = pop();
    }

    if (items_in_queue == 0) {
        velocity = 0;
        accel    = 0;
    }
}

void MotionQueue::clear() {
    while (items_in_queue != 0) {
        pop();
    }
    current_move = MoveData{NO_MOVE_QUEUED, 0, 0, 0};
}

int8_t MotionQueue::push(MoveData new_move) {
    if (items_in_queue<MOVE_QUEUE_LEN) {
        move_array[head] = new_move;
        if (head >= (MOVE_QUEUE_LEN-1)) {
            head = 0;
        } else {
            head += 1;
        }
        items_in_queue += 1;
        return 0;
    } else {
        return -1;
    }
}

uint16_t MotionQueue::getCapacity() {
    return queue_capacity;
}

uint16_t MotionQueue::getSize() {
    return items_in_queue;
}

MoveData MotionQueue::pop() {
    if (items_in_queue > 0) {
        items_in_queue-=1;
        
        MoveData next_move = move_array[tail];

        if (tail >= (MOVE_QUEUE_LEN-1)) {
            tail = 0;
        }else {
            tail+=1;
        }
        return next_move;

    } else {
        // Move of large interval with no counts, should mean do nothing and wait.
        return MoveData{NO_MOVE_QUEUED,0,0,0};
    }
}

