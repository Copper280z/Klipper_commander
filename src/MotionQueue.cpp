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
    uint32_t current_time = clock();
    uint32_t elapsed_time = current_time - previous_time;

    if (trsync != NULL) {
        if (trsync->triggered) {
            current_move = MoveData{NO_MOVE_QUEUED,0,0,0};
            while (items_in_queue != 0) {
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

        // static int print_counter=0;
        // if (print_counter>20) {
        //     // Serial.printf("Current time: %u\n", current_time);
        //     // Serial.printf("Current move interval: %u\n", current_move.interval);
        //     // Serial.printf("Current queue size: %u out of %u\n", items_in_queue, queue_capacity);
            // Serial.printf("pos: %.3f - vel: %.3f - accel: %.3f\n",position,velocity,accel);
        //     print_counter = 0;
        // }
        // print_counter+=1;
        float delta_counts = (float) elapsed_time / (float) current_move.interval; 
        if (delta_counts > 2) {
            // Serial.printf("Possibly Meaningful Warning, loop rate slower than step rate: %.2f\n", delta_counts);
            // Serial.printf("Current elapsed time: %u\n", elapsed_time);
        }
        
        // make sure we don't move beyond the commanded move
        if (floor(delta_counts) > current_move.count) {
            delta_counts = float(current_move.count);
        }
        step_count += current_move.dir;

        position += position_coeff * current_move.dir;
        // Serial.printf("pos: %.3f - coeff: %.3f - delta_counts: %.3f - dir: %d\n", position, position_coeff, delta_counts, current_move.dir);

        // subtract added counts from current_move
        current_move.count -= 1;
    
        // increment current_move.interval by add*delta_counts
        current_move.interval += current_move.add;

        // add counts to position variable
        if (position_var != NULL) {
            *position_var = position;
        }
        velocity = velocity_coeff / current_move.interval;
        // update attached velocity FF var
        if (velocity_var != NULL) {
            *velocity_var = velocity;
        }
        accel = acceleration_coeff * current_move.add / current_move.interval;
        // update attached acceleration var
        if (acceleration_var != NULL) {
            *acceleration_var = accel;
        }
        previous_time = current_time;
        // if (print_counter>20) {
        //     Serial.printf("Current step count: %d\n", step_count);
        // }
    }

    if (current_move.count == 0) {
        current_move = pop(); 
        // if (current_move.interval != NO_MOVE_QUEUED){
        //     Serial.printf("Current move interval: %u - count: %u - add: %d - dir: %d\n", current_move.interval, current_move.count, current_move.add, current_move.dir);
        //     Serial.flush();
        //     Serial.printf("pos: %.3f - vel: %.3f - accel: %.3f\n\n",position,velocity,accel);
        //     Serial.flush();
        // }
    }
    if (items_in_queue==0){
        velocity=0;
        accel=0;
    }
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

