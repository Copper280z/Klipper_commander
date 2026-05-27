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

void MotionQueue::attach(float &position, float &velocity_ff, float &torque_ff) {
    position_var     = &position;
    velocity_var     = &velocity_ff;
    acceleration_var = &torque_ff;
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

        velocity = (velocity_coeff / (float)current_move.interval)
                 * (float)current_move.dir;
        if (velocity_var != NULL) *velocity_var = velocity;

        accel += accel_slope_per_us * (float)current_move.interval;
        if (acceleration_var != NULL) *acceleration_var = accel;
    }

    if (current_move.count == 0) {
        current_move = pop();
        plan_chunk_accel();
    }

    if (items_in_queue == 0 && current_move.count == 0) {
        velocity = 0.0f;
        accel    = 0.0f;
        accel_slope_per_us = 0.0f;
        accel_target = 0.0f;
        if (velocity_var != NULL)     *velocity_var = velocity;
        if (acceleration_var != NULL) *acceleration_var = accel;
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

MoveData MotionQueue::peek(uint16_t offset) const {
    if (offset >= items_in_queue) {
        return MoveData{NO_MOVE_QUEUED, 0, 0, 0};
    }
    size_t idx = tail + offset;
    if (idx >= MOVE_QUEUE_LEN) idx -= MOVE_QUEUE_LEN;
    return move_array[idx];
}

static inline int64_t chunk_duration_us(const MoveData &m) {
    // sum_{k=0..count-1} (interval + add*k) = count*interval + add*count*(count-1)/2
    int64_t c = (int64_t)m.count;
    return c * (int64_t)m.interval
         + (int64_t)m.add * c * (c - 1) / 2;
}

void MotionQueue::plan_chunk_accel() {
    if (current_move.count == 0 || current_move.interval == NO_MOVE_QUEUED) {
        accel_slope_per_us = 0.0f;
        accel_target = 0.0f;
        return;
    }

    float v_now = (velocity_coeff / (float)current_move.interval)
                * (float)current_move.dir;

    int64_t duration_us = chunk_duration_us(current_move);
    if (duration_us <= 0) {
        accel_slope_per_us = 0.0f;
        accel_target = accel;
        return;
    }

    // Velocity-equality tolerance: half a velocity-LSB at the current interval.
    float interval_f = (float)current_move.interval;
    float v_tol = 0.5f * velocity_coeff / (interval_f * interval_f);

    // Forward scan: skip past equal-velocity chunks until we find a change.
    int64_t time_to_change_us = 0;
    float   v_change = 0.0f;
    bool    found_change = false;

    uint16_t scan_limit = items_in_queue;
    if (scan_limit > LOOKAHEAD_MAX) scan_limit = LOOKAHEAD_MAX;

    for (uint16_t i = 0; i < scan_limit; i++) {
        MoveData m = peek(i);
        if (m.count == 0 || m.interval == NO_MOVE_QUEUED) break;

        float v_peek = (velocity_coeff / (float)m.interval) * (float)m.dir;
        if (fabsf(v_peek - v_now) > v_tol) {
            v_change = v_peek;
            found_change = true;
            break;
        }
        time_to_change_us += chunk_duration_us(m);
    }

    if (found_change) {
        // a_target reached at the END of the current chunk; the time from
        // current_move's start to v_change is duration_us + time_to_change_us,
        // but we want a_target such that velocity hits v_change one chunk-end
        // from now. Use the immediate transition time = duration_us.
        accel_target = (v_change - v_now) / ((float)duration_us * 1e-6f);
        (void)time_to_change_us;
    } else if (current_move.add != 0) {
        int32_t i_end_i = (int32_t)current_move.interval
                       + current_move.add * ((int32_t)current_move.count - 1);
        if (i_end_i <= 0) {
            accel_target = 0.0f;
        } else {
            float v_end = (velocity_coeff / (float)i_end_i)
                        * (float)current_move.dir;
            accel_target = (v_end - v_now) / ((float)duration_us * 1e-6f);
        }
    } else {
        accel_target = 0.0f;
    }

    accel_slope_per_us = (accel_target - accel) / (float)duration_us;
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

