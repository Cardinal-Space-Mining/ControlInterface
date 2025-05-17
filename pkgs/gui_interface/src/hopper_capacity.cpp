#include "hopper_capacity.hpp"

float hopper_capacity::get_capacity() const {
    return this->percent_capacity * 100;
}

float hopper_capacity::get_ave_offload_cap() const {
    return this->average_offload_cap;
}

float hopper_capacity::get_estimate_moved() const {
    return this->estimate_moved;
}

void hopper_capacity::set_capacity(float cap) {
    if (cap < percent_capacity - 0.55f) {   // post offload cycle
        estimate_moved += percent_capacity;
        percent_capacity = 0.0f;
    } else {
        percent_capacity = cap;
    }
}
