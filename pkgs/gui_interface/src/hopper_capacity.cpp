#include "hopper_capacity.hpp"

double hopper_capacity::get_capacity() const {
    return this->capacity;
}

void hopper_capacity::calculate_capacity(const std_msgs::msg::Float32 &msg) {
    /* May need to do math here */

    estimate_moved += msg.data;
}
