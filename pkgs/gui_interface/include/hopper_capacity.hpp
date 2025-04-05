#ifndef HOPPER_CAPACITY_HPP
#define HOPPER_CAPACITY_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>

class hopper_capacity {
    public:
        hopper_capacity() {}
        hopper_capacity(rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub) : capacity_sub(sub) {}

        void set_capacity(float cap);
        float get_capacity() const;
        float get_ave_offload_cap() const;
        float get_estimate_moved() const;

    private:
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr capacity_sub;
        float percent_capacity = 0; // value between 0.00 and 1.00
        float estimate_moved = 0;
        float average_offload_cap = 0;
};

#endif
