#ifndef HOPPER_CAPACITY_HPP
#define HOPPER_CAPACITY_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>

class hopper_capacity {
    public:
        hopper_capacity() : capacity(0), estimate_moved(0) {}
        hopper_capacity(rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub) : capacity_sub(sub), capacity(0), estimate_moved(0) {}

        const double get_capacity();

        void calculate_capacity(const std_msgs::msg::Float32 &msg);

    private:
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr capacity_sub;
        double capacity;
        double estimate_moved;
};

#endif
