#include <chrono>
#include <random>

#include <rclcpp/rclcpp.hpp>

#include "custom_types/msg/talon_info.hpp"

using std::make_shared;
using namespace std::chrono_literals;
using namespace custom_types::msg;

class InfoGen : public rclcpp::Node {
    public:
        InfoGen() : rclcpp::Node("info_gen")
            , info_pub_act(this->create_publisher<TalonInfo>(
                "hopper_info", 10))
            , info_pub_rt(this->create_publisher<TalonInfo>(
                "track_right_info", 10))
            , pub_timer(create_wall_timer(100ms, [this]() {
                this->publish(); }))
        {
            temps = std::uniform_real_distribution<>(30.0, 70.0);
            bus_volt = std::uniform_real_distribution<>(14.0, 16.9);
            out_perc = std::uniform_real_distribution<>(0.0, 100.0);
            out_volt = std::uniform_real_distribution<>(14.0, 16.9);
            out_curr = std::uniform_real_distribution<>(0.0, 120.0);
            position = std::uniform_real_distribution<>(0.0, 100.0);
            velocity = std::uniform_real_distribution<>(-100.0, 100.0);
        }

    private:
        void publish() {

            std::random_device rd;
            std::mt19937 gen(rd());

            TalonInfo msg;
            msg.temperature     = temps(gen);
            msg.bus_voltage     = bus_volt(gen);
            msg.output_percent  = out_perc(gen);
            msg.output_voltage  = out_volt(gen);
            msg.output_current  = out_curr(gen);
            msg.position        = position(gen);
            msg.velocity        = velocity(gen);
            info_pub_act->publish(msg);

            msg.temperature     = temps(gen);
            msg.bus_voltage     = bus_volt(gen);
            msg.output_percent  = out_perc(gen);
            msg.output_voltage  = out_volt(gen);
            msg.output_current  = out_curr(gen);
            msg.position        = position(gen);
            msg.velocity        = velocity(gen);
            info_pub_rt->publish(msg);
        }

    private:
        rclcpp::Publisher<TalonInfo>::SharedPtr info_pub_act;
        rclcpp::Publisher<TalonInfo>::SharedPtr info_pub_rt;
        rclcpp::TimerBase::SharedPtr pub_timer;

    private:
        std::mt19937 gen;
        std::uniform_real_distribution<> temps;
        std::uniform_real_distribution<> bus_volt;
        std::uniform_real_distribution<> out_perc;
        std::uniform_real_distribution<> out_volt;
        std::uniform_real_distribution<> out_curr;
        std::uniform_real_distribution<> position;
        std::uniform_real_distribution<> velocity;

};

int main(int argc, char* argv[]) {

    rclcpp::init(argc, argv);

    auto node = make_shared<InfoGen>();

    rclcpp::spin(node);
    rclcpp::shutdown();

    node = nullptr;

    return 0;
}