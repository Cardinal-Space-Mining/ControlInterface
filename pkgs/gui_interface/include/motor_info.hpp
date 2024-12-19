#ifndef MOTOR_INFO_HPP
#define MOTOR_INFO_HPP

#include <algorithm>
#include <cstddef>
#include <iostream>

#include "custom_types/msg/talon_info.hpp"

using namespace custom_types::msg;
using std::vector;

class MotorInfo {
    public:
        MotorInfo(int max = 1000) : offset(0), time(0.000) {
            max_size = max;
            temp.reserve(max_size);
            bus_volt.reserve(max_size);
            out_perc.reserve(max_size);
            out_volt.reserve(max_size);
            out_curr.reserve(max_size);
            position.reserve(max_size);
            velocity.reserve(max_size);
        }

        void add_point(const TalonInfo &msg) {
            if (temp.size() < max_size) {
                temp.push_back(ImVec2(time, msg.temperature));
                bus_volt.push_back(ImVec2(time, msg.bus_voltage));
                out_perc.push_back(ImVec2(time, msg.output_percent));
                out_volt.push_back(ImVec2(time, msg.output_voltage));
                out_curr.push_back(ImVec2(time, msg.output_current));
                position.push_back(ImVec2(time, msg.position));
                velocity.push_back(ImVec2(time, msg.velocity));
            } else {
                temp[offset]     = ImVec2(time, msg.temperature);
                bus_volt[offset] = ImVec2(time, msg.bus_voltage);
                out_perc[offset] = ImVec2(time, msg.output_percent);
                out_volt[offset] = ImVec2(time, msg.output_voltage);
                out_curr[offset] = ImVec2(time, msg.output_current);
                position[offset] = ImVec2(time, msg.position);
                velocity[offset] = ImVec2(time, msg.velocity);

                offset = (offset + 1) % max_size;
            }

            time += 0.100;
        }

        void Erase() {
            if (temp.size() > 0) {
                temp.shrink(0);
                offset = 0;
            }
        }

    public:
        int offset;
        double time;
        int max_size;

    // Array buffers (for implot and storage)
    public:
        ImVector<ImVec2> temp;
        ImVector<ImVec2> bus_volt;
        ImVector<ImVec2> out_perc;
        ImVector<ImVec2> out_volt;
        ImVector<ImVec2> out_curr;
        ImVector<ImVec2> position;
        ImVector<ImVec2> velocity;
        
};

#endif