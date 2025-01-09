#ifndef MOTOR_INFO_HPP
#define MOTOR_INFO_HPP

#include <algorithm>
#include <cstddef>
#include <iostream>

#include "custom_types/msg/talon_info.hpp"

using namespace custom_types::msg;
using std::vector;

/*
 * Stores and updates motor information. Can be used as a rolling graph.
 * 
 * The `max_size` parameter can be used to adjust how many data points are stored
 * before erasing the front and adding to the back. This should be calculated
 * based on the how long (history) of points shown and how often data is expected
 * (i.e. a history of 30 seconds and data every 10th of a second would be a buffer
 * of 300 or a max size of 300).
 * 
 * Moving average calculations are included, will be recalculated upon each new data
 * point collected. Calculation: moving average = total / max_size (this should be
 * adjusted to reduce runtime)  
*/
class MotorInfo {
    public:
        /*
         * Constructor for the MotorInfo class.
         */
        MotorInfo(int max = 1000) : offset(0), time(0.000), ave_temp(0), ave_out_perc(0)
                                  , ave_out_volt(0), ave_out_curr(0), ave_velocity(0) {
            max_size = max;
            temp.reserve(max_size);
            bus_volt.reserve(max_size);
            out_perc.reserve(max_size);
            out_volt.reserve(max_size);
            out_curr.reserve(max_size);
            position.reserve(max_size);
            velocity.reserve(max_size);
        }

        /*
         * Adds new data point if size is under max_size or replaces data
         * at offset increasing the offset value. Offset will reset when
         * the max_size has been reached.
         */
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

            moving_average();
        }

        /*
         * Simple function to erase used memory in vectors. 
         */
        void Erase() {
            if (temp.size() > 0) {
                temp.shrink(0);
                offset = 0;
            }
        }

    private:
        /*
         * Calculates moving averages based on current data points. 
         */
        void moving_average() {

            float temp_total     = 0;
            float out_perc_total = 0;
            float out_volt_total = 0;
            float out_curr_total = 0;
            float velocity_total = 0;

            for (int i = 0; i < temp.size(); i++) {
                temp_total      += temp[i].y;
                out_perc_total  += out_perc[i].y;
                out_volt_total  += out_volt[i].y;
                out_curr_total  += out_curr[i].y;
                velocity_total  += velocity[i].y;
            }

            ave_temp     = temp_total / temp.size();
            ave_out_perc = out_perc_total / out_perc.size();
            ave_out_volt = out_volt_total / out_volt.size();
            ave_out_curr = out_curr_total / out_curr.size();
            ave_velocity = velocity_total / velocity.size();
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
        
    public:
        float ave_temp, ave_out_perc, ave_out_volt, 
              ave_out_curr, ave_velocity;

};

#endif