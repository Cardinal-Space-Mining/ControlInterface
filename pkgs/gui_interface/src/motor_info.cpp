#include "motor_info.hpp"

MotorInfo::MotorInfo(int max) : offset(0), time(0.000), ave_temp(0), ave_out_perc(0), ave_out_volt(0), ave_out_curr(0), ave_velocity(0)
{
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
void MotorInfo::add_point(const TalonInfo &msg)
{
    if (temp.size() < max_size)
    {
        temp.push_back(ImVec2(time, msg.temperature));
        bus_volt.push_back(ImVec2(time, msg.bus_voltage));
        out_perc.push_back(ImVec2(time, msg.output_percent));
        out_volt.push_back(ImVec2(time, msg.output_voltage));
        out_curr.push_back(ImVec2(time, msg.output_current));
        position.push_back(ImVec2(time, msg.position));
        velocity.push_back(ImVec2(time, msg.velocity));
    }
    else
    {
        temp[offset] = ImVec2(time, msg.temperature);
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
void MotorInfo::Erase()
{
    if (temp.size() > 0)
    {
        temp.shrink(0);
        offset = 0;
    }
}

void MotorInfo::moving_average()
{

    float temp_total = 0;
    float out_perc_total = 0;
    float out_volt_total = 0;
    float out_curr_total = 0;
    float velocity_total = 0;

    for (int i = 0; i < temp.size(); i++)
    {
        temp_total += temp[i].y;
        out_perc_total += out_perc[i].y;
        out_volt_total += out_volt[i].y;
        out_curr_total += out_curr[i].y;
        velocity_total += velocity[i].y;
    }

    ave_temp = temp_total / temp.size();
    ave_out_perc = out_perc_total / out_perc.size();
    ave_out_volt = out_volt_total / out_volt.size();
    ave_out_curr = out_curr_total / out_curr.size();
    ave_velocity = velocity_total / velocity.size();
}
