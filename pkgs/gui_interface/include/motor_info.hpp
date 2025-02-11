#ifndef MOTOR_INFO_HPP
#define MOTOR_INFO_HPP

#include <algorithm>
#include <cstddef>
#include <iostream>

#include "imgui.h"

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
class MotorInfo
{
public:
    /*
     * Constructor for the MotorInfo class.
     */
    MotorInfo(int max = 1000);

    /*
     * Adds new data point if size is under max_size or replaces data
     * at offset increasing the offset value. Offset will reset when
     * the max_size has been reached.
     */
    void add_point(const TalonInfo &msg);

    /*
     * Simple function to erase used memory in vectors.
     */
    void Erase();

private:
    /*
     * Calculates moving averages based on current data points.
     */
    void moving_average();

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