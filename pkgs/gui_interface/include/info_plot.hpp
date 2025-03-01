#ifndef INFO_PLOT_HPP
#define INFO_PLOT_HPP

#include <vector>
#include <array>

#include "implot.h"
#include "implot_internal.h"

#include "motor_info.hpp"

using std::shared_ptr;
using std::vector;

using Info = shared_ptr<MotorInfo>;

class InfoPlot
{
public:
    constexpr static int INFO_WIDTH{900};
    constexpr static int INFO_HEIGHT{485};
    constexpr static float width = 150.0f;

public:
    InfoPlot();

    InfoPlot(Info rt, Info lt, Info tren, Info hb, Info ha, int buff);

    void Render();

    // Checkbox options for what motors to show
private:
    bool right_track;
    bool left_track;
    bool trencher;
    bool hopper_belt;
    bool hopper_actuator;

    // Checkbox options for what stat's to show
private:
    bool temperature;
    bool bus_voltage;
    bool output_perc;
    bool output_volt;
    bool output_curr;
    bool velocity;

    // Positions, sizes and colors of Plots and lines.
private:
    static constexpr std::array<ImVec4, 5> colors = {
        ImVec4(0.9f, 0.1f, 0.1f, 0.5f), // Color for Right track     (id:0)
        ImVec4(0.1f, 0.9f, 0.1f, 0.5f), // Color for Left track      (id:1)
        ImVec4(0.1f, 0.1f, 0.9f, 0.5f), // Color for Trencher        (id:2)
        ImVec4(0.8f, 0.1f, 0.8f, 0.5f), // Color for Hopper Belt     (id:3)
        ImVec4(0.1f, 0.8f, 0.8f, 0.5f)  // Color for Hopper Actuator (id:4)
    };

    // Motor Info Objects
private:
    Info right;
    Info left;
    Info trench;
    Info hop_belt;
    Info hop_act;

private:
    const static ImPlotAxisFlags y_flags = ImPlotAxisFlags_Lock;
    const static ImPlotAxisFlags x_flags = ImPlotAxisFlags_NoTickLabels;

    // How much of past data shown in seconds (history)
    // Note: changing history requireds change in buffer size
    float history = 60.0f;
};

#endif