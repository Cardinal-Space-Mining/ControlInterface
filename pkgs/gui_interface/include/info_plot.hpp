#ifndef INFO_PLOT_HPP
#define INFO_PLOT_HPP

#include <vector>

#include "implot.h"
#include "implot_internal.h"

#include "motor_info.hpp"

using std::vector;
using std::shared_ptr;

using Info = shared_ptr<MotorInfo>;

#define INFO_WIDTH 900
#define INFO_HEIGHT 450
const float width = 150.0f;

class InfoPlot {
    public:
        InfoPlot() : right_track(true), left_track(true), trencher(true)
                   , hopper_belt(true), hopper_actuator(true)
                   , temperature(true), bus_voltage(false)
                   , output_perc(false), output_volt(false)
                   , output_curr(false), velocity(false) 
        {}

        InfoPlot(Info rt, Info lt, Info tren, Info hb, Info ha) 
                   : right_track(true), left_track(true), trencher(true)
                   , hopper_belt(true), hopper_actuator(true)
                   , temperature(true), bus_voltage(false)
                   , output_perc(false), output_volt(false)
                   , output_curr(false), velocity(false) 
        {
            right = rt;
            left = lt;
            trench = tren;
            hop_belt = hb;
            hop_act = ha;
        }

        void Render() {

            int active_plots = 0;
            if (temperature)    active_plots++;
            if (bus_voltage)    active_plots++;
            if (output_perc)    active_plots++;
            if (output_volt)    active_plots++;
            if (output_curr)    active_plots++;
            if (velocity)       active_plots++;

            const int cols = active_plots < 3 ? 1 : 2;
            const int rows = (active_plots + cols - 1) / cols;
            const float p_wd = INFO_WIDTH / cols * 0.98f;
            const float p_ht = INFO_HEIGHT / rows * 0.8f;
            int p_idx = 0;
            ImVec2 size(p_wd, p_ht);

            ImGui::SetNextWindowPos(ImVec2(300, 0), ImGuiCond_Always);
            ImGui::SetNextWindowSize(ImVec2(INFO_WIDTH, INFO_HEIGHT), ImGuiCond_Always);
            ImGui::Begin("Real-time Motor Information", nullptr, 
                            ImGuiWindowFlags_NoCollapse | 
                            ImGuiWindowFlags_NoResize | 
                            ImGuiWindowFlags_NoFocusOnAppearing |
                            ImGuiWindowFlags_NoBringToFrontOnFocus);
            
            static ImPlotAxisFlags y_flags;
            // static ImPlotAxisFlags x_flags = ImPlotAxisFlags_NoTickLabels;
            static ImPlotAxisFlags x_flags;
            // How much of past data shown in seconds (history)
            static float history = 25.0f;

            // Temperature Plot
            {
                if (temperature) {  
                    ImVec2 pos((p_idx % cols) * INFO_WIDTH / cols, (p_idx / cols) * INFO_HEIGHT / rows);    

                    ImPlot::BeginPlot("Temperature's", size);
                        ImPlot::SetupAxes(nullptr, "\u00b0Celcius", x_flags, y_flags);
                        ImPlot::SetupAxisLimits(ImAxis_Y1, 0, 120);
                        ImPlot::SetupAxisLimits(ImAxis_X1, hop_act->time - history, hop_act->time, ImGuiCond_Always);

                        if (right_track && right->temp.size() > 0) {
                            ImPlot::SetNextLineStyle(colors[0], 0.25f);
                            ImPlot::PlotLine("right track"
                                        , &right->temp[0].x
                                        , &right->temp[0].y
                                        , right->temp.size()
                                        , 0, hop_act->offset
                                        , 2 * sizeof(float));
                        }

                        if (left_track && left->temp.size() > 0) {
                            ImPlot::SetNextLineStyle(colors[1], 0.25f);
                            ImPlot::PlotLine("left track"
                                        , &left->temp[0].x
                                        , &left->temp[0].y
                                        , left->temp.size()
                                        , 0, left->offset
                                        , 2 * sizeof(float));
                        }

                        if (trencher && trench->temp.size() > 0) {
                            ImPlot::SetNextLineStyle(colors[2], 0.25f);
                            ImPlot::PlotLine("trencher"
                                        , &trench->temp[0].x
                                        , &trench->temp[0].y
                                        , trench->temp.size()
                                        , 0, trench->offset
                                        , 2 * sizeof(float));
                        }

                        if (hopper_belt && hop_belt->temp.size() > 0) {
                            ImPlot::SetNextLineStyle(colors[3], 0.25f);
                            ImPlot::PlotLine("hopper belt"
                                        , &hop_belt->temp[0].x
                                        , &hop_belt->temp[0].y
                                        , hop_belt->temp.size()
                                        , 0, hop_belt->offset
                                        , 2 * sizeof(float));
                        }

                        if (hopper_actuator && hop_act->temp.size() > 0) {
                            ImPlot::SetNextLineStyle(colors[4], 0.25f);
                            ImPlot::PlotLine("actuators"
                                        , &hop_act->temp[0].x
                                        , &hop_act->temp[0].y
                                        , hop_act->temp.size()
                                        , 0, hop_act->offset
                                        , 2 * sizeof(float));
                        }

                    ImPlot::EndPlot();
                }
            }

            // Bus Voltage Plot
            {
                if (bus_voltage) {
                    ImPlot::BeginPlot("Bus Voltage", size);
                        ImPlot::SetupAxes(nullptr, "Voltage", x_flags, y_flags);
                        ImPlot::SetupAxisLimits(ImAxis_Y1, 0, 120);
                        ImPlot::SetupAxisLimits(ImAxis_X1, hop_act->time - history, hop_act->time, ImGuiCond_Always);
                    ImPlot::EndPlot();
                }
            }

            // Output Percent Plot
            {
                if (output_perc) {
                    ImPlot::BeginPlot("Output Percent", size);
                        ImPlot::SetupAxes(nullptr, "Output %", x_flags, y_flags);
                        ImPlot::SetupAxisLimits(ImAxis_Y1, 0, 120);
                        ImPlot::SetupAxisLimits(ImAxis_X1, hop_act->time - history, hop_act->time, ImGuiCond_Always);
                    ImPlot::EndPlot();
                }
            }

            // Output Voltage Plot
            {

            }

            // Output Current Plot
            {

            }

            // Velocity Plot
            {

            }
            
            ImGui::End();

            // Menus for what to show
            {
                ImGui::SetNextWindowPos(ImVec2(300, 20), ImGuiCond_Always);
                ImGui::SetNextWindowSize(ImVec2(150, 150), ImGuiCond_Always);
                ImGui::Begin("Motors", nullptr, ImGuiWindowFlags_NoResize);
                    ImGui::Checkbox("Right Track", &right_track);
                    ImGui::Checkbox("Left Track", &left_track);
                    ImGui::Checkbox("Trencher", &trencher);
                    ImGui::Checkbox("Hopper Belt", &hopper_belt);
                    ImGui::Checkbox("Hopper Actuator", &hopper_actuator);
                ImGui::End();

                ImGui::SetNextWindowPos(ImVec2(450, 20), ImGuiCond_Always);
                ImGui::SetNextWindowSize(ImVec2(150, 200), ImGuiCond_Always);
                ImGui::Begin("Statistics");
                    ImGui::Checkbox("Temperature", &temperature);
                    ImGui::Checkbox("Bus Voltage", &bus_voltage);
                    ImGui::Checkbox("Output Percent", &output_perc);
                    ImGui::Checkbox("Output Voltage", &output_volt);
                    ImGui::Checkbox("Output Current", &output_curr);
                    ImGui::Checkbox("Velocity", &velocity);
                ImGui::End();
            }
        }


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
        vector<ImVec4> colors = {
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
};

#endif