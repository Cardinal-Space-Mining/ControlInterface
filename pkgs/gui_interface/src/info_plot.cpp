#include <vector>

#include "info_plot.hpp"

InfoPlot::InfoPlot() : right_track(true), left_track(true), trencher(true), hopper_belt(true), hopper_actuator(true), temperature(true), bus_voltage(false), output_perc(false), output_volt(false), output_curr(false), velocity(false), history(0)
{
}

InfoPlot::InfoPlot(Info rt, Info lt, Info tren, Info hb, Info ha, int buff)
    : right_track(true), left_track(true), trencher(true), hopper_belt(true), hopper_actuator(true), temperature(true), bus_voltage(false), output_perc(false), output_volt(false), output_curr(false), velocity(false),
      right(std::move(rt)),
      left(std::move(lt)),
      trench(std::move(tren)),
      hop_belt(std::move(hb)),
      hop_act(std::move(ha)),
      history(buff / 10.0)
{
}

void InfoPlot::Render()
{

    int active_plots = 0;
    if (temperature)
        active_plots++;
    if (bus_voltage)
        active_plots++;
    if (output_perc)
        active_plots++;
    if (output_volt)
        active_plots++;
    if (output_curr)
        active_plots++;
    if (velocity)
        active_plots++;

    float p_wd = 0;
    float p_ht = 0;

    if (active_plots <= 2)
    { // 2 or less plots
        p_wd = 880;
        p_ht = 180;
    }
    else if (active_plots == 3)
    { // 3 plots
        p_wd = 880;
        p_ht = 135;
    }
    else if (active_plots == 4)
    { // 4 plots
        p_wd = 440;
        p_ht = 180;
    }
    else
    {
        p_wd = 440;
        p_ht = 135;
    }

    int p_idx = 0;
    ImVec2 size(p_wd, p_ht);

    ImGui::SetNextWindowPos(ImVec2(300, 525), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(INFO_WIDTH, INFO_HEIGHT), ImGuiCond_Always);
    ImGui::Begin("Real-time Motor Information", nullptr,
                 ImGuiWindowFlags_NoCollapse |
                     ImGuiWindowFlags_NoResize |
                     ImGuiWindowFlags_NoFocusOnAppearing |
                     ImGuiWindowFlags_NoBringToFrontOnFocus);

    // Temperature Plot
    {
        if (temperature)
        {
            ImPlot::BeginPlot("Temperature", size);
            ImPlot::SetupAxes(nullptr, "\u00b0C", x_flags, y_flags);
            ImPlot::SetupAxisLimits(ImAxis_Y1, 0, 80);
            ImPlot::SetupAxisLimits(ImAxis_X1, hop_act->time - history, hop_act->time, ImGuiCond_Always);

            if (right_track && !right->temp.empty())
            {
                ImPlot::SetNextLineStyle(colors[0], 0.25f);
                ImPlot::PlotLine("right track", &right->temp[0].x, &right->temp[0].y, right->temp.size(), 0, right->offset, 2 * sizeof(float));
            }

            if (left_track && !left->temp.empty())
            {
                ImPlot::SetNextLineStyle(colors[1], 0.25f);
                ImPlot::PlotLine("left track", &left->temp[0].x, &left->temp[0].y, left->temp.size(), 0, left->offset, 2 * sizeof(float));
            }

            if (trencher && !trench->temp.empty())
            {
                ImPlot::SetNextLineStyle(colors[2], 0.25f);
                ImPlot::PlotLine("trencher", &trench->temp[0].x, &trench->temp[0].y, trench->temp.size(), 0, trench->offset, 2 * sizeof(float));
            }

            if (hopper_belt && !hop_belt->temp.empty())
            {
                ImPlot::SetNextLineStyle(colors[3], 0.25f);
                ImPlot::PlotLine("hopper belt", &hop_belt->temp[0].x, &hop_belt->temp[0].y, hop_belt->temp.size(), 0, hop_belt->offset, 2 * sizeof(float));
            }

            if (hopper_actuator && !hop_act->temp.empty())
            {
                ImPlot::SetNextLineStyle(colors[4], 0.25f);
                ImPlot::PlotLine("actuators", &hop_act->temp[0].x, &hop_act->temp[0].y, hop_act->temp.size(), 0, hop_act->offset, 2 * sizeof(float));
            }

            ImPlot::EndPlot();

            p_idx++;
        }
    }

    if (active_plots > 3 && p_idx % 2 == 1)
    {
        ImGui::SameLine();
    }

    // Bus Voltage Plot
    {
        if (bus_voltage)
        {
            ImPlot::BeginPlot("Bus Voltage", size);
            ImPlot::SetupAxes(nullptr, "V", x_flags, y_flags);
            ImPlot::SetupAxisLimits(ImAxis_Y1, 0, 20);
            ImPlot::SetupAxisLimits(ImAxis_X1, hop_act->time - history, hop_act->time, ImGuiCond_Always);

            if (right_track && !right->bus_volt.empty())
            {
                ImPlot::SetNextLineStyle(colors[0], 0.25f);
                ImPlot::PlotLine("right track", &right->bus_volt[0].x, &right->bus_volt[0].y, right->bus_volt.size(), 0, right->offset, 2 * sizeof(float));
            }

            if (left_track && !left->bus_volt.empty())
            {
                ImPlot::SetNextLineStyle(colors[1], 0.25f);
                ImPlot::PlotLine("left track", &left->bus_volt[0].x, &left->bus_volt[0].y, left->bus_volt.size(), 0, left->offset, 2 * sizeof(float));
            }

            if (trencher && !trench->bus_volt.empty())
            {
                ImPlot::SetNextLineStyle(colors[2], 0.25f);
                ImPlot::PlotLine("trencher", &trench->bus_volt[0].x, &trench->bus_volt[0].y, trench->bus_volt.size(), 0, trench->offset, 2 * sizeof(float));
            }

            if (hopper_belt && !hop_belt->bus_volt.empty())
            {
                ImPlot::SetNextLineStyle(colors[3], 0.25f);
                ImPlot::PlotLine("hopper belt", &hop_belt->bus_volt[0].x, &hop_belt->bus_volt[0].y, hop_belt->bus_volt.size(), 0, hop_belt->offset, 2 * sizeof(float));
            }

            if (hopper_actuator && !hop_act->bus_volt.empty())
            {
                ImPlot::SetNextLineStyle(colors[4], 0.25f);
                ImPlot::PlotLine("actuators", &hop_act->bus_volt[0].x, &hop_act->bus_volt[0].y, hop_act->bus_volt.size(), 0, hop_act->offset, 2 * sizeof(float));
            }

            ImPlot::EndPlot();

            p_idx++;
        }
    }

    if (active_plots > 3 && p_idx % 2 == 1)
    {
        ImGui::SameLine();
    }

    // Output Percent Plot
    {
        if (output_perc)
        {
            ImPlot::BeginPlot("Output Percent", size);
            ImPlot::SetupAxes(nullptr, "%", x_flags, y_flags);
            ImPlot::SetupAxisLimits(ImAxis_Y1, -100, 100);
            ImPlot::SetupAxisLimits(ImAxis_X1, hop_act->time - history, hop_act->time, ImGuiCond_Always);

            if (right_track && !right->out_perc.empty())
            {
                ImPlot::SetNextLineStyle(colors[0], 0.25f);
                ImPlot::PlotLine("right track", &right->out_perc[0].x, &right->out_perc[0].y, right->out_perc.size(), 0, right->offset, 2 * sizeof(float));
            }

            if (left_track && !left->out_perc.empty())
            {
                ImPlot::SetNextLineStyle(colors[1], 0.25f);
                ImPlot::PlotLine("left track", &left->out_perc[0].x, &left->out_perc[0].y, left->out_perc.size(), 0, left->offset, 2 * sizeof(float));
            }

            if (trencher && !trench->out_perc.empty())
            {
                ImPlot::SetNextLineStyle(colors[2], 0.25f);
                ImPlot::PlotLine("trencher", &trench->out_perc[0].x, &trench->out_perc[0].y, trench->out_perc.size(), 0, trench->offset, 2 * sizeof(float));
            }

            if (hopper_belt && !hop_belt->out_perc.empty())
            {
                ImPlot::SetNextLineStyle(colors[3], 0.25f);
                ImPlot::PlotLine("hopper belt", &hop_belt->out_perc[0].x, &hop_belt->out_perc[0].y, hop_belt->out_perc.size(), 0, hop_belt->offset, 2 * sizeof(float));
            }

            if (hopper_actuator && !hop_act->out_perc.empty())
            {
                ImPlot::SetNextLineStyle(colors[4], 0.25f);
                ImPlot::PlotLine("actuators", &hop_act->out_perc[0].x, &hop_act->out_perc[0].y, hop_act->out_perc.size(), 0, hop_act->offset, 2 * sizeof(float));
            }

            ImPlot::EndPlot();

            p_idx++;
        }
    }

    if (active_plots > 3 && p_idx % 2 == 1)
    {
        ImGui::SameLine();
    }

    // Output Voltage Plot
    {
        if (output_volt)
        {
            ImPlot::BeginPlot("Output Voltage", size);
            ImPlot::SetupAxes(nullptr, "V", x_flags, y_flags);
            ImPlot::SetupAxisLimits(ImAxis_Y1, -20, 20);
            ImPlot::SetupAxisLimits(ImAxis_X1, hop_act->time - history, hop_act->time, ImGuiCond_Always);

            if (right_track && !right->out_volt.empty())
            {
                ImPlot::SetNextLineStyle(colors[0], 0.25f);
                ImPlot::PlotLine("right track", &right->out_volt[0].x, &right->out_volt[0].y, right->out_volt.size(), 0, right->offset, 2 * sizeof(float));
            }

            if (left_track && !left->out_volt.empty())
            {
                ImPlot::SetNextLineStyle(colors[1], 0.25f);
                ImPlot::PlotLine("left track", &left->out_volt[0].x, &left->out_volt[0].y, left->out_volt.size(), 0, left->offset, 2 * sizeof(float));
            }

            if (trencher && !trench->out_volt.empty())
            {
                ImPlot::SetNextLineStyle(colors[2], 0.25f);
                ImPlot::PlotLine("trencher", &trench->out_volt[0].x, &trench->out_volt[0].y, trench->out_volt.size(), 0, trench->offset, 2 * sizeof(float));
            }

            if (hopper_belt && !hop_belt->out_volt.empty())
            {
                ImPlot::SetNextLineStyle(colors[3], 0.25f);
                ImPlot::PlotLine("hopper belt", &hop_belt->out_volt[0].x, &hop_belt->out_volt[0].y, hop_belt->out_volt.size(), 0, hop_belt->offset, 2 * sizeof(float));
            }

            if (hopper_actuator && !hop_act->out_volt.empty())
            {
                ImPlot::SetNextLineStyle(colors[4], 0.25f);
                ImPlot::PlotLine("actuators", &hop_act->out_volt[0].x, &hop_act->out_volt[0].y, hop_act->out_volt.size(), 0, hop_act->offset, 2 * sizeof(float));
            }

            ImPlot::EndPlot();

            p_idx++;
        }
    }

    if (active_plots > 3 && p_idx % 2 == 1)
    {
        ImGui::SameLine();
    }

    // Output Current Plot
    {
        if (output_curr)
        {
            ImPlot::BeginPlot("Output Current", size);
            ImPlot::SetupAxes(nullptr, "Current", x_flags, y_flags);
            ImPlot::SetupAxisLimits(ImAxis_Y1, 0, 10);
            ImPlot::SetupAxisLimits(ImAxis_X1, hop_act->time - history, hop_act->time, ImGuiCond_Always);

            if (right_track && !right->out_curr.empty())
            {
                ImPlot::SetNextLineStyle(colors[0], 0.25f);
                ImPlot::PlotLine("right track", &right->out_curr[0].x, &right->out_curr[0].y, right->out_curr.size(), 0, right->offset, 2 * sizeof(float));
            }

            if (left_track && !left->out_curr.empty())
            {
                ImPlot::SetNextLineStyle(colors[1], 0.25f);
                ImPlot::PlotLine("left track", &left->out_curr[0].x, &left->out_curr[0].y, left->out_curr.size(), 0, left->offset, 2 * sizeof(float));
            }

            if (trencher && !trench->out_curr.empty())
            {
                ImPlot::SetNextLineStyle(colors[2], 0.25f);
                ImPlot::PlotLine("trencher", &trench->out_curr[0].x, &trench->out_curr[0].y, trench->out_curr.size(), 0, trench->offset, 2 * sizeof(float));
            }

            if (hopper_belt && !hop_belt->out_curr.empty())
            {
                ImPlot::SetNextLineStyle(colors[3], 0.25f);
                ImPlot::PlotLine("hopper belt", &hop_belt->out_curr[0].x, &hop_belt->out_curr[0].y, hop_belt->out_curr.size(), 0, hop_belt->offset, 2 * sizeof(float));
            }

            if (hopper_actuator && !hop_act->out_curr.empty())
            {
                ImPlot::SetNextLineStyle(colors[4], 0.25f);
                ImPlot::PlotLine("actuators", &hop_act->out_curr[0].x, &hop_act->out_curr[0].y, hop_act->out_curr.size(), 0, hop_act->offset, 2 * sizeof(float));
            }

            ImPlot::EndPlot();

            p_idx++;
        }
    }

    if (active_plots > 3 && p_idx % 2 == 1)
    {
        ImGui::SameLine();
    }

    // Velocity Plot
    {
        if (velocity)
        {
            ImPlot::BeginPlot("Velocity", size);
            ImPlot::SetupAxes(nullptr, "Output rps", x_flags, y_flags);
            ImPlot::SetupAxisLimits(ImAxis_Y1, -100, 100);
            ImPlot::SetupAxisLimits(ImAxis_X1, hop_act->time - history, hop_act->time, ImGuiCond_Always);

            if (right_track && !right->velocity.empty())
            {
                ImPlot::SetNextLineStyle(colors[0], 0.25f);
                ImPlot::PlotLine("right track", &right->velocity[0].x, &right->velocity[0].y, right->velocity.size(), 0, right->offset, 2 * sizeof(float));
            }

            if (left_track && !left->velocity.empty())
            {
                ImPlot::SetNextLineStyle(colors[1], 0.25f);
                ImPlot::PlotLine("left track", &left->velocity[0].x, &left->velocity[0].y, left->velocity.size(), 0, left->offset, 2 * sizeof(float));
            }

            if (trencher && !trench->velocity.empty())
            {
                ImPlot::SetNextLineStyle(colors[2], 0.25f);
                ImPlot::PlotLine("trencher", &trench->velocity[0].x, &trench->velocity[0].y, trench->velocity.size(), 0, trench->offset, 2 * sizeof(float));
            }

            if (hopper_belt && !hop_belt->velocity.empty())
            {
                ImPlot::SetNextLineStyle(colors[3], 0.25f);
                ImPlot::PlotLine("hopper belt", &hop_belt->velocity[0].x, &hop_belt->velocity[0].y, hop_belt->velocity.size(), 0, hop_belt->offset, 2 * sizeof(float));
            }

            if (hopper_actuator && !hop_act->velocity.empty())
            {
                ImPlot::SetNextLineStyle(colors[4], 0.25f);
                ImPlot::PlotLine("actuators", &hop_act->velocity[0].x, &hop_act->velocity[0].y, hop_act->velocity.size(), 0, hop_act->offset, 2 * sizeof(float));
            }

            ImPlot::EndPlot();

            p_idx++;
        }
    }

    ImGui::End();

    // Menus for what to show
    {
        ImGui::SetNextWindowPos(ImVec2(300, 545), ImGuiCond_Always);
        ImGui::SetNextWindowSize(ImVec2(150, 150), ImGuiCond_Always);
        ImGui::Begin("Motors", nullptr, ImGuiWindowFlags_NoResize);
        ImGui::Checkbox("Right Track", &right_track);
        ImGui::Checkbox("Left Track", &left_track);
        ImGui::Checkbox("Trencher", &trencher);
        ImGui::Checkbox("Hopper Belt", &hopper_belt);
        ImGui::Checkbox("Hopper Actuator", &hopper_actuator);
        ImGui::End();

        ImGui::SetNextWindowPos(ImVec2(450, 545), ImGuiCond_Always);
        ImGui::SetNextWindowSize(ImVec2(150, 170), ImGuiCond_Always);
        ImGui::Begin("Statistics", nullptr, ImGuiWindowFlags_NoResize);
        ImGui::Checkbox("Temperature", &temperature);
        ImGui::Checkbox("Bus Voltage", &bus_voltage);
        ImGui::Checkbox("Output Percent", &output_perc);
        ImGui::Checkbox("Output Voltage", &output_volt);
        ImGui::Checkbox("Output Current", &output_curr);
        ImGui::Checkbox("Velocity", &velocity);
        ImGui::End();
    }
}
