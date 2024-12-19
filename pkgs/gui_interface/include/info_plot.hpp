#ifndef INFO_PLOT_HPP
#define INFO_PLOT_HPP

#include "implot.h"
#include "implot_internal.h"

#include "motor_info.hpp"

class InfoPlot {
    public:
        InfoPlot() {}

        void Render() {
            ImGui::Begin("Outer Frame");

                if (ImGui::BeginChild("InnerFrame", ImVec2(200, 300), false)) {
                    ImGui::Text("Content within child");
                    ImGui::Button("Inner button");

                    ImGui::EndChild();
                }

            ImGui::End();
        }

    private:

};

#endif