#ifndef PANEL_HPP
#define PANEL_HPP

#include <SDL2/SDL.h>

#include "imgui.h"
#include "imgui_impl_sdl2.h"
#include "imgui_impl_sdlrenderer2.h"

#include "custom_elements.hpp"
#include "formatted_elements.hpp"

/*
 * TODO: Implement Rendering all elements for panel
 * 
 * Elements:
 * - Robot Control Window
 *   - Toggle Switch TODO: FIX THIS
 *   - maybe more
 * - Motor Stat Plot Window (Phoenix 5)
 *   - Expandable box for each statistic (choose what to view)
 * - Motor Stat Plot Window (Phoenix 6)
 *   - Expandable box for each statistic (choose what to view)
 * - Hopper Capacity Estimate Window
 *   - Current Capacity
 *   - Approximate amount of Material transported
 * - 2D/3D map of arena
*/

class Panel {
    public:

        Panel() {}

        Panel(SDL_Window* wind, SDL_Renderer* rend) : wind(wind), rend(rend), status() {

            if (!wind || !rend) {
                throw std::runtime_error("Invalid SDL_Window or SDL_Renderer passed to Panel");
            }

            IMGUI_CHECKVERSION();
            ImGui::CreateContext();
            ImGuiIO& io = ImGui::GetIO();
            (void)io;
            ImGui::StyleColorsDark();

            if (!ImGui_ImplSDL2_InitForSDLRenderer(wind, rend)) {
                throw std::runtime_error("Failed to initialize ImGui SDL2 Backend");
            }
            if (!ImGui_ImplSDLRenderer2_Init(rend)) {
                throw std::runtime_error("Failed to initialize ImGui SDL2 Renderer Backend");
            }

            initialized = true;

            // Init Elements in Panel
            // Toggle switch
            status = MultiToggle(&toggle_status, status_options, "status switch", "status_switch");
            status.SetColors(toggle_colors);
        }

        ~Panel() {
            if (initialized) {
                ImGui_ImplSDLRenderer2_Shutdown();
                ImGui_ImplSDL2_Shutdown();
                ImGui::DestroyContext();
                initialized = false;
            }
        }

        void Render() {

            if (!initialized || !wind || !rend) {
                throw std::runtime_error("Panel not initialized before calling Render");
            }

            ImGui_ImplSDL2_NewFrame();
            ImGui_ImplSDLRenderer2_NewFrame();
            ImGui::NewFrame();

            ImGui::Begin("Example window");
            ImGui::Text("Hello world");
            status.Render();
            ImGui::End();

            ImGui_ImplSDLRenderer2_RenderDrawData(ImGui::GetDrawData(), rend);
        }


    private:
        SDL_Window* wind;
        SDL_Renderer* rend;

        bool initialized = false;

    // status toggle switch TODO: fix this
    private:
        int toggle_status = 1;
        std::vector<std::string> status_options = {"autonomous", "disabled", "teleop"};
        std::vector<BASE_COLORS> toggle_colors = {BASE_COLORS::BLUE, BASE_COLORS::RED, BASE_COLORS::GREEN};
        MultiToggle status;
};

#endif