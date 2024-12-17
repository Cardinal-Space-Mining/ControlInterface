#ifndef APPLICATION_HPP
#define APPLICATION_HPP

#include <chrono>

#include <SDL2/SDL.h>
#include <stdio.h>

#include "imgui.h"
#include "imgui_impl_sdl2.h"
#include "imgui_impl_sdlrenderer2.h"

#include "custom_elements.hpp"

#include "panel.hpp"

using namespace std::chrono_literals;

#define WIDTH 800
#define HEIGHT 600

class Application : public rclcpp::Node {
    public:
        Application() : rclcpp::Node("control_gui") 
            , wind(SDL_CreateWindow("Mission Control GUI", SDL_WINDOWPOS_CENTERED,
                                    SDL_WINDOWPOS_CENTERED, WIDTH, HEIGHT, SDL_WINDOW_RESIZABLE))
            , rend(SDL_CreateRenderer(wind, -1, SDL_RENDERER_ACCELERATED))
            // , update_timer(create_wall_timer(100ms, [this]() { this->update(); }))
        {

            if (!wind) {
                RCLCPP_ERROR(this->get_logger(), "Could not create SDL Window");
            }
            if (!rend) {
                RCLCPP_ERROR(this->get_logger(), "Could not create SDL Renderer");
            }

            if (!ImGui_ImplSDL2_InitForSDLRenderer(wind, rend)) {
                RCLCPP_ERROR(this->get_logger(), "Failed to Initialize ImGui backend");
                rclcpp::shutdown();
            }
            if (!ImGui_ImplSDLRenderer2_Init(rend)) {
                RCLCPP_ERROR(this->get_logger(), "Failed to Initialize ImGui Renderer backend");
                rclcpp::shutdown();
            }

            update_timer = create_wall_timer(10ms, [this]() { this->update(); });

            init_elements();

            RCLCPP_DEBUG(this->get_logger(), "Initialized GUI");
        }

        ~Application() {
            ImGui_ImplSDLRenderer2_Shutdown();
            ImGui_ImplSDL2_Shutdown();
            ImGui::DestroyContext();
            if (rend) {
                SDL_DestroyRenderer(rend);
            }
            if (wind) {
                SDL_DestroyWindow(wind);
            }
            SDL_Quit();
        }

    private:
        void init_elements() {
            // Robot Status Toggle Switch
            status_toggle = MultiToggle(&robot_status, status_options, "status_switch", "Status");
            status_toggle.SetColors(toggle_cols);


        }

        void update() {

            SDL_Event e;
            while (SDL_PollEvent(&e)) {
                ImGui_ImplSDL2_ProcessEvent(&e);
                this->handle_event(e);
            }

            ImGui_ImplSDLRenderer2_NewFrame();
            ImGui_ImplSDL2_NewFrame();
            ImGui::NewFrame();

            // Robot Status
            {
                ImGui::SetNextWindowPos(ImVec2(0, 400), ImGuiCond_Always);
                ImGui::SetNextWindowSize(ImVec2(300, 200), ImGuiCond_Always);
                ImGui::Begin("Robot Status Control", nullptr, 
                    ImGuiWindowFlags_NoResize | 
                    ImGuiWindowFlags_NoMove | 
                    ImGuiWindowFlags_NoCollapse);

                    status_toggle.Render();
                ImGui::End();
            }

            // Motor Status Information
            {
                ImGui::SetNextWindowPos(ImVec2(0, 0), ImGuiCond_Always);
                ImGui::SetNextWindowSize(ImVec2(300, 300), ImGuiCond_Always);
                ImGui::Begin("Motor Status Information", nullptr,
                    ImGuiWindowFlags_NoResize |
                    ImGuiWindowFlags_NoMouseInputs |
                    ImGuiWindowFlags_NoCollapse);

                    // Talon SRX - Phoenix 5 Motors
                    {
                        ImGui::Text("Talon SRX - Phoenix 5");
                    }

                    // Talon FX - Phoenix 6 Motors
                    {
                        ImGui::Text("Talon FX - Phoenix 6");
                    }

                ImGui::End();
            }

            ImGui::Render();
            SDL_RenderClear(rend);
            ImGui_ImplSDLRenderer2_RenderDrawData(ImGui::GetDrawData(), rend);
            SDL_RenderPresent(rend);
        }

        void handle_event(SDL_Event& e) {
            if (e.type == SDL_QUIT) {
                this->~Application();
            }
        }

    // SDL and ImGui Storing and Configs
    private:
        SDL_Window* wind;
        SDL_Renderer* rend;

    // Timer base
    private:
        rclcpp::TimerBase::SharedPtr update_timer;

    // Robot Status Toggle
    private:
        int robot_status = 1;
        std::vector<std::string> status_options = {"autonomous", "disabled", "teleop"};
        std::vector<BASE_COLORS> toggle_cols = {BASE_COLORS::BLUE, BASE_COLORS::RED, BASE_COLORS::GREEN};
        MultiToggle status_toggle;

    // Motor Status Info
    private:
        
};

#endif