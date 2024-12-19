#ifndef APPLICATION_HPP
#define APPLICATION_HPP

#include <chrono>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int8.hpp>

#include "custom_types/msg/talon_info.hpp"

#include <SDL2/SDL.h>
#include <stdio.h>

#include "implot.h"
#include "implot_internal.h"

#include "imgui.h"
#include "imgui_impl_sdl2.h"
#include "imgui_impl_sdlrenderer2.h"

#include "custom_elements.hpp"

#include "motor_info.hpp"

using std::vector;
using namespace std::chrono_literals;
using namespace custom_types::msg;

#define WIDTH 800
#define HEIGHT 600
#define BUFFER_SIZE 250

class Application : public rclcpp::Node {
    public:
        Application() : rclcpp::Node("control_gui") 
            , wind(SDL_CreateWindow("Mission Control GUI", SDL_WINDOWPOS_CENTERED,
                                    SDL_WINDOWPOS_CENTERED, WIDTH, HEIGHT, SDL_WINDOW_RESIZABLE))
            , rend(SDL_CreateRenderer(wind, -1, SDL_RENDERER_ACCELERATED))
            // Motor info Subscribers
            , track_right_info(this->create_subscription<TalonInfo>(
                "track_right_info", 10, [this](const TalonInfo &msg)
                { update_info(msg, 0); }))
            , track_left_info(this->create_subscription<TalonInfo>(
                "track_left_info", 10, [this](const TalonInfo &msg)
                { update_info(msg, 1); }))
            , trencher_info(this->create_subscription<TalonInfo>(
                "hopper_info", 10, [this](const TalonInfo &msg)
                { update_info(msg, 2); }))
            , hopper_belt_info(this->create_subscription<TalonInfo>(
                "hopper_belt_info", 10, [this](const TalonInfo &msg)
                { update_info(msg, 3); }))
            , hopper_actuator_info(this->create_subscription<TalonInfo>(
                "hopper_info", 10, [this](const TalonInfo &msg)
                { update_info(msg, 4); }))
            // Robot Status Publisher
            , robot_status_pub(this->create_publisher<std_msgs::msg::Int8>(
                "robot_status", 10))
            // Motor Info Class init's
            , right_track(BUFFER_SIZE)
            , left_track(BUFFER_SIZE)
            , trencher(BUFFER_SIZE)
            , hopper_belt(BUFFER_SIZE)
            , hopper_actuator(BUFFER_SIZE)
        {

            if (!wind) {
                RCLCPP_ERROR(this->get_logger(), "Could not create SDL Window");
            }
            if (!rend) {
                RCLCPP_ERROR(this->get_logger(), "Could not create SDL Renderer");
            }

            // Initialize ImGui
            IMGUI_CHECKVERSION();
            ImGui::CreateContext();
            ImGuiIO& io = ImGui::GetIO(); 
            (void)io;
            ImGui::StyleColorsDark();

            ImPlot::CreateContext();

            if (!ImGui_ImplSDL2_InitForSDLRenderer(wind, rend)) {
                RCLCPP_ERROR(this->get_logger(), "Failed to Initialize ImGui backend");
                rclcpp::shutdown();
            }
            if (!ImGui_ImplSDLRenderer2_Init(rend)) {
                RCLCPP_ERROR(this->get_logger(), "Failed to Initialize ImGui Renderer backend");
                rclcpp::shutdown();
            }

            // Initializing Timers for update gui and sending robot status
            update_timer        = create_wall_timer(25ms, [this]() { this->update(); });
            status_update_timer = create_wall_timer(100ms, [this]() {
                std_msgs::msg::Int8 state;
                state.data = static_cast<int>(robot_status);
                robot_status_pub->publish(state);
            });

            init_elements();

            RCLCPP_DEBUG(this->get_logger(), "Initialized GUI");
        }

        ~Application() {
            ImGui_ImplSDLRenderer2_Shutdown();
            ImGui_ImplSDL2_Shutdown();
            ImGui::DestroyContext();
            ImPlot::DestroyContext();
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
                ImGui::SetNextWindowSize(ImVec2(300, 350), ImGuiCond_Always);
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

            // Motor Info Status's
            {
                RCLCPP_DEBUG(this->get_logger(), "Starting plot panel");
                ImGui::SetNextWindowPos(ImVec2(300, 0), ImGuiCond_Always);
                ImGui::SetNextWindowSize(ImVec2(500, 350), ImGuiCond_Always);
                ImGui::Begin("Hopper Actuator Info Plot", nullptr,
                    ImGuiWindowFlags_NoResize |
                    ImGuiWindowFlags_NoMouseInputs |
                    ImGuiWindowFlags_NoCollapse);
                
                {
                    static ImPlotAxisFlags y_flags = ImPlotAxisFlags_NoTickLabels;
                    static ImPlotAxisFlags x_flags = ImPlotAxisFlags_NoTickLabels;
                    static float history = 25.0f;
                    ImPlot::BeginPlot("Hopper Actuator", ImVec2(-1, 150));
                        ImPlot::SetupAxes(nullptr, nullptr, x_flags, y_flags);
                        ImPlot::SetupAxisLimits(ImAxis_Y1, 0, 120);
                        ImPlot::SetupAxisLimits(ImAxis_X1, hopper_actuator.time-history, hopper_actuator.time, ImGuiCond_Always);
                        
                        if (hopper_actuator.temp.size() > 0) {
                            ImPlot::PlotLine("temperature"
                                                , &hopper_actuator.temp[0].x
                                                , &hopper_actuator.temp[0].y
                                                , hopper_actuator.temp.size()
                                                , 0
                                                , hopper_actuator.offset
                                                , 2*sizeof(float));
                            ImPlot::PlotLine("voltage"
                                                , &hopper_actuator.bus_volt[0].x
                                                , &hopper_actuator.bus_volt[0].y
                                                , hopper_actuator.bus_volt.size()
                                                , 0
                                                , hopper_actuator.offset
                                                , 2*sizeof(float));
                            ImPlot::SetNextLineStyle(ImVec4(0.1f, 0.9f, 0.1f, 0.2f), 0.25f);
                            ImPlot::PlotLine("current"
                                                , &hopper_actuator.out_curr[0].x
                                                , &hopper_actuator.out_curr[0].y
                                                , hopper_actuator.out_curr.size()
                                                , 0
                                                , hopper_actuator.offset
                                                , 2*sizeof(float));
                            
                        }

                    ImPlot::EndPlot();
                }   

                ImGui::End();

                RCLCPP_DEBUG(this->get_logger(), "Finished plot panel");
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

        void update_info(const TalonInfo &msg, const int id) {
            switch(id) {
                case 0:
                    // right_track.add_message(msg);
                    break;
                case 1:
                    // left_track.add_message(msg);
                    break;
                case 2:
                    // trencher.add_message(msg);
                    break;
                case 3:
                    // hopper_belt.add_message(msg);
                    break;
                case 4:
                    // hopper_actuator.add_message(msg);
                    hopper_actuator.add_point(msg);
                    break;
            }
        }

    // SDL and ImGui Storing and Configs
    private:
        SDL_Window* wind;
        SDL_Renderer* rend;

    // Timer base
    private:
        rclcpp::TimerBase::SharedPtr update_timer;
        rclcpp::TimerBase::SharedPtr status_update_timer;

    // Robot Status Toggle (ImGui)
    private:
        int robot_status = 1;
        std::vector<std::string> status_options = {"autonomous", "disabled", "teleop"};
        std::vector<BASE_COLORS> toggle_cols = {BASE_COLORS::BLUE, BASE_COLORS::RED, BASE_COLORS::GREEN};
        MultiToggle status_toggle;

    // Motor Status Info (ImGui/ImPlot)
    private:

    // Publisher and Subscriber Nodes
    private:
        rclcpp::Subscription<TalonInfo>::SharedPtr track_right_info;
        rclcpp::Subscription<TalonInfo>::SharedPtr track_left_info;
        rclcpp::Subscription<TalonInfo>::SharedPtr trencher_info;
        rclcpp::Subscription<TalonInfo>::SharedPtr hopper_belt_info;
        rclcpp::Subscription<TalonInfo>::SharedPtr hopper_actuator_info;

        rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr robot_status_pub;    

    // Motor Data classes
    private:
        MotorInfo right_track;
        MotorInfo left_track;
        MotorInfo trencher;
        MotorInfo hopper_belt;
        MotorInfo hopper_actuator;

};

#endif