#ifndef APPLICATION_HPP
#define APPLICATION_HPP

#include <chrono>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int8.hpp>
#include <sensor_msgs/msg/image.hpp>

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
#include "info_plot.hpp"

using std::vector;
using std::shared_ptr;
using namespace std::chrono_literals;
using namespace custom_types::msg;
using Info = shared_ptr<MotorInfo>;

#define WIDTH 1200
#define HEIGHT 900
#define BUFFER_SIZE 600

class Application : public rclcpp::Node {
    public:
        Application() : rclcpp::Node("control_gui") 
            , wind(SDL_CreateWindow("Mission Control GUI", SDL_WINDOWPOS_CENTERED,
                                    SDL_WINDOWPOS_CENTERED, WIDTH, HEIGHT, SDL_WINDOW_RESIZABLE | SDL_WINDOW_OPENGL))
            , rend(SDL_CreateRenderer(wind, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC))
            // Motor info Subscribers
            , track_right_sub(this->create_subscription<TalonInfo>(
                "track_right_info", 10, [this](const TalonInfo &msg)
                { update_info(msg, 0); }))
            , track_left_sub(this->create_subscription<TalonInfo>(
                "track_left_info", 10, [this](const TalonInfo &msg)
                { update_info(msg, 1); }))
            , trencher_sub(this->create_subscription<TalonInfo>(
                "trencher_info", 10, [this](const TalonInfo &msg)
                { update_info(msg, 2); }))
            , hopper_belt_sub(this->create_subscription<TalonInfo>(
                "hopper_belt_info", 10, [this](const TalonInfo &msg)
                { update_info(msg, 3); }))
            , hopper_actuator_sub(this->create_subscription<TalonInfo>(
                "hopper_info", 10, [this](const TalonInfo &msg)
                { update_info(msg, 4); }))
            // Robot Status Publisher
            , robot_status_pub(this->create_publisher<std_msgs::msg::Int8>(
                "robot_status", 10))
            , status_toggle()
            , image_sub(this->create_subscription<sensor_msgs::msg::Image>(
                "camera/image_raw", 10, [this](const sensor_msgs::msg::Image &msg)
                { 
                    if (msg.encoding != "bgr8") {
                        RCLCPP_ERROR(this->get_logger(), "Unsupported encoding: %s", msg.encoding.c_str());
                    }
                    this->imageCallback(msg);
                    // RCLCPP_INFO(this->get_logger(), "\rReceived an image!");
                }))
            , cam_width(640)
            , cam_height(480)
        {

            if (!wind) {
                RCLCPP_ERROR(this->get_logger(), "Could not create SDL Window");
                return;
            }
            if (!rend) {
                RCLCPP_ERROR(this->get_logger(), "Could not create SDL Renderer");
                return;
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
            if (video_tex) SDL_DestroyTexture(video_tex);
            ImGui_ImplSDLRenderer2_Shutdown();
            ImGui_ImplSDL2_Shutdown();
            ImPlot::DestroyContext();
            ImGui::DestroyContext();
            if (rend) SDL_DestroyRenderer(rend);
            if (wind) SDL_DestroyWindow(wind);
            SDL_Quit();
        }

    private:
        void init_elements() {
            // Robot Status Toggle Switch
            status_toggle = MultiToggle(&robot_status, status_options, "status_switch", "Status");
            status_toggle.SetColors(toggle_cols);

            right_track     = std::make_shared<MotorInfo>(BUFFER_SIZE);
            left_track      = std::make_shared<MotorInfo>(BUFFER_SIZE);
            trencher        = std::make_shared<MotorInfo>(BUFFER_SIZE);
            hopper_belt     = std::make_shared<MotorInfo>(BUFFER_SIZE);
            hopper_actuator = std::make_shared<MotorInfo>(BUFFER_SIZE);

            // Info Plots
            plots = InfoPlot(right_track, left_track, trencher, hopper_belt, hopper_actuator, BUFFER_SIZE);
        }

        void update() {

            SDL_Event e;
            while (SDL_PollEvent(&e)) {
                ImGui_ImplSDL2_ProcessEvent(&e);

                if (e.type == SDL_QUIT) {
                    this->~Application();
                    // rclcpp::shutdown();
                    return;
                }
            }

            ImGui_ImplSDLRenderer2_NewFrame();
            ImGui_ImplSDL2_NewFrame();
            ImGui::NewFrame();

            // Robot Status
            {
                ImGui::SetNextWindowPos(ImVec2(0, 700), ImGuiCond_Always);
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

                        ImGui::Text("Hopper Actuator:");
                        if (hopper_actuator->position.size() > 0) {
                            ImGui::Text("\tPosition: %.3f", hopper_actuator->position.back().y);
                        } else {
                            ImGui::Text("\tPosition: ");
                        }

                        ImGui::Text("\tAverage Temp: %.2f", hopper_actuator->ave_temp);            
                    }

                    // Talon FX - Phoenix 6 Motors
                    {
                        ImGui::Text("Talon FX - Phoenix 6");
                    }

                ImGui::End();
            }

            // Motor Data plots
            {
                plots.Render();
            }

            // Camera feed
            {
                ImGui::SetNextWindowPos(ImVec2(300, 450), ImGuiCond_Always);
                ImGui::SetNextWindowSize(ImVec2(900, 450), ImGuiCond_Always);
                ImGui::Begin("Video Display");
                    ImGui::Image((ImTextureID) video_tex, ImVec2(cam_width, cam_height));
                ImGui::End();
            }

            ImGui::Render();
            SDL_SetRenderDrawColor(rend, 0, 0, 0, 255);
            SDL_RenderClear(rend);
            ImGui_ImplSDLRenderer2_RenderDrawData(ImGui::GetDrawData(), rend);
            SDL_RenderPresent(rend);
        }

        void update_info(const TalonInfo &msg, const int id) {
            switch(id) {
                case 0:
                    right_track->add_point(msg);
                    break;
                case 1:
                    left_track->add_point(msg);
                    break;
                case 2:
                    trencher->add_point(msg);
                    break;
                case 3:
                    hopper_belt->add_point(msg);
                    break;
                case 4:
                    hopper_actuator->add_point(msg);
                    break;
            }
        }

        void imageCallback(const sensor_msgs::msg::Image &msg) {
            if (!video_tex || msg.width != cam_width || msg.height != cam_height) {

                RCLCPP_DEBUG(this->get_logger(), "first if conditional");

                cam_width = msg.width;
                cam_height = msg.height;

                if (video_tex) {
                    SDL_DestroyTexture(video_tex);
                }

                video_tex = SDL_CreateTexture(
                    rend,
                    SDL_PIXELFORMAT_RGB24,
                    SDL_TEXTUREACCESS_STREAMING,
                    cam_width, cam_height);

                if (!video_tex) {
                    RCLCPP_ERROR(this->get_logger(), "Failed to create SDL Texture for live video");
                    return;
                }
            }

            void* pixels;
            int pitch;
            if (SDL_LockTexture(video_tex, nullptr, &pixels, &pitch) == 0) {
                uint8_t* dst = static_cast<uint8_t*>(pixels);
                const uint8_t* src = msg.data.data();

                for (int y = 0; y < cam_height; ++y) {
                    for (int x = 0; x < cam_width; ++x) {
                        int idx = y * pitch + x * 3;
                        dst[idx + 0] = src[y * cam_width * 3 + x * 3 + 2];
                        dst[idx + 1] = src[y * cam_width * 3 + x * 3 + 1];
                        dst[idx + 2] = src[y * cam_width * 3 + x * 3 + 0];
                    }
                }

                SDL_UnlockTexture(video_tex);
            } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to lock SDL texture");
            }
        }

    // SDL and ImGui Storing and Configs
    private:
        SDL_Window* wind;
        SDL_Renderer* rend;
        SDL_GLContext glcontext;

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

    // Publisher and Subscriber Nodes
    private:
        rclcpp::Subscription<TalonInfo>::SharedPtr track_right_sub;
        rclcpp::Subscription<TalonInfo>::SharedPtr track_left_sub;
        rclcpp::Subscription<TalonInfo>::SharedPtr trencher_sub;
        rclcpp::Subscription<TalonInfo>::SharedPtr hopper_belt_sub;
        rclcpp::Subscription<TalonInfo>::SharedPtr hopper_actuator_sub;

        rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr robot_status_pub;    

    // Motor Status Info (ImGui/ImPlot)
    private:
        InfoPlot plots;

    // Motor Data classes
    private:
        Info right_track;
        Info left_track;
        Info trencher;
        Info hopper_belt;
        Info hopper_actuator;

    private:
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub;
        SDL_Texture* video_tex = SDL_CreateTexture(rend, SDL_PIXELFORMAT_RGB24, SDL_TEXTUREACCESS_STREAMING, cam_width, cam_height);
        int cam_width;
        int cam_height;
};

#endif