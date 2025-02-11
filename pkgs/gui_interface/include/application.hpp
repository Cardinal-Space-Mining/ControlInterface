#ifndef APPLICATION_HPP
#define APPLICATION_HPP

#include <chrono>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int8.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/point_types_conversion.h>

#include "custom_types/msg/talon_info.hpp"

#include <SDL2/SDL.h>
#include <cstdio>

#include "implot.h"
#include "implot_internal.h"

#include "imgui.h"
#include "imgui_impl_sdl2.h"
#include "imgui_impl_sdlrenderer2.h"

#include "custom_elements.hpp"

#include "motor_info.hpp"
#include "info_plot.hpp"

using std::shared_ptr;
using std::vector;
using namespace std::chrono_literals;
using namespace custom_types::msg;
using Info = shared_ptr<MotorInfo>;

#define WIDTH 1920
#define HEIGHT 1080
#define BUFFER_SIZE 600

class Application : public rclcpp::Node
{
public:
    Application();

    // Make it unmovable and uncopyable
    Application(Application &other) = delete;
    Application(Application &&other) = delete;
    Application &operator=(const Application &&) = delete;
    Application &operator=(const Application &) = delete;

    ~Application() override;

private:
    void init_elements();

    void update();

    void update_info(const TalonInfo &msg, const int id);

    void imageCallback(const sensor_msgs::msg::CompressedImage &msg);

    void pclCallback(const sensor_msgs::msg::PointCloud2 &msg);

    void pcl2ToPCL(const sensor_msgs::msg::PointCloud2 &msg, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);

    // SDL and ImGui Storing and Configs
private:
    SDL_Window *wind = [this]() -> SDL_Window *
    {
        SDL_Window *wind_ = SDL_CreateWindow("Mission Control GUI", SDL_WINDOWPOS_CENTERED,
                                             SDL_WINDOWPOS_CENTERED, WIDTH, HEIGHT, SDL_WINDOW_RESIZABLE);
        if (wind_ == nullptr)
        {
            RCLCPP_ERROR(this->get_logger(), "Could not create SDL Window. Error Msg: %s", SDL_GetError());
        }
        return wind_;
    }();

    SDL_Renderer *rend = [this]() -> SDL_Renderer *
    {
        SDL_Renderer *rend_ = SDL_CreateRenderer(wind, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
        if (rend_ == nullptr)
        {
            RCLCPP_ERROR(this->get_logger(), "Could not create SDL Renderer. Error msg: %s", SDL_GetError());
            return nullptr;
        }
        return rend_;
    }();

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

    // Camera Feed
private:
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr image_sub;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr camera_selection;
    int cam_width = 640;
    int cam_height = 480;
    SDL_Texture *video_tex = [this]()
    {
        SDL_Texture *video_tex_ = SDL_CreateTexture(rend, SDL_PIXELFORMAT_RGB24, SDL_TEXTUREACCESS_STREAMING, cam_width, cam_height);
        if (video_tex_ == nullptr)
        {
            RCLCPP_ERROR(this->get_logger(), "Could not create SDL_CreateTexture. Error Msg: %s", SDL_GetError());
        }
        return video_tex_;
    }();

    int current_camera;
    int last_camera;
    const std::vector<std::string> camera_options = {"Front Camera", "Rear Camera", "Left Camera", "Right Camera", "No Camera"};

    // 3D Lidar Point Cloud
private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_map_sub;
    SDL_Texture *pcl_tex = [this]()
    {
        SDL_Texture *pcl_tex_ = SDL_CreateTexture(rend, SDL_PIXELFORMAT_RGBA8888, SDL_TEXTUREACCESS_STREAMING, 800, 600);
        if (pcl_tex_ == nullptr)
        {
            RCLCPP_ERROR(this->get_logger(), "Count not create SDL_CreateTexture. Error Msg: %s", SDL_GetError());
        }
        return pcl_tex_;
    }();
};

#endif