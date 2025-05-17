#ifndef APPLICATION_HPP
#define APPLICATION_HPP

#include <chrono>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/int32.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include "talon_msgs/msg/talon_info.hpp"

#include <SDL2/SDL.h>
#include <glad/glad.h>

#include "implot.h"
#include "imgui.h"
#include "imgui_impl_sdl2.h"
#include "imgui_impl_opengl3.h"

#include "custom_elements.hpp"
#include "motor_info.hpp"
#include "info_plot.hpp"
#include "hopper_capacity.hpp"
#include "lidar_map.hpp"

#ifndef GUI_PUBLISH_HEARTBEAT
#define GUI_PUBLISH_HEARTBEAT 1
#endif

using std::shared_ptr;
using std::vector;
using namespace std::chrono_literals;
using namespace talon_msgs::msg;
using Info = shared_ptr<MotorInfo>;

class Application : public rclcpp::Node
{
private:
    constexpr static int WIDTH{1920};
    constexpr static int HEIGHT{1080};
    constexpr static size_t BUFFER_SIZE{600};

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

    void update_hopper_cap(const std_msgs::msg::Float32 &msg);
    void imageCallback(const sensor_msgs::msg::CompressedImage &msg);

    SDL_Window *wind = nullptr;
    SDL_GLContext gl_context = nullptr;

    // Timer base
private:
    rclcpp::TimerBase::SharedPtr update_timer;
    rclcpp::TimerBase::SharedPtr status_update_timer;

    // Robot Status Toggle (ImGui)
private:
    int robot_status = 1;
    const std::vector<std::string> status_options = {"autonomous", "disabled", "teleop"};
    const std::vector<BASE_COLORS> toggle_cols = {BASE_COLORS::BLUE, BASE_COLORS::RED, BASE_COLORS::GREEN};
    MultiToggle status_toggle;

    // Publisher and Subscriber Nodes
private:
    rclcpp::Subscription<TalonInfo>::SharedPtr track_right_sub;
    rclcpp::Subscription<TalonInfo>::SharedPtr track_left_sub;
    rclcpp::Subscription<TalonInfo>::SharedPtr trencher_sub;
    rclcpp::Subscription<TalonInfo>::SharedPtr hopper_belt_sub;
    rclcpp::Subscription<TalonInfo>::SharedPtr hopper_actuator_sub;

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr hopper_capacity_sub;

    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr robot_status_pub;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr motor_config_pub;
    #if GUI_PUBLISH_HEARTBEAT
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr heartbeat;
    #endif

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

    // Hopper fullness detection
private:
    hopper_capacity cap;

    // Camera Feed
private:
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr image_sub;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr camera_selection;
    int cam_width = 640;
    int cam_height = 480;
    GLuint video_tex = 0;

    int current_camera;
    int last_camera;
    const std::vector<std::string> camera_options = {"Front Camera", "Rear Camera", "Left Camera", "Right Camera", "No Camera"};

    // Lidar Map
private:
    LidarMap map;
    bool lidar_map_hovered = false;
};

#endif
