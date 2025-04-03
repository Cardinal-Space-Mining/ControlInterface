#include "lidar_map.hpp"

#include <chrono>
#include <bit>
#include <cmath>
#include <iomanip>

LidarMap::LidarMap(rclcpp::Node& parent) : parent(parent) {}

LidarMap::LidarMap(SDL_Renderer* rend, rclcpp::Node& parent)
                    : rend(rend)
                      , parent(parent)
                      // , pcl_sub(parent.create_subscription<sensor_msgs::msg::PointCloud2>(
                                // "multiscan/lidar_scan", rclcpp::SensorDataQoS{}, [this](const sensor_msgs::msg::PointCloud2& msg) {
                            // pclCallback(msg);
                        // }))
                      , pcl_tex(SDL_CreateTexture(rend,
                                                  SDL_PIXELFORMAT_RGBA4444,
                                                  SDL_TEXTUREACCESS_STREAMING,
                                                  map_wd, map_ht))
                      , frame_buffer(map_wd * map_ht, 0x00000000)
{
    pcl_sub = parent.create_subscription<sensor_msgs::msg::PointCloud2>(
            "multiscan/lidar_scan", rclcpp::SensorDataQoS{},
            [this](const sensor_msgs::msg::PointCloud2& msg) {
                this->pclCallback(msg);
                // RCLCPP_INFO(this->parent.get_logger(), "post callback?");
            });

    RCLCPP_INFO(parent.get_logger(), "Lidar Map initialized");
}

LidarMap::~LidarMap() {
    RCLCPP_INFO(parent.get_logger(), "Lidar map closed");
    if (pcl_tex != nullptr) {
        SDL_DestroyTexture(pcl_tex);
    }
}

SDL_Texture* LidarMap::getPCL() const {
    return pcl_tex;
}

int LidarMap::getWd() const {
    return map_wd;
}

int LidarMap::getHt() const {
    return map_ht;
}

void LidarMap::updateTexture() {

    if (pcl_tex == nullptr) {
        pcl_tex = SDL_CreateTexture(
            rend,
            SDL_PIXELFORMAT_RGBA4444,
            SDL_TEXTUREACCESS_STREAMING,
            map_wd, map_ht);
        
        if (pcl_tex == nullptr) {
            RCLCPP_ERROR(this->parent.get_logger(), "Failed to create SDL Texture for PCL: %s", SDL_GetError());
            return;
        } else {
            RCLCPP_INFO(this->parent.get_logger(), "SDL_CreateTexture successful");
        }
    }

    void* pixels = nullptr;
    int pitch = 0;
    if (SDL_LockTexture(pcl_tex, nullptr, &pixels, &pitch) != 0) {
        RCLCPP_ERROR(this->parent.get_logger(), "SDL_LockTexture failed: %s", SDL_GetError());
        return;
    } else {
        // RCLCPP_INFO(this->parent.get_logger(), "SDL_LockTexture successful");
    }

    memcpy(pixels, frame_buffer.data(), frame_buffer.size() * sizeof(uint32_t));
    SDL_UnlockTexture(pcl_tex);
    // RCLCPP_INFO(this->parent.get_logger(), "SDL_UnlockTexture successful");
}

void LidarMap::pclCallback(const sensor_msgs::msg::PointCloud2& msg) {

    static auto last_update = std::chrono::steady_clock::now();
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_update).count();

    if (elapsed < 200) {
        return;
    }

    auto time_since_epoch = now.time_since_epoch();
    auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(time_since_epoch).count();
    double seconds = milliseconds / 1000.0;

    RCLCPP_INFO(this->parent.get_logger(), "pclCallback called at %f seconds", seconds);

    std::fill(frame_buffer.begin(), frame_buffer.end(), 0x00000000);

    // pointer to data in pcl2 message
    const uint8_t* ptr = msg.data.data();
    size_t point_step = msg.point_step;

    // iterate over all points in pcl2 msg
    for (size_t i = 0; i < msg.width * msg.height; ++i) {
        // read x, y, z coords from pcl2 msg
        float x = *reinterpret_cast<const float*>(ptr + msg.fields[0].offset);
        float y = *reinterpret_cast<const float*>(ptr + msg.fields[1].offset);
        float z = *reinterpret_cast<const float*>(ptr + msg.fields[2].offset);

        // move ptr to next step in pcl2 msg
        ptr += point_step;

        // skip points if invalid
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
            continue;
        }

        // perspective projection to 2d screen
        float fov = 90.0f;
        float aspect = static_cast<float>(map_wd) / map_ht;
        float focal_len = 1.0 / tan((fov * 0.5f) * M_PI / 180.0f);

        // transform 3d to 2d space
        float screen_x = (x / -z) * focal_len * map_wd / aspect + map_wd / 2.0;
        float screen_y = (y / -z) * focal_len * map_ht / aspect + map_ht / 2.0;

        // map 2d projected point to frame buffer coords
        int pixel_x = static_cast<int>(screen_x);
        int pixel_y = static_cast<int>(screen_y);

        if (pixel_x >= 0 && pixel_x < map_wd && pixel_y >= 0 && pixel_y < map_ht) {
            // // set pixel color to yellow
            // frame_buffer[pixel_y * map_wd + pixel_x] = 0x000000FF; // RGBA
            int point_radius = 20;
            for (int dx = -point_radius; dx <= point_radius; ++dx) {
                for (int dy = -point_radius; dy <= point_radius; ++dy) {
                    if (dx * dx + dy * dy <= point_radius * point_radius) {
                        int draw_x = pixel_x + dx;
                        int draw_y = pixel_y + dy;
                        if (draw_x >= 0 && draw_x < map_wd && draw_y >= 0 && draw_y < map_ht) {
                            frame_buffer[draw_y * map_wd + draw_x] = 0xFFFFFFFF;
                        }
                    }
                }
            }
        }
    }

    updateTexture();
    last_update = now;
}