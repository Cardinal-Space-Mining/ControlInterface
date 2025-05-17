#ifndef LIDAR_MAP_HPP
#define LIDAR_MAP_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <glad/glad.h>
#include <SDL2/SDL.h>
#include <vector>

class LidarMap
{
public:
    LidarMap(rclcpp::Node &parent);
    ~LidarMap();

    void init();

    GLuint getTexture();
    int getWd() const { return map_wd; };
    int getHt() const { return map_ht; };

    void handleKeyState(const Uint8 *keys);

private:
    void pclCallback(const sensor_msgs::msg::PointCloud2 &msg);
    void renderToFramebuffer();

    rclcpp::Node &parent;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_sub;

    GLuint fbo = 0;
    GLuint gl_tex = 0;
    GLuint gl_rbo = 0;
    GLuint shader_program = 0;
    int map_wd = 800;
    int map_ht = 600;

    const float grid_step = 1.0f;
    const float grid_min = -10.f * grid_step;
    const float grid_max = 10.f * grid_step;

    // Point cloud data
    std::vector<float> points_xyz;
    bool points_dirty = false;
    GLuint points_vao = 0;
    GLuint points_vbo = 0;

    // Grid lines
    void createGridLines();
    std::vector<float> grid_lines;
    GLuint grid_vao = 0;
    GLuint grid_vbo = 0;

    // Camera parameters
    float yaw = 0.0f;
    float pitch = 0.0f;
    float radius = 20.0f; // Distance from origin

    // Mouse input tracking
    int last_mouse_x = 0;
    int last_mouse_y = 0;
    bool rotating = false;
};
#endif
