#ifndef LIDAR_MAP_HPP
#define LIDAR_MAP_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <SDL2/SDL.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/point_types_conversion.h>

#include <cstdio>
#include <vector>

class LidarMap {
    public:
        LidarMap(rclcpp::Node& parent);
        LidarMap(SDL_Renderer* rend, rclcpp::Node& parent);

        SDL_Texture* getPCL() const;
        int getWd() const;
        int getHt() const;

        ~LidarMap();

    private:
        void pclCallback(const sensor_msgs::msg::PointCloud2 &msg);
        void updateTexture();

    private:
        SDL_Renderer* rend;
        rclcpp::Node& parent;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_sub;
        int map_wd = 800;
        int map_ht = 600;
        SDL_Texture* pcl_tex;

        std::vector<uint32_t> frame_buffer;
};
#endif
