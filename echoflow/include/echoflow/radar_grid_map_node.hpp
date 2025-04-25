#pragma once
#include "package_defs.hpp"
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <rclcpp/rclcpp.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <marine_sensor_msgs/msg/radar_sector.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include "radar_grid_map.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

NS_HEAD 


/**
 * @brief TODO
 * 
 */
class RadarGridMapNode : public rclcpp::Node 
{
public:
    /**
     * @brief Construct a new Radar Grid Map Node object
     * 
     * TODO
     */
    RadarGridMapNode();

    struct Parameters
    {
        float map_length_;
        float map_width_;
        float map_resolution_;
        float map_initial_position_x_;
        float map_initial_position_y_;

        /**
         * @brief Declares all parameters and initializes stored variables within Parameters struct.
         * 
         * @param node Pointer to node to use to initialize the parameters.
         */
        void init(rclcpp::Node * node);
    };

protected:
    Parameters parameters_;

    /**
     * @brief TODO
     * 
     * @param msg 
     */
    void radarSectorCallback(const marine_sensor_msgs::msg::RadarSector::SharedPtr msg);

    /**
     * @brief todo
     * 
     * @param msg 
     */
    void radarPointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr grid_map_publisher_;
    rclcpp::Subscription<marine_sensor_msgs::msg::RadarSector>::SharedPtr radar_sector_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr radar_pointcloud_subscriber_;

private:
    RadarGridMap radar_map_;

};

NS_FOOT
