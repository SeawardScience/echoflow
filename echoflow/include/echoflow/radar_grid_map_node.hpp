#pragma once
#include "package_defs.hpp"
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <marine_sensor_msgs/msg/radar_sector.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

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

protected:

    void radarSectorCallback(const marine_sensor_msgs::msg::RadarSector::SharedPtr msg);
    void radarPointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr grid_map_publisher_;
    rclcpp::Subscription<marine_sensor_msgs::msg::RadarSector>::SharedPtr radar_sector_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr radar_pointcloud_subscriber_;

};

NS_FOOT
