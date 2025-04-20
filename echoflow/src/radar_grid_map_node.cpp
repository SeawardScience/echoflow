#include "radar_grid_map_node.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

NS_HEAD

RadarGridMapNode::RadarGridMapNode()
    : Node("radar_grid_map")
{
    grid_map_publisher_ = this->create_publisher<grid_map_msgs::msg::GridMap>(
                          "radar_grid_map", rclcpp::QoS(1).transient_local());


    // subscribe to radar data sector 
    radar_sector_subscriber_ = this->create_subscription<marine_sensor_msgs::msg::RadarSector>(
                               "data", 50, std::bind(&RadarGridMapNode::radarSectorCallback, this, _1));                         

    // subscribe to radar point cloud
    radar_pointcloud_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                                   "pointcloud", 50, std::bind(&RadarGridMapNode::radarPointCloudCallback, this, _1));


}

void RadarGridMapNode::radarSectorCallback(const marine_sensor_msgs::msg::RadarSector::SharedPtr msg)
{
    std::cerr << "received radar sector message!" << std::endl;
}

void RadarGridMapNode::radarPointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    std::cerr << "received radar point cloud message!" << std::endl;
}

NS_FOOT