#pragma once

#include <cmath>
#include <memory>
#include <string>
#include <vector>
#include <geometry_msgs/msg/point.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include "grid_map_filters.hpp"
#include "particle_filter.hpp"

NS_HEAD

class ParticleFilterNode : public rclcpp::Node {
public:
  ParticleFilterNode();
  std::shared_ptr<grid_map::GridMap> map_ptr_;
private:
  void update();
  void publishPointCloud();

  std::unique_ptr<MultiTargetParticleFilter> pf_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr detections_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::vector<Detection> pending_detections_;
  bool initialized_ = false;
  rclcpp::Time last_update_time_;
};

NS_FOOT
