#pragma once

#include <geometry_msgs/msg/point.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <radar_grid_map_node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include "grid_map_filters.hpp"
#include "particle_filter.hpp"

NS_HEAD

/**
 * @brief Node that uses a particle filter to track targets in a 2D grid map of marine radar data.
 *
 * This node shares a pointer to a grid map with the radar_grid_map_node and spawns particles on areas
 * of the map with valid radar returns in order to track the position and heading of moving radar targets.
 *
 * The node publishes a pointcloud of particles and a grid map with aggregated statistics on the particles
 * (number of particles per cell, average and standard deviation of x-position, y-position, heading and velocity).
 */
class ParticleFilterNode : public rclcpp::Node
{
public:
  /**
   * @brief Construct a new Particle Filter Node object.
   *
   */
  ParticleFilterNode();

  std::shared_ptr<grid_map::GridMap> map_ptr_;

private:
  /**
   * @brief todo
   *
   */
  void update();

  /**
   * @brief Convert particle filters to a pointcloud and publish.
   *
   * Publishes a sensor_msgs::msg::PointCloud2 topic.
   */
  void publishPointCloud();

  std::unique_ptr<MultiTargetParticleFilter> pf_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::vector<Detection> pending_detections_;

  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr pf_statistics_pub_;
  grid_map::GridMap *pf_statistics_;

  bool initialized_ = false;
  rclcpp::Time last_update_time_;
};

NS_FOOT
