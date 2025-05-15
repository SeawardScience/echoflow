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
//#include "heading_utilities.hpp"
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
   */
  ParticleFilterNode();

  /**
   * @brief Struct to hold ROS2 parameters.
   */
  struct Parameters
  {
    struct {
      int num_particles = 100000;
      float update_interval = 0.1;            // seconds
    } particle_filter;

    struct {
      std::string frameId = "map";
      float length = 1500.0;
      float width = 1500.0;
      float resolution = 10.0;
      float pub_interval = 1.0;               // seconds
    } particle_filter_statistics;

    /**
     * @brief Declares all node parameters.
     *
     * @param node Pointer to the ROS2 node for parameter declaration.
     */
    void declare(rclcpp::Node * node);

    /**
     * @brief Updates all node parameters.
     *
     * @param node Pointer to the ROS2 node for parameter update.
     */
    void update(rclcpp::Node * node);
  };

  std::shared_ptr<grid_map::GridMap> map_ptr_;

protected:
  Parameters parameters_;     // Runtime parameters.

private:
  /**
   * @brief Main particle filter update function.
   *
   * Updates the particle filter by predicting the next particle position and heading,
   * udpating particle weights, and resampling particles. Also computes aggregated statistics
   * on the particles in the point cloud.
   *
   * Publishes: point cloud of all live particles.
   */
  void update();

  /**
   * @brief Computes and publishes statistics on the particles in the particle filter.
   *
   * Computes the following statistics on the particles in each cell over a user-specified
   * window of the grid map:
   *    * Number of particles
   *    * Average x-position of particles (mean and standard deviation)
   *    * Average y-position of particles (mean and standard deviation)
   *    * Average heading of particles (circular mean and circular standard deviation)
   *    * Average velocity of particles (mean and standard deviation)
   *
   * Publishes: grid_map_msgs::msg::GridMap topic containing particle filter statistics as
   * layers in a grid map.
   */
   void computeParticleFilterStatistics();

  /**
   * @brief Convert particles to a pointcloud and publish.
   *
   * Publishes: sensor_msgs::msg::PointCloud2 topic of all currently live particles.
   */
  void publishPointCloud();

  std::unique_ptr<MultiTargetParticleFilter> pf_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::vector<Detection> pending_detections_;

  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr pf_statistics_pub_;
  rclcpp::TimerBase::SharedPtr pf_statistics_timer_;
  grid_map::GridMap *pf_statistics_;

  bool initialized_ = false;
  rclcpp::Time last_update_time_;
};

NS_FOOT
