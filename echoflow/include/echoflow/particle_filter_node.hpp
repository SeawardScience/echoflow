#pragma once

#include <cmath>
#include <memory>
#include <string>
#include <vector>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
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
   */
  ParticleFilterNode();

  /**
   * @brief Struct to hold ROS2 parameters.
   */
  struct Parameters
  {
    struct {
      int num_particles = 100000;
      double update_interval = 0.1;            // seconds
      double initial_max_speed = 20.0;
      double observation_sigma = 100.0;
      double decay_factor = 0.95;
      double min_resample_speed = 3.0;
      double noise_std_pos = 0.1;
      double noise_std_yaw = 0.4;
      double noise_std_yaw_rate = 1.0;
      double noise_std_speed = 4.0;
    } particle_filter;

    struct {
      std::string frameId = "map";
      double length = 1500.0;
      double width = 1500.0;
      double resolution = 50.0;
      double pub_interval = 1.0;               // seconds
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
   * @brief Applies parameters to the particle filter.
   *
   * Applies the current parameters to the particle filter, including noise distributions and
   * other settings. This is called when parameters are updated and on intialization.
   */
  void applyParameters();

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

  rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr parameter_event_sub_; //!< Handle for parameter event subscription
  /**
   * @brief Store particle positions and headings in a pose array and publish. Function can be
   * visualized in rviz2 as a PoseArray showing particle headings.
   *
   * Note: The PoseArray message displays a unit vector with the given position and heading.
   * It does not scale the vector according to the particle speed, so this has been
   * termed "particle heading field" instead of "particle velocity field".
   *
   * Publishes: geometry_msgs::msg::PoseArray topic of all particles and headings.
   */
  void publishParticleHeadingField();

  /**
   * @brief todo
   *
   */
  void publishCellHeadingField();

  /**
   * @brief Helper function to convert heading into "2D" quaternion, i.e. quaternion representing
   * rotation around Z-axis.
   *
   * @param heading Heading to convert to quaternion.
   * @return geometry_msgs::msg::Quaternion quaternion representation of given heading.
   */
  geometry_msgs::msg::Quaternion headingToQuaternion(float heading);

  std::unique_ptr<MultiTargetParticleFilter> pf_;
  grid_map::GridMap *pf_statistics_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr cell_heading_field_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr particle_heading_field_pub_;
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr pf_statistics_pub_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr pf_statistics_timer_;

  std::vector<Detection> pending_detections_;

  bool initialized_ = false;
  rclcpp::Time last_update_time_;
};

NS_FOOT
