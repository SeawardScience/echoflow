/** Copyright © 2025 Seaward Science. */

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
#include "statistics.hpp"

NS_HEAD

/**
 * @brief Node that uses a particle filter to track targets in a 2D grid map of marine radar data.
 *
 * This node shares a pointer to a grid map with the radar_grid_map_node and spawns particles on areas
 * of the map with valid radar returns in order to track the position and course of moving radar targets.
 *
 * The node publishes a pointcloud of particles and a grid map with aggregated statistics on the particles
 * (number of particles per cell, mean and standard deviation of particle age, x-position, y-position,
 * course and velocity).
 */
class ParticleFilterNode : public rclcpp::Node
{
public:
  /**
   * @brief Construct a new Particle Filter Node object.
   */
  ParticleFilterNode();

  /**
   * @brief All configurable parameters for the particle filter node.
   */
  struct Parameters
  {
  /**
   * @brief Parameters controlling the behavior of the particle filter.
   */
    struct {
      /**
       * @brief Total number of particles used in the filter.
       */
      int num_particles = 100000;

      /**
       * @brief Time (in seconds) between particle weight updates.
       */
      double update_interval = 0.2;

      /**
       * @brief Maximum speed (m/s) assigned to particles during initialization.
       */
      double initial_max_speed = 20.0;

      /**
       * @brief Standard deviation (m) of the observation likelihood model.
       */
      double observation_sigma = 50.0;

      /**
       * @brief Half-life (in seconds) for exponential decay of particle weights.
       *
       * Lower values cause weights to fade more quickly over time.
       * This should generally be on the order of the radar sweep time.
       */
      double weight_decay_half_life = 3.0;

      /**
       * @brief Fraction of particles (per second) that are reseeded with random poses on each resample step.
       *
       * Increase this value to more quickly lock on to newly detected targets.
       */
      double seed_fraction = 0.001;

      /**
       * @brief Standard deviation (m) of positional noise added during resampling.
       */
      double noise_std_position = 0.1;

      /**
       * @brief Standard deviation (radians) of yaw angle noise added during resampling.
       */
      double noise_std_yaw = 0.05;

      /**
       * @brief Standard deviation (radians/sec) of yaw rate noise added during resampling.
       */
      double noise_std_yaw_rate = 0.0;

      /**
       * @brief Standard deviation (m/s) of speed noise added during resampling.
       */
      double noise_std_speed = 0.2;

      /**
       * @brief Maximum physical size (in meters) for a trackable target blob.
       *
       * Used to reduce computational load on large targets like shorelines.
       */
      double maximum_target_size = 200.0;

      /**
   * @brief Density (particles/m²) at which the weight of a particle will be reduced by half.
   *
   * Lower this value if particles cluster too aggressively on single targets.
   */
      double density_feedback_factor = 0.8;
    } particle_filter;


    /**
     * @brief Parameters defining the statistical grid map used for monitoring particle behavior.
     */
    struct {
      std::string frame_id = "map";   //!< Coordinate frame in which the particle statistics map is published.
      double length = 2500.0;         //!< Length (in meters) of the grid map.
      double width = 2500.0;          //!< Width (in meters) of the grid map.
      double resolution = 25.0;       //!< Resolution of each grid cell (in meters).
      double pub_interval = 0.5;      //!< Time interval (in seconds) between publishing the statistics map.
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

  std::shared_ptr<grid_map::GridMap> map_ptr_;  //!< Shared pointer to the underlying grid map of radar intensity & EDT layers.

protected:
  Parameters parameters_;     //!< Runtime parameters.

private:
  /**
   * @brief Applies parameters to the particle filter.
   *
   * Applies the current parameters to the particle filter, including noise distributions and
   * other settings. This is called when parameters are updated and on intialization.
   */
  void applyParameters();

  /**
   * @brief Main particle filter update function.
   *
   * Updates the particle filter by predicting the next particle position and course,
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
   *    * Average age of particles (mean and standard deviation)
   *    * Average x-position of particles (mean and standard deviation)
   *    * Average y-position of particles (mean and standard deviation)
   *    * Average speed of particles (mean and standard deviation)
   *    * Average course of particles (circular mean and circular standard deviation)
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

  /**
   * @brief Store particle positions and course angle in a pose array and publish. Function can be
   * visualized in rviz2 as a PoseArray showing particle course angles as a vector.
   *
   * Publishes: geometry_msgs::msg::PoseArray topic of all particles and courses.
   */
  void publishParticleVectorField();

  /**
   * @brief Store mean cell x/y position and course angle in a pose array and publish. Function can be
   * visualized in rviz2 as a PoseArray showing mean cell course angles as a vector.
   *
   * Publishes: geometry_msgs::msg::PoseArray topic of mean cell x/y positions and mean course.
   */
  void publishCellVectorField();

  /**
   * @brief Helper function to convert 2D Euler angle into "yaw" quaternion,
   * i.e. quaternion representing rotation around Z-axis.
   *
   * @param angle angle to convert to quaternion.
   * @return geometry_msgs::msg::Quaternion quaternion representation of given angle.
   */
  geometry_msgs::msg::Quaternion angleToYawQuaternion(float angle);

  std::unique_ptr<MultiTargetParticleFilter> pf_; //!< Pointer to the multi-target particle filter instance.
  std::shared_ptr<grid_map::GridMap> pf_statistics_; //!< Pointer to the grid map containing particle filter statistics.

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_; //!< Publisher for the particle point cloud.
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr cell_vector_field_pub_; //!< Publisher for the mean cell vector field.
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr particle_vector_field_pub_; //!< Publisher for the particle vector field.
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr pf_statistics_pub_; //!< Publisher for the particle filter statistics grid map.

  rclcpp::TimerBase::SharedPtr timer_; //!< Timer for the particle filter update function.
  rclcpp::TimerBase::SharedPtr pf_statistics_timer_; //!< Timer for computing and publishing particle filter statistics.

  rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr parameters_on_set_callback_; //!< Callback triggered *before* parameters are applied to the node.

  std::shared_ptr<rclcpp::ParameterEventHandler> parameter_event_handler_; //!< Parameter event handler for listening to parameter changes.

  rclcpp::ParameterEventCallbackHandle::SharedPtr parameter_event_callback_handle_; //!< Callback triggered *after* parameter changes have been successfully applied to the node.

  bool initialized_ = false; //!< Flag indicating whether the particle filter has been initialized.
  rclcpp::Time last_update_time_; //!< Timestamp of the last particle filter update.
};

NS_FOOT
