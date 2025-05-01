#pragma once

#include "package_defs.hpp"
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/wait_for_message.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <marine_sensor_msgs/msg/radar_sector.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <deque>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

using std::placeholders::_1;
using namespace std::chrono_literals;

NS_HEAD

    /**
 * @brief Node that builds a 2D grid map from incoming marine radar sector messages.
 *
 * This node subscribes to radar sector messages, applies optional near-field clutter filtering,
 * performs coordinate transformations using TF2, and publishes both a grid map and an occupancy grid.
 */
    class RadarGridMapNode : public rclcpp::Node
{
public:
  /**
     * @brief Construct a new RadarGridMapNode object.
     */
  RadarGridMapNode();

  /**
     * @brief Struct to hold ros2 parameters.
     */
  struct Parameters
  {
    struct {
      std::string frame_id = "map";   ///< The fixed frame for the output grid map.
      float length = 400.0;         ///< Length of the grid map in meters.
      float width = 400.0;          ///< Width of the grid map in meters.
      float resolution = 10.0;        ///< map cell resolution in meters.
      float pub_interval = 0.1;       ///< map publication interval in seconds.
    } map;

    struct {
      float near_clutter_range = 0.0; ///< Maximum range in meters for near-field clutter filtering.
    } filter;

    int max_queue_size = 1000; ///< Maximum number of radar messages to buffer.

    /**
       * @brief Declares and initializes all node parameters.
       *
       * @param node Pointer to the ROS2 node for parameter declaration.
       */
    void init(rclcpp::Node * node);
  };

  std::shared_ptr<grid_map::GridMap> getMapPtr(){return map_ptr_;}

protected:
  Parameters parameters_; ///< Runtime parameters.

  /**
     * @brief Waits for topics and TFs to become available.
     */
  void waitForTopics();

  /**
     * @brief Publishes the current grid map as an occupancy grid.
     */
  void publishCostmap();

  /**
     * @brief Callback when a new radar sector message is received.
     *
     * @param msg Shared pointer to the received radar sector message.
     */
  void radarSectorCallback(const marine_sensor_msgs::msg::RadarSector::SharedPtr msg);

  /**
     * @brief Adds a radar sector message to the internal processing queue.
     *
     * @param msg Shared pointer to the radar sector message.
     */
  void addToQueue(const marine_sensor_msgs::msg::RadarSector::SharedPtr msg);

  /**
     * @brief Processes all messages in the radar sector queue.
     */
  void procesQueue();

  /**
     * @brief Processes a single radar sector message.
     *
     * @param msg Shared pointer to the radar sector message.
     */
  void processMsg(const marine_sensor_msgs::msg::RadarSector::SharedPtr msg);

  /**
     * @brief Recenters the grid map around a new position.
     *
     * @param new_center New center position in map coordinates.
     */
  void recenterMap(const grid_map::Position& new_center);



private:
  //------------------------------------------------------------------------------
  // ROS Interfaces
  //------------------------------------------------------------------------------

  // Publishers
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr grid_map_publisher_; ///< Publishes the full radar grid map.
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_publisher_; ///< Publishes a simplified occupancy grid.

  // Subscriber
  rclcpp::Subscription<marine_sensor_msgs::msg::RadarSector>::SharedPtr radar_sector_subscriber_; ///< Subscribes to incoming radar sector messages.

  // Timers
  rclcpp::TimerBase::SharedPtr costmap_timer_; ///< Timer to periodically publish the occupancy grid.
  rclcpp::TimerBase::SharedPtr queue_timer_;   ///< Timer to regularly process buffered radar sector messages.

  //------------------------------------------------------------------------------
  // Core Data
  //------------------------------------------------------------------------------

  std::shared_ptr<grid_map::GridMap> map_ptr_; ///< Main grid map data structure.
  std::deque<marine_sensor_msgs::msg::RadarSector::SharedPtr> radar_sector_queue_; ///< Queue for buffering radar sector messages.

  //------------------------------------------------------------------------------
  // TF and Timing
  //------------------------------------------------------------------------------

  std::shared_ptr<tf2_ros::Buffer> m_tf_buffer; ///< TF2 buffer for transform lookups.
  std::shared_ptr<tf2_ros::TransformListener> m_tf_listener; ///< TF2 listener attached to the buffer.

  rclcpp::Clock clock_; ///< Node-local clock used for throttling and timing.

  //------------------------------------------------------------------------------
  // Status Flags and Counters
  //------------------------------------------------------------------------------

  bool tf_ready_ = false; ///< True once at least one valid TF has been received.
  bool pose_initialized_ = false; ///< True once initial map centering has been done.
  bool timer_interval_initialized_ = false; ///< True once timers are fully initialized.

  size_t drop_counter = 0; ///< Counter for number of dropped radar messages due to queue overflow.
  int64_t scan_time_; ///< Optional timestamp of last radar scan processing.

};

NS_FOOT
