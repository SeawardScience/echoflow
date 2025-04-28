#pragma once
#include "package_defs.hpp"
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/wait_for_message.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <marine_sensor_msgs/msg/radar_sector.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
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
        std::string map_frame_id_;
        float map_length_;
        float map_width_;
        float map_resolution_;
        int64_t map_publish_time_interval_;

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
     * @brief todo
     * 
     */
    void waitForTopics();

    /**
     * @brief Update map origin from robot pose and publish grid_map 
     * on same time interval as the radar takes to complete a scan. 
     * 
     * By default, timer is set to scan_time from the first RadarSector message received. 
     * This time may not exactly correspond to the timestamp that the radar begins a new scan.
     * 
     * User can also set the timer interval as a parameter.
     */
    void scanTimerCallback();

    /**
     * @brief TODO
     * 
     * @param msg 
     */
    void poseCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);

    /**
     * @brief TODO
     * 
     * @param msg 
     */
    void radarSectorCallback(const marine_sensor_msgs::msg::RadarSector::SharedPtr msg);

    rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr grid_map_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr position_subscriber_;
    rclcpp::Subscription<marine_sensor_msgs::msg::RadarSector>::SharedPtr radar_sector_subscriber_;
    rclcpp::TimerBase::SharedPtr scan_timer_;

private:
    RadarGridMap radar_map_;
    rclcpp::Clock clock_;

    // 
    bool pose_initialized_ = false;
    bool timer_interval_initialized_ = false;
    int64_t scan_time_;

};

NS_FOOT
