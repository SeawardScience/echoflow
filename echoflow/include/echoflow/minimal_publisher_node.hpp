#pragma once  // Favor using this over the #ifndef, #define method


// First include your local package stuff
#include "package_defs.hpp"  //  This is where we include all our namespace stuff for the package

// then include external libary stuff
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>



NS_HEAD  // macro for consistantly defining our namespace for the package


/**
 * @brief a minimal publisher class that extends the rclcpp::Node class to
 * demonstrate how to create ROS2 Node
 *
 * This class is used to create a ROS2 publisher that regularly publishes
 * messages to a topic.
 */
class MinimalPublisherNode : public rclcpp::Node
{
public:
    /**
     * @brief Construct a new Minimal Publisher object
     *
     * The constructor initializes the publisher and timer.
     */
    MinimalPublisherNode();

protected:
    /**
     * @brief Callback function for the timer
     *
     * This function is called periodically by the timer. It publishes
     * a message containing a string with a counter.
     */
    void timer_callback();

    std::string message_;

    rclcpp::TimerBase::SharedPtr timer_; ///< Shared pointer to the timer
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_; ///< Shared pointer to the publisher
    size_t count_; ///< Counter for the number of messages published
};

NS_FOOT
