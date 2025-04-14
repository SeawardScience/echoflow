#include "minimal_publisher_node.hpp"

using namespace std::chrono_literals;

NS_HEAD

MinimalPublisherNode::MinimalPublisherNode()
    : Node("minimal_publisher"), count_(0)
{
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);

    message_="Hello, world! ";
    this->declare_parameter("message", message_);
    this->get_parameter("message", message_);


    timer_ = this->create_wall_timer(
        500ms, std::bind(&MinimalPublisherNode::timer_callback, this));
}

void MinimalPublisherNode::timer_callback()
{
    auto message = std_msgs::msg::String();
    message.data = message_ + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
}

NS_FOOT
