#include "radar_grid_map_node.hpp"

NS_HEAD

template<typename T>
void setupParam(T * variable, rclcpp::Node *node, std::string topic, T initial_val){
  node->declare_parameter(topic, initial_val);
  *variable =
      node->get_parameter(topic).get_parameter_value().get<T>();
}

// a explicit overload for string is required for casting to work correctly
void setupParam(std::string * variable, rclcpp::Node *node , std::string topic, std::string initial_val){
  setupParam<std::string>(variable, node, topic, initial_val);
}

void RadarGridMapNode::Parameters::init(rclcpp::Node *node)
{
  setupParam(&map.frame_id, node, "map.frameId", map.frame_id);
  setupParam(&map.length, node, "map.length", map.length);
  setupParam(&map.width, node, "map.width", map.width);
  setupParam(&map.resolution, node, "map.resolution", map.resolution);
  setupParam(&map.pub_interval, node, "map.pub_interval", map.pub_interval); // TODO default should be scan_time
  setupParam(&max_queue_size, node, "max_queue_size", max_queue_size);
}

RadarGridMapNode::RadarGridMapNode()
    : Node("radar_grid_map")
{
    parameters_.init(this);

    grid_map_publisher_ = this->create_publisher<grid_map_msgs::msg::GridMap>(
                          "radar_grid_map", rclcpp::QoS(1).transient_local());

    radar_sector_subscriber_ = this->create_subscription<marine_sensor_msgs::msg::RadarSector>(
                               "data", 50, std::bind(&RadarGridMapNode::radarSectorCallback, this, _1));

    // TF listener
    m_tf_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    m_tf_listener = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer);


    map_ptr_.reset(new grid_map::GridMap({"elevation"}));
    map_ptr_->setFrameId(parameters_.map.frame_id);
    map_ptr_->setGeometry(grid_map::Length(parameters_.map.length, parameters_.map.width), parameters_.map.resolution);
    RCLCPP_INFO(
        this->get_logger(),
        "Created map with size %f x %f m (%i x %i cells).",
        map_ptr_->getLength().x(), map_ptr_->getLength().y(),
        map_ptr_->getSize()(0), map_ptr_->getSize()(1));


    // scan_timer_ = this->create_wall_timer(std::chrono::seconds(scan_time_),
    //                 std::bind(&RadarGridMapNode::scanTimerCallback, this));
                      

}

void RadarGridMapNode::waitForTopics() {

}

void RadarGridMapNode::scanTimerCallback()
{

}

void RadarGridMapNode::radarSectorCallback(const marine_sensor_msgs::msg::RadarSector::SharedPtr msg)
{
  addToQueue(msg);
  procesQueue();
}

void RadarGridMapNode::addToQueue(const marine_sensor_msgs::msg::RadarSector::SharedPtr msg)
{
  size_t MAX_QUEUE_SIZE = parameters_.max_queue_size; // Max allowed in queue

  if (radar_sector_queue_.size() >= MAX_QUEUE_SIZE)
  {
    radar_sector_queue_.pop_front();
    drop_counter++;
  }

  radar_sector_queue_.push_back(msg);

  // Only occasionally warn if we've dropped
  if (drop_counter > 0)
  {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "Radar sector queue full (>%zu). Dropped %zu message(s).",
                         MAX_QUEUE_SIZE, drop_counter);

    drop_counter = 0; // Reset counter after warning
  }


}

void RadarGridMapNode::procesQueue()
{
  while (!radar_sector_queue_.empty())
  {
    auto msg = radar_sector_queue_.front(); // Peek at the first message

    // Check if transform is available without throwing exceptions
    if (!m_tf_buffer->canTransform(
            parameters_.map.frame_id,
            msg->header.frame_id,
            rclcpp::Time(msg->header.stamp),
            tf2::durationFromSec(0.01)))  // small timeout (10ms)
    {
      // ❌ Transform not ready yet, stop processing
      RCLCPP_DEBUG(this->get_logger(),
                   "Transform not yet available for radar sector at time %u.%u. Stopping queue processing.",
                   msg->header.stamp.sec,
                   msg->header.stamp.nanosec);
      break;
    }

    // ✅ Transform is available, so get it
    auto transform = m_tf_buffer->lookupTransform(
        parameters_.map.frame_id,
        msg->header.frame_id,
        rclcpp::Time(msg->header.stamp));

    RCLCPP_DEBUG(this->get_logger(),
                 "Processing radar sector at time %u.%u. Queue size: %zu",
                 msg->header.stamp.sec,
                 msg->header.stamp.nanosec,
                 radar_sector_queue_.size());

    processMsg(msg);               // Process the message
    radar_sector_queue_.pop_front(); // Remove from queue
  }
}




void RadarGridMapNode::processMsg(const marine_sensor_msgs::msg::RadarSector::SharedPtr msg)
{
  geometry_msgs::msg::TransformStamped transform;

  try
  {
    transform = m_tf_buffer->lookupTransform(
        parameters_.map.frame_id,
        msg->header.frame_id,
        rclcpp::Time(msg->header.stamp)
        );
  }
  catch (const tf2::TransformException &ex)
  {
    RCLCPP_WARN(this->get_logger(), "Could not transform sector for processing: %s", ex.what());
    return;
  }

  RCLCPP_INFO(this->get_logger(),
              "Processing radar sector at time %u.%u. Queue size: %zu",
              msg->header.stamp.sec,
              msg->header.stamp.nanosec,
              radar_sector_queue_.size());


}


NS_FOOT
