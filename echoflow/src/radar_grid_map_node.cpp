#include "radar_grid_map_node.hpp"

#include <radar_grid_map_node.hpp>

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
  setupParam(&filter.near_clutter_range, node, "filter.near_clutter_range,", filter.near_clutter_range);
}

RadarGridMapNode::RadarGridMapNode()
    : Node("radar_grid_map")
{
    parameters_.init(this);

    grid_map_publisher_ = this->create_publisher<grid_map_msgs::msg::GridMap>(
                          "radar_grid_map", rclcpp::QoS(1).transient_local());

    costmap_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("costmap", 10);

    radar_sector_subscriber_ = this->create_subscription<marine_sensor_msgs::msg::RadarSector>(
                               "data", 50, std::bind(&RadarGridMapNode::radarSectorCallback, this, _1));

    // TF listener
    m_tf_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    m_tf_listener = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer);


    map_ptr_.reset(new grid_map::GridMap({"intensity"}));
    map_ptr_->setFrameId(parameters_.map.frame_id);
    map_ptr_->setGeometry(grid_map::Length(parameters_.map.length, parameters_.map.width), parameters_.map.resolution);
    RCLCPP_INFO(
        this->get_logger(),
        "Created map with size %f x %f m (%i x %i cells).",
        map_ptr_->getLength().x(), map_ptr_->getLength().y(),
        map_ptr_->getSize()(0), map_ptr_->getSize()(1));


    costmap_timer_ = this->create_wall_timer(std::chrono::milliseconds(int(parameters_.map.pub_interval*1000)),
                           std::bind(&RadarGridMapNode::publishCostmap, this));

    queue_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&RadarGridMapNode::procesQueue, this));

                      

}

void RadarGridMapNode::waitForTopics() {

}

void RadarGridMapNode::publishCostmap()
{
  if (!map_ptr_)
    return;

  nav_msgs::msg::OccupancyGrid occupancyGrid;
  grid_map::GridMapRosConverter::toOccupancyGrid(*map_ptr_, "intensity", 0.0, 1.0, occupancyGrid);
  costmap_publisher_->publish(occupancyGrid);

}


void RadarGridMapNode::radarSectorCallback(const marine_sensor_msgs::msg::RadarSector::SharedPtr msg)
{
  addToQueue(msg);
}


void RadarGridMapNode::addToQueue(const marine_sensor_msgs::msg::RadarSector::SharedPtr msg)
{
  size_t MAX_QUEUE_SIZE = parameters_.max_queue_size;

  if (!tf_ready_)
  {
    if (m_tf_buffer->canTransform(
            parameters_.map.frame_id,
            msg->header.frame_id,
            rclcpp::Time(msg->header.stamp),
            tf2::durationFromSec(0.01)))
    {
      tf_ready_ = true; // ✅ TFs are now ready
      RCLCPP_INFO(this->get_logger(), "TFs are now available. Radar messages will be buffered normally.");
    }
    else
    {
      // ❌ Still no TF — drop this message
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                           "Dropping radar sector at time %u.%u: TF not yet available from %s to %s.",
                           msg->header.stamp.sec, msg->header.stamp.nanosec,
                           msg->header.frame_id.c_str(), parameters_.map.frame_id.c_str());
      return;
    }
  }

  // ✅ If we get here, TFs are ready!

  if (radar_sector_queue_.size() >= MAX_QUEUE_SIZE)
  {
    radar_sector_queue_.pop_front();
    drop_counter++;
  }

  radar_sector_queue_.push_back(msg);

  if (drop_counter > 0)
  {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "Radar sector queue full (>%zu). Dropped %zu message(s).",
                         MAX_QUEUE_SIZE, drop_counter);

    drop_counter = 0;
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


void RadarGridMapNode::recenterMap(const grid_map::Position& new_center)
{
  grid_map::Position old_center = map_ptr_->getPosition();
  double distance = (new_center - old_center).norm();
  double move_threshold = 10.0 * parameters_.map.resolution; // 10x cell size

  if (distance > move_threshold)
  {
    map_ptr_->move(new_center);
    RCLCPP_DEBUG(this->get_logger(),
                "Recentered map by %.2f meters (threshold %.2f meters).",
                distance, move_threshold);
  }
  else
  {
    RCLCPP_DEBUG(this->get_logger(),
                 "Map recenter skipped. Distance %.2f meters < threshold %.2f meters.",
                 distance, move_threshold);
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

  RCLCPP_DEBUG(this->get_logger(),
              "Processing radar sector at time %u.%u. Queue size: %zu",
              msg->header.stamp.sec,
              msg->header.stamp.nanosec,
              radar_sector_queue_.size());

  // Move map if needed
  double x = transform.transform.translation.x;
  double y = transform.transform.translation.y;
  grid_map::Position new_center(x, y);
  recenterMap(new_center);



  double yaw, roll, ptich;
  {
    tf2::Quaternion q(
        transform.transform.rotation.x,
        transform.transform.rotation.y,
        transform.transform.rotation.z,
        transform.transform.rotation.w);
    tf2::Matrix3x3(q).getRPY(roll, ptich, yaw);
  }

  grid_map::Polygon sector_polygon;

  // Build triangle vertices
  grid_map::Position p0(transform.transform.translation.x, transform.transform.translation.y);

  double left_angle = msg->angle_start + yaw;
  double right_angle = msg->angle_start + (msg->intensities.size() - 1) * msg->angle_increment + yaw;
  double max_range = msg->range_max * 1.3; // Stretch a little to ensure coverage

  grid_map::Position p1(
      transform.transform.translation.x + max_range * std::cos(left_angle),
      transform.transform.translation.y + max_range * std::sin(left_angle));

  grid_map::Position p2(
      transform.transform.translation.x + max_range * std::cos(right_angle),
      transform.transform.translation.y + max_range * std::sin(right_angle));

  sector_polygon.addVertex(p0);
  sector_polygon.addVertex(p1);
  sector_polygon.addVertex(p2);

  // Precompute range increment
  float range_increment = (msg->range_max - msg->range_min);
  if (!msg->intensities.empty())
    range_increment /= static_cast<float>(msg->intensities.front().echoes.size());

  // Iterate over all map cells inside the sector
  for (grid_map::PolygonIterator it(*map_ptr_, sector_polygon); !it.isPastEnd(); ++it)
  {
    map_ptr_->at("intensity", *it) = NAN;
    grid_map::Position pos;
    map_ptr_->getPosition(*it, pos);

    // Compute range and angle from radar
    double dx = pos.x() - transform.transform.translation.x;
    double dy = pos.y() - transform.transform.translation.y;
    double range = std::hypot(dx, dy);
    // Skip if clearly out of radar range
    if (range > msg->range_max){
      continue;
    }

    double angle = std::atan2(dy, dx) - yaw;

    // Normalize angle into [0, 2*pi] range
    while (angle < 0.0)
      angle += 2.0 * M_PI;
    while (angle > 2.0 * M_PI)
      angle -= 2.0 * M_PI;

    // Normalize sector start/end
    double sector_start = msg->angle_start;
    double sector_end = msg->angle_start + msg->angle_increment * (msg->intensities.size() - 1);

    // Normalize sector limits into [0, 2*pi] as well
    while (sector_start < 0.0)
      sector_start += 2.0 * M_PI;
    while (sector_end > 2.0 * M_PI)
      sector_end -= 2.0 * M_PI;

    // Check if this map cell is inside the radar sector
    // bool inside_sector = false;
    // if (sector_start <= sector_end)
    //   inside_sector = (angle >= sector_start && angle <= sector_end);
    // else
    //   inside_sector = (angle >= sector_start || angle <= sector_end);

    // if (!inside_sector)
    //   continue; // skip cells outside beam sector

    // Find nearest beam index
    int i = static_cast<int>((angle - msg->angle_start) / msg->angle_increment + 0.5); // nearest neighbor
    if (i < 0 || i >= static_cast<int>(msg->intensities.size()))
      continue;

    // Find nearest range bin
    int j = static_cast<int>((range - msg->range_min) / range_increment + 0.5);
    if (j < 0 || j >= static_cast<int>(msg->intensities[i].echoes.size()))
      continue;

    // Fetch echo intensity
    float echo_intensity = msg->intensities[i].echoes[j];

    // Write to map
    if (echo_intensity > 0.0f)
      map_ptr_->at("intensity", *it) = echo_intensity;
    else
      map_ptr_->at("intensity", *it) = NAN; // Mark empty or unknown
  }

}


NS_FOOT
