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
}

RadarGridMapNode::RadarGridMapNode()
    : Node("radar_grid_map")
{
    parameters_.init(this);

    grid_map_publisher_ = this->create_publisher<grid_map_msgs::msg::GridMap>(
                          "radar_grid_map", rclcpp::QoS(1).transient_local());

    position_subscriber_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
                           "pose", 10, std::bind(&RadarGridMapNode::poseCallback, this, _1));

    radar_sector_subscriber_ = this->create_subscription<marine_sensor_msgs::msg::RadarSector>(
                               "data", 50, std::bind(&RadarGridMapNode::radarSectorCallback, this, _1));



    map_ptr_.reset(new grid_map::GridMap({"elevation"}));
    map_ptr_->setFrameId(parameters_.map.frame_id);
    map_ptr_->setGeometry(grid_map::Length(parameters_.map.length, parameters_.map.width), parameters_.map.resolution);
    RCLCPP_INFO(
        this->get_logger(),
        "Created map with size %f x %f m (%i x %i cells).",
        map_ptr_->getLength().x(), map_ptr_->getLength().y(),
        map_ptr_->getSize()(0), map_ptr_->getSize()(1));


    scan_timer_ = this->create_wall_timer(std::chrono::seconds(scan_time_),
                    std::bind(&RadarGridMapNode::scanTimerCallback, this));
                      

}

void RadarGridMapNode::waitForTopics() {

}

void RadarGridMapNode::scanTimerCallback()
{

}

void RadarGridMapNode::poseCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{

}

void RadarGridMapNode::radarSectorCallback(const marine_sensor_msgs::msg::RadarSector::SharedPtr msg)
{

}

NS_FOOT
