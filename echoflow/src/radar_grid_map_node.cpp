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
  setupParam (&map_frame_id_, node, "map_parameters.frameId", "map");
  setupParam(&map_length_, node, "map_parameters.length", 0.0f);
  setupParam(&map_width_, node, "map_parameters.width", 0.0f);
  setupParam(&map_resolution_, node, "map_parameters.resolution", 0.1f);
  setupParam(&map_publish_time_interval_, node, "publisher_parameters.timeInterval", (int64_t) 1); // TODO default should be scan_time
}

RadarGridMapNode::RadarGridMapNode()
    : Node("radar_grid_map"),
    radar_map_({"halo_a", "halo_b"}) // todo why isn't setBasicLayers working? can't get declaration working in any other way
    // need to revisit this to be able to change layer names based on the number of radar frequencies etc 
{


    waitForTopics();


    // Initialize parameters 
    parameters_.init(this);




    grid_map_publisher_ = this->create_publisher<grid_map_msgs::msg::GridMap>(
                          "radar_grid_map", rclcpp::QoS(1).transient_local());

    position_subscriber_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
                           "pose", 10, std::bind(&RadarGridMapNode::poseCallback, this, _1));

    radar_sector_subscriber_ = this->create_subscription<marine_sensor_msgs::msg::RadarSector>(
                               "data", 50, std::bind(&RadarGridMapNode::radarSectorCallback, this, _1));



    // have each node run one complete radar, run multiple nodes one per frequency 

    // TODO: Wait to initialize until a pose  is received
    // Wait to initialize map until pose received from tf




    // TODO: should first check to make sure both radar frequency topics exist, generalize to other data topic names 
    // also generalize code to single-frequency radars but just getting this working for now
    // Should this generalize to any-frequency number radars? (e.g. are there even more than dual-frequency radars)
    //radar_map_.setBasicLayers({"halo_a", "halo_b"});
    radar_map_.setFrameId(parameters_.map_frame_id_);
    radar_map_.setGeometry(grid_map::Length(parameters_.map_length_, parameters_.map_width_), parameters_.map_resolution_, 
                           grid_map::Position(0, 0)); // TODO: set position with initial pose 

    RCLCPP_INFO(this->get_logger(), 
                "Created map with size %f x %f [m] (%i x %i cells). \n"
                " Map center located at (%f, %f) in the %s frame.",
                radar_map_.getLength().x(), 
                radar_map_.getLength().y(),
                radar_map_.getSize()(0),
                radar_map_.getSize()(1),
                radar_map_.getPosition().x(),
                radar_map_.getPosition().y(),
                radar_map_.getFrameId().c_str()
    );

    radar_map_.clearAll();

   


    // TODO: wait to initialize timer until RadarSector scan_time received

    scan_timer_ = this->create_wall_timer(std::chrono::seconds(scan_time_),
                    std::bind(&RadarGridMapNode::scanTimerCallback, this));
                      

}

void RadarGridMapNode::waitForTopics() {
    std::string pose_topic = "pose";
    std::string data_topic = "data";

    auto pose_msg = sensor_msgs::msg::NavSatFix();

    // TODO: Wait for the position topic to be published before initializing 
    while(!pose_initialized_)
    {
        pose_initialized_ = true;
        // TODO: apparently I don't know how to do this in ROS2 
        // This is currently overwriting any user-specified parameter. 
        std::cerr << "made it here!" << std::endl;
        rclcpp::wait_for_message(pose_msg, this->shared_from_this(), pose_topic, std::chrono::seconds(1));
        std::cerr << "wait for message called!" << std::endl;
        RCLCPP_WARN(this->get_logger(), "Waiting for initial pose.");
    }
    
}

void RadarGridMapNode::scanTimerCallback()
{

    // Publish grid_map 
    radar_map_.setTimestamp(this->now().nanoseconds());
    std::unique_ptr<grid_map_msgs::msg::GridMap> grid_map_msg = grid_map::GridMapRosConverter::toMessage(radar_map_);
    this->grid_map_publisher_->publish(std::move(grid_map_msg));
    RCLCPP_INFO_THROTTLE(this->get_logger(), clock_, 1000, "Radar grid map published.");
}

void RadarGridMapNode::poseCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{


}

void RadarGridMapNode::radarSectorCallback(const marine_sensor_msgs::msg::RadarSector::SharedPtr msg)
{

    if(!timer_interval_initialized_)
    {
        scan_time_ = msg->scan_time.nanosec * 10e9;     // scan time in seconds 
    }


    // Use a line iterator to fill in each ray of the radar sector 
    // The start index of the line iterator is the current robot pose 
    grid_map::Index start_position(1000.0, 1000.0);     // TODO! update with current robot pose 

    // The end index of the line iterator is x: max_range*cosine(angle); y = max_range*sine(angle)

    // Iterate through each ray of the radar sector to fill the map 
    //for (size_t i = 0; i < msg->intensities.size(); i++)
    //{
        //double ray_angle = msg->angle_start + i * msg->angle_increment;
        double ray_angle = msg->angle_start;

        // Compute the start and end indices of the grid_map line iterator from the radar sector range and angles
        grid_map::Index end_position(msg->range_max*cos(ray_angle), msg->range_max*sin(ray_angle));
        
        //std::cerr << "start position: " << start_position(0) << ", " << start_position(1) << std::endl;
        //std::cerr << "end_position: " << end_position(0) << ", " << end_position(1) << std::endl;       
        //std::cerr << "end_position_shifted: " << start_position(0) + end_position(0) << ", " << start_position(1) + end_position(1) << std::endl; 

        std::cerr << "time_increment: " << msg->time_increment.nanosec << std::endl;
        std::cerr << "scan_time: " << msg->scan_time.nanosec << std::endl;

        // Important: keep track of cells that have been modified 
        // be able to publish modified cells 

        for (grid_map::LineIterator map_iterator(radar_map_, start_position, end_position);
            !map_iterator.isPastEnd(); ++map_iterator)
        {
            //std::cerr << "iterator position: " << *map_iterator << std::endl;
            
            const grid_map::Index index(*map_iterator);

            if (!radar_map_.isValid(index)) {
                //std::cerr << "invalid index!" << std::endl;
                //continue;
            } else {
                radar_map_.at("halo_a", *map_iterator) = 1.0;
            }
            
            /*try 
            {
                radar_map_.at("halo_a", *map_iterator) = 1.0;

            } catch (const std::out_of_range& exception) {
                std::cerr << "Exception: " << exception.what() << std::endl;
            }*/
        }


    //}



}

NS_FOOT
