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
  setupParam(&map_length_, node, "map_parameters.length", 0.0f);
  setupParam(&map_width_, node, "map_parameters.width", 0.0f);
  setupParam(&map_resolution_, node, "map_parameters.resolution", 0.1f);
  setupParam(&map_initial_position_x_, node, "map_parameters.initial_position_x", 0.0f);
  setupParam(&map_initial_position_y_, node, "map_parameters.initial_position_y", 0.0f);
}

RadarGridMapNode::RadarGridMapNode()
    : Node("radar_grid_map"),
    radar_map_({"halo_a", "halo_b"}) // todo why isn't setBasicLayers working? can't get declaration working in any other way
    // need to revisit this to be able to change layer names based on the number of radar frequencies etc 
{
    // Initialize parameters 
    parameters_.init(this);

    // TODO: should first check to make sure both radar frequency topics exist, generalize to other data topic names 
    // also generalize code to single-frequency radars but just getting this working for now
    // Should this generalize to any-frequency number radars? (e.g. are there even more than dual-frequency radars)
    //radar_map_.setBasicLayers({"halo_a", "halo_b"});
    radar_map_.setFrameId("map");
    radar_map_.setGeometry(grid_map::Length(parameters_.map_length_, parameters_.map_width_), parameters_.map_resolution_, 
                           grid_map::Position(parameters_.map_initial_position_x_, parameters_.map_initial_position_y_));

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

    grid_map_publisher_ = this->create_publisher<grid_map_msgs::msg::GridMap>(
                          "radar_grid_map", rclcpp::QoS(1).transient_local());

    // subscribe to radar data sector 
    // TODO: do this for each radar frequency -- testing with halo_a for now 
    radar_sector_subscriber_ = this->create_subscription<marine_sensor_msgs::msg::RadarSector>(
                               "data", 50, std::bind(&RadarGridMapNode::radarSectorCallback, this, _1));                         

    // subscribe to radar point cloud
    // TODO: do this for each radar frequency 
    radar_pointcloud_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                                   "/pointcloud", 50, std::bind(&RadarGridMapNode::radarPointCloudCallback, this, _1));


}

void RadarGridMapNode::radarSectorCallback(const marine_sensor_msgs::msg::RadarSector::SharedPtr msg)
{

    rclcpp::Clock clock;
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
        
        std::cerr << "start position: " << start_position(0) << ", " << start_position(1) << std::endl;
        std::cerr << "end_position: " << end_position(0) << ", " << end_position(1) << std::endl;       
        std::cerr << "end_position_shifted: " << start_position(0) + end_position(0) << ", " << start_position(1) + end_position(1) << std::endl; 

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

    // Publish grid map 
    radar_map_.setTimestamp(msg->header.stamp.nanosec);
    std::unique_ptr<grid_map_msgs::msg::GridMap> grid_map_msg = grid_map::GridMapRosConverter::toMessage(radar_map_);
    this->grid_map_publisher_->publish(std::move(grid_map_msg));
    RCLCPP_INFO_THROTTLE(this->get_logger(), clock, 1000, "Grid map published.");

}

void RadarGridMapNode::radarPointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    rclcpp::Clock clock;

    // Iterate through cloud to get x/y/intensities 
    sensor_msgs::PointCloud2Iterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(*msg, "y");
    // sensor_msgs::PointCloud2Iterator<float> iter_z(*msg, "z"); //  z values all zero, ignore for now
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_intensity(*msg, "intensity");

    // Set up line iterator for grid_map -- point cloud data will be ordered in radar sector, so match iterator w/ radar sweep

    grid_map::Matrix& map_layer_data = radar_map_["halo_a"]; // todo genericize layers

    // todo: update this with robot pose 
    //grid_map::Index start_position();


    /*for (grid_map::LineIterator iter_map(radar_map_, start_position, end_position); 
        !iter_map.isPastEnd(); ++iter_map)
    {

        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y)
        {

        }


    }*/

    // Iterate through grid_map 
    for (grid_map::GridMapIterator iter_map(radar_map_); !iter_map.isPastEnd(); ++iter_map)
    {
        grid_map::Position map_position;
        radar_map_.getPosition(*iter_map, map_position);
        // todo genericize layers
        //radar_map_.at("halo_a", *iter_map)
    }

    /*for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_intensity)
    {
      //std::cerr << "x pt: " << *iter_x << std::endl;
      //std::cerr << "y pt: " << *iter_y << std::endl;
      //std::cerr << "z pt: " << *iter_z << std::endl;
      //std::cerr << "intensity: " << *iter_intensity << std::endl;

    }*/



}

NS_FOOT
