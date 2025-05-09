#include "particle_filter_node.hpp"

NS_HEAD

ParticleFilterNode::ParticleFilterNode()
      : Node("particle_filter_node")
{
  this->declare_parameter("num_particles", 100000);
  size_t num_particles = this->get_parameter("num_particles").as_int();
  pf_ = std::make_unique<MultiTargetParticleFilter>(num_particles);

  cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("particle_cloud", 10);

  timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&ParticleFilterNode::update, this));

  // Initialize GridMap for keeping track of particle filter statistics
  pf_statistics_pub_ = this->create_publisher<grid_map_msgs::msg::GridMap>("particle_filter_statistics", 10);
  pf_statistics_ = new grid_map::GridMap({"particles_per_cell",
                                          "average_x_position",
                                          "average_y_position",
                                          "average_heading",
                                          "average_velocity"});

  // TODO: need to let user specify size of this map, but for now just set position here
  pf_statistics_->setGeometry(grid_map::Length(1500, 1500), 10.0);
  pf_statistics_->setFrameId("map"); // todo get from params

  last_update_time_ = now();

}

void ParticleFilterNode::update()
{
  auto now_time = now();
  double dt = (now_time - last_update_time_).seconds();
  last_update_time_ = now_time;

  computeEDTFromIntensity(*map_ptr_,"intensity","edt");

  if (!initialized_) {
    pf_->initialize(map_ptr_);
    initialized_ = true;
  }

  if (!initialized_) return;

  pf_->initialize(map_ptr_);

  pf_->predict(dt);
  pf_->updateWeights(map_ptr_);
  pf_->resample();
  pending_detections_.clear();

  pf_statistics_->clearAll();
  std::cerr << "cleared statistics map" << std::endl;

  publishPointCloud();
}

void ParticleFilterNode::publishPointCloud()
{
  const auto& particles = pf_->getParticles();
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.frame_id = "map";
  cloud.header.stamp = this->get_clock()->now();
  cloud.height = 1;
  cloud.width = particles.size();

  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2Fields(
          7,  // number of fields
          "x", 1, sensor_msgs::msg::PointField::FLOAT32,
          "y", 1, sensor_msgs::msg::PointField::FLOAT32,
          "z", 1, sensor_msgs::msg::PointField::FLOAT32,
          "heading", 1, sensor_msgs::msg::PointField::FLOAT32,
          "speed", 1, sensor_msgs::msg::PointField::FLOAT32,
          "yaw_rate", 1, sensor_msgs::msg::PointField::FLOAT32,
          "weight", 1, sensor_msgs::msg::PointField::FLOAT32);
  modifier.resize(particles.size());

  sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
  sensor_msgs::PointCloud2Iterator<float> iter_heading(cloud, "heading");
  sensor_msgs::PointCloud2Iterator<float> iter_speed(cloud, "speed");
  sensor_msgs::PointCloud2Iterator<float> iter_yaw_rate(cloud, "yaw_rate");
  sensor_msgs::PointCloud2Iterator<float> iter_weight(cloud, "weight");

  for (const auto& p : particles)
  {
    *iter_x = static_cast<float>(p.x);
    *iter_y = static_cast<float>(p.y);
    *iter_z = 0.0f;
    *iter_heading = static_cast<float>(p.heading);
    *iter_speed = static_cast<float>(p.speed);
    *iter_yaw_rate = static_cast<float>(p.yaw_rate);
    *iter_weight = static_cast<float>(p.weight);
    ++iter_x; ++iter_y; ++iter_z;
    ++iter_heading; ++iter_speed; ++iter_yaw_rate; ++iter_weight;

    // Accumulate statistics on the current particles
    // TODO: currently uses same map size/resolution as grid map -- need to update this to
    // use different map sizes and resolutions from the grid map
    grid_map::Position pos(p.x, p.y);
    if (pf_statistics_->isInside(pos))
    {
      if (std::isnan(pf_statistics_->atPosition("particles_per_cell", pos)))
      {
        //std::cerr << "setting to zero" << std::endl;
        pf_statistics_->atPosition("particles_per_cell", pos) = 0;
        //std::cerr << pf_statistics_->atPosition("particles_per_cell", pos) << std::endl;
        pf_statistics_->atPosition("average_x_position", pos) = 0;
        pf_statistics_->atPosition("average_y_position", pos) = 0;
        pf_statistics_->atPosition("average_heading", pos) = 0;
        pf_statistics_->atPosition("average_velocity", pos) = 0;
      }
      else
      {
        pf_statistics_->atPosition("particles_per_cell", pos)++;
        pf_statistics_->atPosition("average_x_position", pos) = pf_statistics_->atPosition("average_x_position", pos) + p.x;
        pf_statistics_->atPosition("average_y_position", pos) += p.y;
        pf_statistics_->atPosition("average_heading", pos) += p.heading;
        pf_statistics_->atPosition("average_velocity", pos) += p.speed;
      }
    }

  }

  // Compute averages of pf statistics
  // TODO: to compute the standard deviation of each metric we're going to have to keep track of
  // the values for particles in each cell instead of just accumulating totals and averaging!
  // need to re-implement this
  // TODO: at some point the map is not getting repopulated after it's set to NAN on clearing, so these calculations become nan
  for (grid_map::GridMapIterator iterator(*pf_statistics_); !iterator.isPastEnd(); ++iterator)
  {
    pf_statistics_->at("average_x_position", *iterator) = pf_statistics_->at("average_x_position", *iterator) / pf_statistics_->at("particles_per_cell", *iterator);

  }

  // TODO: move this to own function once it's working
  //grid_map::Position radar_grid_map_center = map_ptr_->getPosition();
  //pf_statistics_->move(radar_grid_map_center);
  std::unique_ptr<grid_map_msgs::msg::GridMap> message;
  message = grid_map::GridMapRosConverter::toMessage(*pf_statistics_);
  pf_statistics_pub_->publish(std::move(message));

  cloud_pub_->publish(cloud);
}

NS_FOOT

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto filter_node = std::make_shared<echoflow::ParticleFilterNode>();
  auto grid_node = std::make_shared<echoflow::RadarGridMapNode>();

  filter_node->map_ptr_ = grid_node->getMapPtr();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(filter_node);
  executor.add_node(grid_node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
