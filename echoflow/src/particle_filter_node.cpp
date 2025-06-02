#include "particle_filter_node.hpp"

NS_HEAD

void ParticleFilterNode::Parameters::declare(rclcpp::Node * node)
{
  node->declare_parameter("particle_filter.num_particles", particle_filter.num_particles);
  node->declare_parameter("particle_filter.update_interval", particle_filter.update_interval);
  node->declare_parameter("particle_filter_statistics.frameId", particle_filter_statistics.frameId);
  node->declare_parameter("particle_filter_statistics.length", particle_filter_statistics.length);
  node->declare_parameter("particle_filter_statistics.width", particle_filter_statistics.width);
  node->declare_parameter("particle_filter_statistics.resolution", particle_filter_statistics.resolution);
  node->declare_parameter("particle_filter_statistics.pub_interval", particle_filter_statistics.pub_interval);
}

void ParticleFilterNode::Parameters::update(rclcpp::Node * node)
{
  node->get_parameter("particle_filter.num_particles", particle_filter.num_particles);
  node->get_parameter("particle_filter.update_interval", particle_filter.update_interval);
  node->get_parameter("particle_filter_statistics.frameId", particle_filter_statistics.frameId);
  node->get_parameter("particle_filter_statistics.length", particle_filter_statistics.length);
  node->get_parameter("particle_filter_statistics.width", particle_filter_statistics.width);
  node->get_parameter("particle_filter_statistics.resolution", particle_filter_statistics.resolution);
  node->get_parameter("particle_filter_statistics.pub_interval", particle_filter_statistics.pub_interval);
}

ParticleFilterNode::ParticleFilterNode()
      : Node("particle_filter")
{
  parameters_.declare(this);
  parameters_.update(this);

  pf_ = std::make_unique<MultiTargetParticleFilter>(parameters_.particle_filter.num_particles);

  cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("particle_cloud", 10);
  cell_heading_field_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("cell_heading_field", 10);
  particle_heading_field_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("particle_heading_field", 10);

  // Timer for particle filter update function
  timer_ = create_wall_timer(
              std::chrono::milliseconds(static_cast<int>(
                                parameters_.particle_filter.update_interval * 1000)),
              std::bind(&ParticleFilterNode::update, this));

  // Initialize GridMap for keeping track of particle filter statistics
  pf_statistics_pub_ = this->create_publisher<grid_map_msgs::msg::GridMap>("particle_filter_statistics", 10);
  pf_statistics_ = new grid_map::GridMap({"particles_per_cell",
                                          "x_position_mean",        // Arithmetic mean of x-position of particles
                                          "x_position_ssdm",        // Sum of squared deviations from mean used for computing variance/stdev
                                          "x_position_std_dev",     // Standard deviation of x-position of particles
                                          "y_position_mean",
                                          "y_position_ssdm",
                                          "y_position_std_dev",
                                          "speed_mean",
                                          "speed_ssdm",
                                          "speed_std_dev",
                                          "heading_mean",
                                          "heading_std_dev",
                                          "heading_sines",          // These layers store the heading converted to Cartesian coordinates
                                          "heading_cosines"         // for calculating the circular mean and standard deviation
                                          });

  // Timer for computing and publishing particle filter statistics on user-settable time interval
  pf_statistics_timer_ = create_wall_timer(
                            std::chrono::milliseconds(static_cast<int>(
                                    parameters_.particle_filter_statistics.pub_interval * 1000)),
                            std::bind(&ParticleFilterNode::computeParticleFilterStatistics, this));

  pf_statistics_->setGeometry(grid_map::Length(parameters_.particle_filter_statistics.length,
                                               parameters_.particle_filter_statistics.width),
                              parameters_.particle_filter_statistics.resolution);
  pf_statistics_->setFrameId(parameters_.particle_filter_statistics.frameId);

  last_update_time_ = now();
}

void ParticleFilterNode::update()
{
  auto now_time = now();
  double dt = (now_time - last_update_time_).seconds();
  last_update_time_ = now_time;

  computeEDTFromIntensity(*map_ptr_, "intensity", "edt");

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

  publishPointCloud();
}

void ParticleFilterNode::computeParticleFilterStatistics()
{
  pf_statistics_->clearAll();
  const auto& particles = pf_->getParticles();

  // Zero out all cells in the particle statistics grid before re-computing statistics
  for (const auto& layer : pf_statistics_->getLayers()) {
    (*pf_statistics_)[layer].setConstant(0.0);
  }

  // Iterate through all particles and update the particle filter statistics grid map
  // Accumulate total particle count per cell, then update the arithmetic means and
  // standard deviations. Also convert headings from polar to Cartesian coordinates and store
  for (const auto& particle : particles) {
    grid_map::Position position(particle.x, particle.y);
    if (pf_statistics_->isInside(position)) {

      // Update count of particles per cell
      pf_statistics_->atPosition("particles_per_cell", position)++;

      // Store prior means for running standard deviation computation
      float prior_x_position_mean = pf_statistics_->atPosition("x_position_mean", position);
      float prior_y_position_mean = pf_statistics_->atPosition("y_position_mean", position);
      float prior_speed_mean = pf_statistics_->atPosition("speed_mean", position);

      // Update sequential arithmetic means for x position, y position, particle speed
      pf_statistics_->atPosition("x_position_mean", position) = computeSequentialMean(
                                 particle.x,
                                 pf_statistics_->atPosition("particles_per_cell", position),
                                 pf_statistics_->atPosition("x_position_mean", position));
      pf_statistics_->atPosition("y_position_mean", position) = computeSequentialMean(
                                 particle.y,
                                 pf_statistics_->atPosition("particles_per_cell", position),
                                 pf_statistics_->atPosition("y_position_mean", position));
      pf_statistics_->atPosition("speed_mean", position) = computeSequentialMean(
                                 particle.speed,
                                 pf_statistics_->atPosition("particles_per_cell", position),
                                 pf_statistics_->atPosition("speed_mean", position));

      // Update sum of squared deviations from mean and standard deviations
      // for x position, y position, particle speed
      auto [x_std_dev, x_ssdm] = computeSequentialStdDev(particle.x,
                                                         pf_statistics_->atPosition("particles_per_cell", position),
                                                         prior_x_position_mean,
                                                         pf_statistics_->atPosition("x_position_mean", position),
                                                         pf_statistics_->atPosition("x_position_ssdm", position));
      pf_statistics_->atPosition("x_position_ssdm", position) = x_ssdm;
      pf_statistics_->atPosition("x_position_std_dev", position) = x_std_dev;

      auto [y_std_dev, y_ssdm] = computeSequentialStdDev(particle.y,
                                                         pf_statistics_->atPosition("particles_per_cell", position),
                                                         prior_y_position_mean,
                                                         pf_statistics_->atPosition("y_position_mean", position),
                                                         pf_statistics_->atPosition("y_position_ssdm", position));
      pf_statistics_->atPosition("y_position_ssdm", position) = y_ssdm;
      pf_statistics_->atPosition("y_position_std_dev", position) = y_std_dev;

      auto [speed_std_dev, speed_ssdm] = computeSequentialStdDev(particle.speed,
                                                         pf_statistics_->atPosition("particles_per_cell", position),
                                                         prior_speed_mean,
                                                         pf_statistics_->atPosition("speed_mean", position),
                                                         pf_statistics_->atPosition("speed_ssdm", position));
      pf_statistics_->atPosition("speed_ssdm", position) = speed_ssdm;
      pf_statistics_->atPosition("speed_std_dev", position) = speed_std_dev;

      // Sum heading sines and cosines for each cell
      // This is effectively converting a heading in polar coordinates to Cartesian coordinates
      // in order to compute the arithmetic mean of the headings
      pf_statistics_->atPosition("heading_sines", position) += sin(particle.heading);
      pf_statistics_->atPosition("heading_cosines", position) += cos(particle.heading);
    }
  }

  // Iterate through grid map and compute circular means and standard deviations for heading
  for (grid_map::GridMapIterator iterator(*pf_statistics_); !iterator.isPastEnd(); ++iterator) {

    // Only calculate statistics for cells where there are particles
    if (pf_statistics_->at("particles_per_cell", *iterator) > 0) {

      pf_statistics_->at("heading_mean", *iterator) = computeCircularMean(pf_statistics_->at("heading_sines", *iterator),
                                                                        pf_statistics_->at("heading_cosines", *iterator));

      pf_statistics_->at("heading_std_dev", *iterator) = computeCircularStdDev(pf_statistics_->at("heading_sines", *iterator),
                                                                               pf_statistics_->at("heading_cosines", *iterator),
                                                                               pf_statistics_->at("particles_per_cell", *iterator));

    // Otherwise leave cell with zero value and move on to the next cell
    } else {
      continue;
    }
  }

  // Update particle filter statistics grid map position and publish
  grid_map::Position radar_grid_map_center = map_ptr_->getPosition();
  pf_statistics_->move(radar_grid_map_center);
  std::unique_ptr<grid_map_msgs::msg::GridMap> message;
  message = grid_map::GridMapRosConverter::toMessage(*pf_statistics_);
  pf_statistics_pub_->publish(std::move(message));

  publishParticleHeadingField();
  publishCellHeadingField();
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

  for (const auto& particle : particles) {
    *iter_x = static_cast<float>(particle.x);
    *iter_y = static_cast<float>(particle.y);
    *iter_z = 0.0f;
    *iter_heading = static_cast<float>(particle.heading);
    *iter_speed = static_cast<float>(particle.speed);
    *iter_yaw_rate = static_cast<float>(particle.yaw_rate);
    *iter_weight = static_cast<float>(particle.weight);
    ++iter_x; ++iter_y; ++iter_z;
    ++iter_heading; ++iter_speed; ++iter_yaw_rate; ++iter_weight;
  }

  cloud_pub_->publish(cloud);
}

void ParticleFilterNode::publishParticleHeadingField()
{
  const auto& particles = pf_->getParticles();
  geometry_msgs::msg::PoseArray heading_field;
  heading_field.header.frame_id = "map";
  heading_field.header.stamp = this->get_clock()->now();

  geometry_msgs::msg::Pose pose;
  geometry_msgs::msg::Quaternion quaternion;
  for (const auto& particle : particles) {
    pose.position.x = particle.x;
    pose.position.y = particle.y;
    pose.position.z = 0.0f;
    quaternion = headingToQuaternion(particle.heading);
    pose.orientation.x = quaternion.x;
    pose.orientation.y = quaternion.y;
    pose.orientation.z = quaternion.z;
    pose.orientation.w = quaternion.w;
    heading_field.poses.push_back(pose);
  }

  particle_heading_field_pub_->publish(heading_field);
}

void ParticleFilterNode::publishCellHeadingField()
{
  geometry_msgs::msg::PoseArray heading_field;
  heading_field.header.frame_id = "map";
  heading_field.header.stamp = this->get_clock()->now();

  geometry_msgs::msg::Pose pose;
  geometry_msgs::msg::Quaternion quaternion;
  for (grid_map::GridMapIterator iterator(*pf_statistics_); !iterator.isPastEnd(); ++iterator) {
    pose.position.x = pf_statistics_->at("x_position_mean", *iterator);
    pose.position.y = pf_statistics_->at("y_position_mean", *iterator);
    pose.position.z = 0.0f;
    quaternion = headingToQuaternion(pf_statistics_->at("heading_mean", *iterator));
    pose.orientation.x = quaternion.x;
    pose.orientation.y = quaternion.y;
    pose.orientation.z = quaternion.z;
    pose.orientation.w = quaternion.w;
    heading_field.poses.push_back(pose);
  }

  cell_heading_field_pub_->publish(heading_field);
}

geometry_msgs::msg::Quaternion ParticleFilterNode::headingToQuaternion(float heading)
{
  geometry_msgs::msg::Quaternion quaternion;
  quaternion.x = 0.0f;
  quaternion.y = 0.0f;
  quaternion.z = sin(heading / 2);
  quaternion.w = cos(heading / 2);
  return quaternion;
}

NS_FOOT
