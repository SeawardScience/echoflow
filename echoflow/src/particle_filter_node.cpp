/** Copyright © 2015 Seaward Science. */

#include "particle_filter_node.hpp"

NS_HEAD

void ParticleFilterNode::Parameters::declare(rclcpp::Node * node)
{
  node->declare_parameter("particle_filter.num_particles", particle_filter.num_particles);
  node->declare_parameter("particle_filter.update_interval", particle_filter.update_interval);
  node->declare_parameter("particle_filter.initial_max_speed", particle_filter.initial_max_speed);
  node->declare_parameter("particle_filter.observation_sigma", particle_filter.observation_sigma);
  node->declare_parameter("particle_filter.weight_decay_half_life", particle_filter.weight_decay_half_life);
  node->declare_parameter("particle_filter.seed_fraction", particle_filter.seed_fraction);
  node->declare_parameter("particle_filter.noise_std_pos", particle_filter.noise_std_pos);
  node->declare_parameter("particle_filter.noise_std_yaw", particle_filter.noise_std_yaw);
  node->declare_parameter("particle_filter.noise_std_yaw_rate", particle_filter.noise_std_yaw_rate);
  node->declare_parameter("particle_filter.noise_std_speed", particle_filter.noise_std_speed);
  node->declare_parameter("particle_filter.maximum_target_size", particle_filter.maximum_target_size);
  node->declare_parameter("particle_filter.density_feedback_factor", particle_filter.density_feedback_factor);



  node->declare_parameter("particle_filter_statistics.frame_id", particle_filter_statistics.frame_id);
  node->declare_parameter("map.length", particle_filter_statistics.length);
  node->declare_parameter("map.width", particle_filter_statistics.width);
  node->declare_parameter("particle_filter_statistics.resolution", particle_filter_statistics.resolution);
  node->declare_parameter("particle_filter_statistics.pub_interval", particle_filter_statistics.pub_interval);

}

void ParticleFilterNode::Parameters::update(rclcpp::Node * node)
{
  node->get_parameter("particle_filter.num_particles", particle_filter.num_particles);
  node->get_parameter("particle_filter.update_interval", particle_filter.update_interval);
  node->get_parameter("particle_filter.initial_max_speed", particle_filter.initial_max_speed);
  node->get_parameter("particle_filter.observation_sigma", particle_filter.observation_sigma);
  node->get_parameter("particle_filter.weight_decay_half_life", particle_filter.weight_decay_half_life);
  node->get_parameter("particle_filter.seed_fraction", particle_filter.seed_fraction);
  node->get_parameter("particle_filter.noise_std_pos", particle_filter.noise_std_pos);
  node->get_parameter("particle_filter.noise_std_yaw", particle_filter.noise_std_yaw);
  node->get_parameter("particle_filter.noise_std_yaw_rate", particle_filter.noise_std_yaw_rate);
  node->get_parameter("particle_filter.noise_std_speed", particle_filter.noise_std_speed);
  node->get_parameter("particle_filter.maximum_target_size", particle_filter.maximum_target_size);
  node->get_parameter("particle_filter.density_feedback_factor", particle_filter.density_feedback_factor);



  node->get_parameter("particle_filter_statistics.frame_id", particle_filter_statistics.frame_id);
  node->get_parameter("map.length", particle_filter_statistics.length);
  node->get_parameter("map.width", particle_filter_statistics.width);
  node->get_parameter("particle_filter_statistics.resolution", particle_filter_statistics.resolution);
  node->get_parameter("particle_filter_statistics.pub_interval", particle_filter_statistics.pub_interval);
}

ParticleFilterNode::ParticleFilterNode()
      : Node("particle_filter")
{
  parameters_.declare(this);
  parameters_.update(this);

  pf_ = std::make_unique<MultiTargetParticleFilter>(parameters_.particle_filter.num_particles,
                                                  parameters_.particle_filter.initial_max_speed);

  applyParameters();  // set pf parameters initially

  // Register a "set parameter" callback (pre-commit) to check and log parameter changes
  parameters_on_set_callback_ = this->add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> &parameters) {
          rcl_interfaces::msg::SetParametersResult result;
          result.successful = true;

          auto paramChanged = [&parameters](const std::string &name) {
              return std::any_of(parameters.begin(), parameters.end(),
                                 [&name](const auto &p) { return p.get_name() == name; });
          };

          // Log each parameter change
          for (const auto &parameter : parameters) {
              RCLCPP_INFO(this->get_logger(), "Parameter '%s' changed to '%s'",
                          parameter.get_name().c_str(),
                          parameter.value_to_string().c_str());
          }

          // parameters that require restart
          if (paramChanged("particle_filter.num_particles") ||
              paramChanged("particle_filter.initial_max_speed")) {
              RCLCPP_WARN(this->get_logger(),
                          "Change to 'num_particles' or 'initial_max_speed' will not take effect until node is restarted.");
          }
          return result;
      });

  // Register an "on parameter event" callback (post-commit) to apply parameter change
  parameter_event_handler_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
  parameter_event_callback_handle_ = parameter_event_handler_->add_parameter_event_callback(
      [this](const rcl_interfaces::msg::ParameterEvent &event) {
          // Only respond to local parameter changes
          if (event.node != this->get_fully_qualified_name()) {
              return;
          }
          parameters_.update(this);
          applyParameters();
      });

  cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("particle_cloud", 10);
  cell_vector_field_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("cell_vector_field", 10);
  particle_vector_field_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("particle_vector_field", 10);

  // Timer for particle filter update function
  timer_ = create_wall_timer(
              std::chrono::milliseconds(static_cast<int>(
                                parameters_.particle_filter.update_interval * 1000)),
              std::bind(&ParticleFilterNode::update, this));

  // Initialize GridMap for keeping track of particle filter statistics
  pf_statistics_pub_ = this->create_publisher<grid_map_msgs::msg::GridMap>("particle_filter_statistics", 10);
  pf_statistics_.reset( new grid_map::GridMap({"particles_per_cell",
                                          "x_position_mean",        // Arithmetic mean of x-position of particles
                                          "x_position_ssdm",        // Sum of squared deviations from mean used for computing variance/stdev
                                          "x_position_std_dev",     // Standard deviation of x-position of particles
                                          "y_position_mean",
                                          "y_position_ssdm",
                                          "y_position_std_dev",
                                          "speed_mean",
                                          "speed_ssdm",
                                          "speed_std_dev",
                                          "course_mean",
                                          "course_std_dev",
                                          "course_sines",          // These layers store the particle course converted to Cartesian coordinates
                                          "course_cosines"         // for calculating the circular mean and standard deviation
  }));

  // Timer for computing and publishing particle filter statistics on user-settable time interval
  pf_statistics_timer_ = create_wall_timer(
                            std::chrono::milliseconds(static_cast<int>(
                                    parameters_.particle_filter_statistics.pub_interval * 1000)),
                            std::bind(&ParticleFilterNode::computeParticleFilterStatistics, this));

  pf_statistics_->setGeometry(grid_map::Length(parameters_.particle_filter_statistics.length,
                                               parameters_.particle_filter_statistics.width),
                              parameters_.particle_filter_statistics.resolution);
  pf_statistics_->setFrameId(parameters_.particle_filter_statistics.frame_id);

  last_update_time_ = now();
}

void ParticleFilterNode::applyParameters() {
    pf_->observation_sigma_ = parameters_.particle_filter.observation_sigma;
    pf_->weight_decay_half_life_ = parameters_.particle_filter.weight_decay_half_life;
    pf_->seed_fraction_ = parameters_.particle_filter.seed_fraction;
    pf_->noise_std_pos_ = parameters_.particle_filter.noise_std_pos;
    pf_->noise_std_yaw_ = parameters_.particle_filter.noise_std_yaw;
    pf_->noise_std_yaw_rate_ = parameters_.particle_filter.noise_std_yaw_rate;
    pf_->noise_std_speed_ = parameters_.particle_filter.noise_std_speed;
    pf_->density_feedback_factor_ = parameters_.particle_filter.density_feedback_factor;
    pf_->updateNoiseDistributions();
}

void ParticleFilterNode::update()
{
  auto now_time = now();
  double dt = (now_time - last_update_time_).seconds();
  last_update_time_ = now_time;

  echoflow::grid_map_filters::filterLargeBlobsFromLayer(*map_ptr_, "intensity", "targets", parameters_.particle_filter.maximum_target_size);
  echoflow::grid_map_filters::computeEDTFromIntensity(*map_ptr_, "targets", "edt");

  if (!initialized_) {
    pf_->initialize(map_ptr_);
    initialized_ = true;
  }
  pf_->resample(map_ptr_, pf_statistics_,dt);
  pf_->predict(dt);
  pf_->updateWeights(map_ptr_,pf_statistics_,dt);
  publishPointCloud();

  //pending_detections_.clear();
}

void ParticleFilterNode::computeParticleFilterStatistics()
{
  pf_statistics_->clearAll();
  const auto& particles = pf_->getParticles();

  // Initialize all cells containing particles to zero
  for (const auto& particle : particles) {
    grid_map::Position position(particle.x, particle.y);
    if (pf_statistics_->isInside(position)) {
      if (std::isnan(pf_statistics_->atPosition("particles_per_cell", position))) {
        for (const auto& layer : pf_statistics_->getLayers()) {
          pf_statistics_->atPosition(layer, position) = 0.0;
        }
      }
    }
  }

  // Prior means for computing mean and standard deviation
  float prior_x_position_mean = 0.0;
  float prior_y_position_mean = 0.0;
  float prior_speed_mean = 0.0;

  // Iterate through all particles and update the particle filter statistics grid map
  // Accumulate total particle count per cell, then update the means and standard deviations for all particle parameters.
  for (const auto& particle : particles) {
    grid_map::Position position(particle.x, particle.y);
    if (pf_statistics_->isInside(position)) {
      // Update the particle count and prior means for this cell
      pf_statistics_->atPosition("particles_per_cell", position)++;
      prior_x_position_mean = pf_statistics_->atPosition("x_position_mean", position);
      prior_y_position_mean = pf_statistics_->atPosition("y_position_mean", position);
      prior_speed_mean = pf_statistics_->atPosition("speed_mean", position);

      // Update sequential arithmetic means for x position, y position, particle speed
      pf_statistics_->atPosition("x_position_mean", position) = echoflow::statistics::computeSequentialMean(
                                 particle.x,
                                 pf_statistics_->atPosition("particles_per_cell", position),
                                 prior_x_position_mean);
      pf_statistics_->atPosition("y_position_mean", position) = echoflow::statistics::computeSequentialMean(
                                 particle.y,
                                 pf_statistics_->atPosition("particles_per_cell", position),
                                 prior_y_position_mean);
      pf_statistics_->atPosition("speed_mean", position) = echoflow::statistics::computeSequentialMean(
                                 particle.speed,
                                 pf_statistics_->atPosition("particles_per_cell", position),
                                 prior_speed_mean);

      // Update sum of squared deviations from mean and standard deviations
      // for x position, y position, particle speed
      auto [x_std_dev, x_ssdm] = echoflow::statistics::computeSequentialStdDev(particle.x,
                                                         pf_statistics_->atPosition("particles_per_cell", position),
                                                         prior_x_position_mean,
                                                         pf_statistics_->atPosition("x_position_mean", position),
                                                         pf_statistics_->atPosition("x_position_ssdm", position));
      pf_statistics_->atPosition("x_position_ssdm", position) = x_ssdm;
      pf_statistics_->atPosition("x_position_std_dev", position) = x_std_dev;

      auto [y_std_dev, y_ssdm] = echoflow::statistics::computeSequentialStdDev(particle.y,
                                                         pf_statistics_->atPosition("particles_per_cell", position),
                                                         prior_y_position_mean,
                                                         pf_statistics_->atPosition("y_position_mean", position),
                                                         pf_statistics_->atPosition("y_position_ssdm", position));
      pf_statistics_->atPosition("y_position_ssdm", position) = y_ssdm;
      pf_statistics_->atPosition("y_position_std_dev", position) = y_std_dev;

      auto [speed_std_dev, speed_ssdm] = echoflow::statistics::computeSequentialStdDev(particle.speed,
                                                         pf_statistics_->atPosition("particles_per_cell", position),
                                                         prior_speed_mean,
                                                         pf_statistics_->atPosition("speed_mean", position),
                                                         pf_statistics_->atPosition("speed_ssdm", position));
      pf_statistics_->atPosition("speed_ssdm", position) = speed_ssdm;
      pf_statistics_->atPosition("speed_std_dev", position) = speed_std_dev;

      // Sum course angle sines and cosines for each cell
      // This is effectively converting a course angle in polar coordinates to Cartesian coordinates
      // in order to compute the arithmetic mean of the course angles
      pf_statistics_->atPosition("course_sines", position) += sin(particle.course);
      pf_statistics_->atPosition("course_cosines", position) += cos(particle.course);
    }
  }

  // Iterate through grid map and compute circular means and standard deviations for course angle
  for (grid_map::GridMapIterator iterator(*pf_statistics_); !iterator.isPastEnd(); ++iterator) {

    // Only calculate statistics for cells where there are particles
    if (pf_statistics_->at("particles_per_cell", *iterator) > 0) {

      pf_statistics_->at("course_mean", *iterator) = echoflow::statistics::computeCircularMean(
                                                            pf_statistics_->at("course_sines", *iterator),
                                                            pf_statistics_->at("course_cosines", *iterator));

      pf_statistics_->at("course_std_dev", *iterator) = echoflow::statistics::computeCircularStdDev(
                                                            pf_statistics_->at("course_sines", *iterator),
                                                            pf_statistics_->at("course_cosines", *iterator),
                                                            pf_statistics_->at("particles_per_cell", *iterator));

    // Otherwise leave cell with NaN value and move on to the next cell
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

  publishParticleVectorField();
  publishCellVectorField();
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
          8,  // number of fields
          "x", 1, sensor_msgs::msg::PointField::FLOAT32,
          "y", 1, sensor_msgs::msg::PointField::FLOAT32,
          "z", 1, sensor_msgs::msg::PointField::FLOAT32,
          "course", 1, sensor_msgs::msg::PointField::FLOAT32,
          "speed", 1, sensor_msgs::msg::PointField::FLOAT32,
          "yaw_rate", 1, sensor_msgs::msg::PointField::FLOAT32,
          "weight", 1, sensor_msgs::msg::PointField::FLOAT32,
          "age", 1, sensor_msgs::msg::PointField::FLOAT32);
  modifier.resize(particles.size());

  sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
  sensor_msgs::PointCloud2Iterator<float> iter_course(cloud, "course");
  sensor_msgs::PointCloud2Iterator<float> iter_speed(cloud, "speed");
  sensor_msgs::PointCloud2Iterator<float> iter_yaw_rate(cloud, "yaw_rate");
  sensor_msgs::PointCloud2Iterator<float> iter_weight(cloud, "weight");
  sensor_msgs::PointCloud2Iterator<float> iter_age(cloud, "age");

  for (const auto& particle : particles) {
    *iter_x = static_cast<float>(particle.x);
    *iter_y = static_cast<float>(particle.y);
    *iter_z = 0.0f;
    *iter_course = static_cast<float>(particle.course);
    *iter_speed = static_cast<float>(particle.speed);
    *iter_yaw_rate = static_cast<float>(particle.yaw_rate);
    *iter_weight = static_cast<float>(particle.weight);
    *iter_age = static_cast<float>(particle.age);
    ++iter_x; ++iter_y; ++iter_z;
    ++iter_course; ++iter_speed; ++iter_yaw_rate; ++iter_weight; ++iter_age;
  }

  cloud_pub_->publish(cloud);
}

void ParticleFilterNode::publishParticleVectorField()
{
  const auto& particles = pf_->getParticles();
  geometry_msgs::msg::PoseArray vector_field;
  vector_field.header.frame_id = "map";
  vector_field.header.stamp = this->get_clock()->now();

  geometry_msgs::msg::Pose pose;
  geometry_msgs::msg::Quaternion quaternion;
  for (const auto& particle : particles) {
    pose.position.x = particle.x;
    pose.position.y = particle.y;
    pose.position.z = 0.0f;
    quaternion = angleToYawQuaternion(particle.course);
    pose.orientation.x = quaternion.x;
    pose.orientation.y = quaternion.y;
    pose.orientation.z = quaternion.z;
    pose.orientation.w = quaternion.w;
    vector_field.poses.push_back(pose);
  }

  particle_vector_field_pub_->publish(vector_field);
}

void ParticleFilterNode::publishCellVectorField()
{
  geometry_msgs::msg::PoseArray vector_field;
  vector_field.header.frame_id = "map";
  vector_field.header.stamp = this->get_clock()->now();

  geometry_msgs::msg::Pose pose;
  geometry_msgs::msg::Quaternion quaternion;
  for (grid_map::GridMapIterator iterator(*pf_statistics_); !iterator.isPastEnd(); ++iterator) {
    if (std::isnan(pf_statistics_->at("particles_per_cell", *iterator))) {
      continue;
    } else {
      pose.position.x = pf_statistics_->at("x_position_mean", *iterator);
      pose.position.y = pf_statistics_->at("y_position_mean", *iterator);
      pose.position.z = 0.0f;
      quaternion = angleToYawQuaternion(pf_statistics_->at("course_mean", *iterator));
      pose.orientation.x = quaternion.x;
      pose.orientation.y = quaternion.y;
      pose.orientation.z = quaternion.z;
      pose.orientation.w = quaternion.w;
      vector_field.poses.push_back(pose);
    }
  }

  cell_vector_field_pub_->publish(vector_field);
}

geometry_msgs::msg::Quaternion ParticleFilterNode::angleToYawQuaternion(float angle)
{
  geometry_msgs::msg::Quaternion quaternion;
  quaternion.x = 0.0f;
  quaternion.y = 0.0f;
  quaternion.z = sin(angle / 2);
  quaternion.w = cos(angle / 2);
  return quaternion;
}

NS_FOOT
