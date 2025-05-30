#include "particle_filter_node.hpp"

NS_HEAD

void ParticleFilterNode::Parameters::declare(rclcpp::Node * node)
{
  node->declare_parameter("particle_filter.num_particles", particle_filter.num_particles);
  node->declare_parameter("particle_filter.update_interval", particle_filter.update_interval);

  node->declare_parameter("particle_filter.initial_max_speed", particle_filter.initial_max_speed);
  node->declare_parameter("particle_filter.observation_sigma", particle_filter.observation_sigma);
  node->declare_parameter("particle_filter.decay_factor", particle_filter.decay_factor);
  node->declare_parameter("particle_filter.min_resample_speed", particle_filter.min_resample_speed);
  node->declare_parameter("particle_filter.noise_std_pos", particle_filter.noise_std_pos);
  node->declare_parameter("particle_filter.noise_std_yaw", particle_filter.noise_std_yaw);
  node->declare_parameter("particle_filter.noise_std_yaw_rate", particle_filter.noise_std_yaw_rate);
  node->declare_parameter("particle_filter.noise_std_speed", particle_filter.noise_std_speed);

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

  node->get_parameter("particle_filter.initial_max_speed", particle_filter.initial_max_speed);
  node->get_parameter("particle_filter.observation_sigma", particle_filter.observation_sigma);
  node->get_parameter("particle_filter.decay_factor", particle_filter.decay_factor);
  node->get_parameter("particle_filter.min_resample_speed", particle_filter.min_resample_speed);
  node->get_parameter("particle_filter.noise_std_pos", particle_filter.noise_std_pos);
  node->get_parameter("particle_filter.noise_std_yaw", particle_filter.noise_std_yaw);
  node->get_parameter("particle_filter.noise_std_yaw_rate", particle_filter.noise_std_yaw_rate);
  node->get_parameter("particle_filter.noise_std_speed", particle_filter.noise_std_speed);

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

  pf_ = std::make_unique<MultiTargetParticleFilter>(parameters_.particle_filter.num_particles,
                                                  parameters_.particle_filter.initial_max_speed);

  applyParameters();  // set pf parameters initially

  parameter_event_sub_ = this->add_on_set_parameters_callback(
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
              // TODO: (bonney) handle param updates case by case
          }

          // parameters that require restart
          if (paramChanged("particle_filter.num_particles") ||
              paramChanged("particle_filter.initial_max_speed")) {
              RCLCPP_WARN(this->get_logger(),
                          "Change to 'num_particles' or 'initial_max_speed' will not take effect until node is restarted.");
          }

          parameters_.update(this);
          applyParameters(); // dynamically update particle filter parameters

          return result;
      });

  cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("particle_cloud", 10);

  // Timer for particle filter update function
  timer_ = create_wall_timer(
              std::chrono::milliseconds(static_cast<int>(
                                parameters_.particle_filter.update_interval * 1000)),
              std::bind(&ParticleFilterNode::update, this));

  // Initialize GridMap for keeping track of particle filter statistics
  pf_statistics_pub_ = this->create_publisher<grid_map_msgs::msg::GridMap>("particle_filter_statistics", 10);
  pf_statistics_ = new grid_map::GridMap({"particles_per_cell",
                                          "x_position_mean",
                                          "x_position_std_dev",
                                          "y_position_mean",
                                          "y_position_std_dev",
                                          "heading_mean",
                                          "heading_sines",   // TODO (antonella): this is a hacky way of keeping track of
                                          "heading_cosines", // sines and cosines in order to compute the circular mean
                                          "heading_std_dev", // and circ variance -- likely can be improved
                                          "velocity_mean",
                                          "velocity_std_dev"});

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

void ParticleFilterNode::applyParameters() {
    pf_->observation_sigma_ = parameters_.particle_filter.observation_sigma;
    pf_->decay_factor_ = parameters_.particle_filter.decay_factor;
    pf_->min_resample_speed_ = parameters_.particle_filter.min_resample_speed;
    pf_->noise_std_pos_ = parameters_.particle_filter.noise_std_pos;
    pf_->noise_std_yaw_ = parameters_.particle_filter.noise_std_yaw;
    pf_->noise_std_yaw_rate_ = parameters_.particle_filter.noise_std_yaw_rate;
    pf_->noise_std_speed_ = parameters_.particle_filter.noise_std_speed;
    pf_->updateNoiseDistributions();
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

  // Accumulate number of particles per cell and totals of the particle properties
  for (const auto& particle : particles) {
    grid_map::Position position(particle.x, particle.y);
    if (pf_statistics_->isInside(position)) {
      if (std::isnan(pf_statistics_->atPosition("particles_per_cell", position))) {
        pf_statistics_->atPosition("particles_per_cell", position) = 0;
        pf_statistics_->atPosition("x_position_mean", position) = 0;
        pf_statistics_->atPosition("x_position_std_dev", position) = 0;
        pf_statistics_->atPosition("y_position_mean", position) = 0;
        pf_statistics_->atPosition("y_position_std_dev", position) = 0;
        pf_statistics_->atPosition("heading_mean", position) = 0;
        pf_statistics_->atPosition("heading_sines", position) = 0;
        pf_statistics_->atPosition("heading_cosines", position) = 0;
        pf_statistics_->atPosition("heading_std_dev", position) = 0;
        pf_statistics_->atPosition("velocity_mean", position) = 0;
        pf_statistics_->atPosition("velocity_std_dev", position) = 0;
      } else {
        pf_statistics_->atPosition("particles_per_cell", position)++;
        pf_statistics_->atPosition("x_position_mean", position) += particle.x;
        pf_statistics_->atPosition("y_position_mean", position) += particle.y;
        // todo: re-factor this into something more sensible
        // accumulate sum of sines and cosines of heading
        pf_statistics_->atPosition("heading_sines", position) += sin(particle.heading);
        pf_statistics_->atPosition("heading_cosines", position) += cos(particle.heading);
        pf_statistics_->atPosition("velocity_mean", position) += particle.speed;
      }
    }
  }

  // Compute averages of pf statistics
  for (grid_map::GridMapIterator iterator(*pf_statistics_); !iterator.isPastEnd(); ++iterator) {
    pf_statistics_->at("x_position_mean", *iterator) = pf_statistics_->at("x_position_mean", *iterator)
                                                          / pf_statistics_->at("particles_per_cell", *iterator);
    pf_statistics_->at("y_position_mean", *iterator) = pf_statistics_->at("y_position_mean", *iterator)
                                                          / pf_statistics_->at("particles_per_cell", *iterator);
    // todo: this implementation can hopefully be improved
    // todo (antonella): look into eigen matrix functions, might be able to make some of this more effient w/o iterators
    pf_statistics_->at("heading_mean", *iterator) = atan2(pf_statistics_->at("heading_sines", *iterator),
                                                          pf_statistics_->at("heading_cosines", *iterator));
    // Compute C, S for circ std dev
    pf_statistics_->at("heading_sines", *iterator) = pf_statistics_->at("heading_sines", *iterator)
                                                    / pf_statistics_->at("particles_per_cell", *iterator);
    pf_statistics_->at("heading_cosines", *iterator) = pf_statistics_->at("heading_cosines", *iterator)
                                                    / pf_statistics_->at("particles_per_cell", *iterator);
    pf_statistics_->at("velocity_mean", *iterator) = pf_statistics_->at("velocity_mean", *iterator)
                                                        / pf_statistics_->at("particles_per_cell", *iterator);
  }

  // Accumulate squared deviations from the mean
  for (const auto& particle : particles) {
    grid_map::Position position(particle.x, particle.y);
    if (pf_statistics_->isInside(position)) {
      pf_statistics_->atPosition("x_position_std_dev", position) +=
          pow(particle.x - pf_statistics_->atPosition("x_position_mean", position), 2);
      pf_statistics_->atPosition("y_position_std_dev", position) +=
          pow(particle.x - pf_statistics_->atPosition("y_position_mean", position), 2);
      // todo: heading calculation
      pf_statistics_->atPosition("velocity_std_dev", position) +=
          pow(particle.x - pf_statistics_->atPosition("velocity_mean", position), 2);
    }
  }

  // Compute standard deviation
  for (grid_map::GridMapIterator iterator(*pf_statistics_); !iterator.isPastEnd(); ++iterator) {
    pf_statistics_->at("x_position_std_dev", *iterator) = sqrt(pf_statistics_->at("x_position_std_dev", *iterator)
                                                          / pf_statistics_->at("particles_per_cell", *iterator));
    pf_statistics_->at("y_position_std_dev", *iterator) = sqrt(pf_statistics_->at("y_position_std_dev", *iterator)
                                                          / pf_statistics_->at("particles_per_cell", *iterator));
    pf_statistics_->at("heading_std_dev", *iterator) = sqrt(2 * (1 - sqrt(
                                                       pow(pf_statistics_->at("heading_sines", *iterator), 2) +
                                                       pow(pf_statistics_->at("heading_cosines", *iterator), 2))));
    pf_statistics_->at("velocity_std_dev", *iterator) = sqrt(pf_statistics_->at("velocity_std_dev", *iterator)
                                                        / pf_statistics_->at("particles_per_cell", *iterator));
  }

  grid_map::Position radar_grid_map_center = map_ptr_->getPosition();
  pf_statistics_->move(radar_grid_map_center);
  std::unique_ptr<grid_map_msgs::msg::GridMap> message;
  message = grid_map::GridMapRosConverter::toMessage(*pf_statistics_);
  pf_statistics_pub_->publish(std::move(message));
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
          "heading", 1, sensor_msgs::msg::PointField::FLOAT32, // semantic but heading should probably be bearing, more intuitive
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

NS_FOOT
