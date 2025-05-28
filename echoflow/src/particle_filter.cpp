#include "particle_filter.hpp"

NS_HEAD

MultiTargetParticleFilter::MultiTargetParticleFilter(size_t num_particles,
                                                       double initial_max_speed)
  : initial_max_speed_(initial_max_speed)
{
  particles_.resize(num_particles);
  rng_.seed(std::random_device{}());
}

void MultiTargetParticleFilter::initialize(std::shared_ptr<grid_map::GridMap> map_ptr)
{
  if (!map_ptr || !map_ptr->exists("intensity")) {
    RCLCPP_WARN(rclcpp::get_logger("MultiTargetParticleFilter"), "GridMap missing or lacks 'intensity' layer.");
    return;
  }

  // Gather valid positions from the map
  std::vector<grid_map::Position> valid_positions;
  for (grid_map::GridMapIterator it(*map_ptr); !it.isPastEnd(); ++it) {
    const auto& index = *it;
    if (!map_ptr->isValid(index, "intensity")) continue;

    double val = map_ptr->at("intensity", index);
    if (std::isnan(val) || val <= 0.0) continue;

    grid_map::Position position;
    if (map_ptr->getPosition(index, position)) {
      valid_positions.push_back(position);
    }
  }

  if (valid_positions.empty()) {
    RCLCPP_WARN(rclcpp::get_logger("MultiTargetParticleFilter"), "No valid positions with intensity > 0 found.");
    return;
  }

  // Add new particles at randomly selected valid positions
  // TODO: revisit this section for tracking different sizes of blobs or "ignoring" static blobs
  for (size_t i = 0; i < 10; ++i) {
    const auto& position = valid_positions[rand() % valid_positions.size()];
    Target particle;
    particle.x = position.x();
    particle.y = position.y();
    particle.speed = initial_max_speed_ * static_cast<double>(rand()) / RAND_MAX;
    particle.heading = 2.0 * M_PI * static_cast<double>(rand()) / RAND_MAX;
    particle.yaw_rate = 0.0 * static_cast<double>(rand()) / RAND_MAX; // This is ZERO always...
    particle.weight = 1.0;  // Will be normalized later
    particles_.push_back(particle);
  }
}

void MultiTargetParticleFilter::predict(double dt)
{
  for (auto& particle : particles_) {
    double velocity = particle.speed + noise_speed_(rng_);
    double yaw = particle.heading + noise_yaw_(rng_);
    double omega = particle.yaw_rate + noise_yaw_rate_(rng_);

    if (std::abs(omega) > 1e-3) {
      double radius = velocity / omega;
      particle.x += radius * (std::sin(yaw + omega * dt) - std::sin(yaw)) + noise_pos_(rng_);
      particle.y += radius * (-std::cos(yaw + omega * dt) + std::cos(yaw)) + noise_pos_(rng_);
    } else {
      particle.x += velocity * std::cos(yaw) * dt + noise_pos_(rng_);
      particle.y += velocity * std::sin(yaw) * dt + noise_pos_(rng_);
    }

    particle.heading += omega * dt;
    particle.heading = std::fmod(particle.heading + 2 * M_PI, 2 * M_PI);
  }
}

void MultiTargetParticleFilter::updateWeights(std::shared_ptr<grid_map::GridMap> map_ptr)
{
  if (!map_ptr || !map_ptr->exists("edt")) {
    RCLCPP_WARN(rclcpp::get_logger("MultiTargetParticleFilter"), "GridMap does not contain 'edt' layer."); // maybe change to throttle
    return;
  }

  double sigma = observation_sigma_;
  const double decay_factor = decay_factor_;
  double total_weight = 0.0;

  for (auto& particle : particles_) {
    grid_map::Position position(particle.x, particle.y);
    double new_weight = 0.0;

    // Check if the particle is inside the map
    if (map_ptr->isInside(position)) {
      try {
        double distance = map_ptr->atPosition("edt", position);
        new_weight = std::exp(- (distance * distance) / (2.0 * sigma * sigma));;
      } catch (const std::out_of_range& e) {
        // fall through to decay
      }
    }

    // If no valid reading, retain some of the previous weight
    if (new_weight == 0.0) {
      new_weight = particle.weight * decay_factor;
    }

    particle.weight = new_weight;
    total_weight += new_weight;
  }

  // Normalize weights
  if (total_weight > 0.0) {
    for (auto& particle : particles_) {
      particle.weight /= total_weight;
    }
  }
}

void MultiTargetParticleFilter::resample()
{
  // Filter out particles with speed > min_resample_speed
  std::vector<Target> filtered_particles;
  for (const auto& particle : particles_) {   // debugging code to limit velocity TODO: set min resample speed threshold to a param
    if (particle.speed >= min_resample_speed_) {
      filtered_particles.push_back(particle);
    }
  }

  // If no particles survive the filter, fall back to all particles to avoid failure
  const auto& source_particles = filtered_particles.empty() ? particles_ : filtered_particles;

  std::vector<Target> new_particles;
  new_particles.reserve(source_particles.size());

  std::uniform_real_distribution<double> dist_u(0.0, 1.0); // uniform distribution
  double step = 1.0 / source_particles.size();
  double r = dist_u(rng_) * step; // initial offset
  double c = source_particles[0].weight; // cumulative weight
  size_t i = 0; // source index

  // TODO: revisit variable names
  for (size_t m = 0; m < source_particles.size(); ++m) // m resample index
  {
    double U = r + m * step; // uniform sample point along [0,1] range used to pick a particle based on weights
    while (U > c && i < source_particles.size() - 1)
    {
      ++i;
      c += source_particles[i].weight;
    }
    new_particles.push_back(source_particles[i]);
    new_particles.back().weight = 1.0 / source_particles.size();
  }

  particles_ = std::move(new_particles);
}

const std::vector<Target>& MultiTargetParticleFilter::getParticles()
{
  return particles_;
}

NS_FOOT
