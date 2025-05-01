#include "particle_filter.hpp"

NS_HEAD

MultiTargetParticleFilter::MultiTargetParticleFilter(size_t num_particles) {
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

    grid_map::Position pos;
    if (map_ptr->getPosition(index, pos)) {
      valid_positions.push_back(pos);
    }
  }

  if (valid_positions.empty()) {
    RCLCPP_WARN(rclcpp::get_logger("MultiTargetParticleFilter"), "No valid positions with intensity > 0 found.");
    return;
  }

  // Add new particles at randomly selected valid positions
  for (size_t i = 0; i < 10; ++i) {
    const auto& pos = valid_positions[rand() % valid_positions.size()];
    Target p;
    p.x = pos.x();
    p.y = pos.y();
    p.heading = 2.0 * M_PI * ((double)rand() / RAND_MAX);
    p.speed = 20.0 * ((double)rand() / RAND_MAX);
    p.yaw_rate = 0.0 * ((double)rand() / RAND_MAX);
    p.weight = 1.0;  // Will be normalized later
    particles_.push_back(p);
  }
}


void MultiTargetParticleFilter::predict(double dt) {
  for (auto& p : particles_) {
    double v = p.speed + noise_speed_(rng_);
    double yaw = p.heading + noise_yaw_(rng_);
    double omega = p.yaw_rate + noise_yaw_rate_(rng_);

    if (std::abs(omega) > 1e-3) {
      double radius = v / omega;
      p.x += radius * (std::sin(yaw + omega * dt) - std::sin(yaw)) + noise_pos_(rng_);
      p.y += radius * (-std::cos(yaw + omega * dt) + std::cos(yaw)) + noise_pos_(rng_);
    } else {
      p.x += v * std::cos(yaw) * dt + noise_pos_(rng_);
      p.y += v * std::sin(yaw) * dt + noise_pos_(rng_);
    }

    p.heading += omega * dt;
    p.heading = std::fmod(p.heading + 2 * M_PI, 2 * M_PI);
  }
}

void MultiTargetParticleFilter::updateWeights(std::shared_ptr<grid_map::GridMap> map_ptr)
{
  if (!map_ptr || !map_ptr->exists("edt")) {
    RCLCPP_WARN(rclcpp::get_logger("MultiTargetParticleFilter"), "GridMap does not contain 'edt' layer.");
    return;
  }

  double sigma = 30.0;

  const double decay_factor = 0.95;  // Retain 50% of previous weight if outside detection
  double total_weight = 0.0;

  for (auto& p : particles_) {
    grid_map::Position pos(p.x, p.y);
    double new_weight = 0.0;

    if (map_ptr->isInside(pos)) {
      try {
        double dist = map_ptr->atPosition("edt", pos);
        new_weight = std::exp(- (dist * dist) / (2.0 * sigma * sigma));;
      } catch (const std::out_of_range& e) {
        // fall through to decay
      }
    }

    // If no valid reading, retain some of the previous weight
    if (new_weight == 0.0) {
      new_weight = p.weight * decay_factor;
    }

    p.weight = new_weight;
    total_weight += new_weight;
  }

  if (total_weight > 0.0) {
    for (auto& p : particles_) {
      p.weight /= total_weight;
    }
  }
}



void MultiTargetParticleFilter::resample() {
  // Filter out particles with speed < 3 m/s
  std::vector<Target> filtered_particles;
  for (const auto& p : particles_) {
    if (p.speed >= 3.0) {
      filtered_particles.push_back(p);
    }
  }

  // If no particles survive the filter, fall back to all particles to avoid failure
  const auto& source_particles = filtered_particles.empty() ? particles_ : filtered_particles;

  std::vector<Target> new_particles;
  new_particles.reserve(source_particles.size());

  std::uniform_real_distribution<double> dist_u(0.0, 1.0);
  double step = 1.0 / source_particles.size();
  double r = dist_u(rng_) * step;
  double c = source_particles[0].weight;
  size_t i = 0;

  for (size_t m = 0; m < source_particles.size(); ++m) {
    double U = r + m * step;
    while (U > c && i < source_particles.size() - 1) {
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
