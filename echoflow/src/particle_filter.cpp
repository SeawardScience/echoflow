#include "particle_filter.hpp"

NS_HEAD

MultiTargetParticleFilter::MultiTargetParticleFilter(size_t num_particles,
                                                     double initial_max_speed)
  : num_particles_(num_particles), initial_max_speed_(initial_max_speed)
{
  particles_.resize(num_particles_);
  rng_.seed(std::random_device{}());
}

void MultiTargetParticleFilter::initialize(std::shared_ptr<grid_map::GridMap> map_ptr)
{
  RCLCPP_DEBUG(rclcpp::get_logger("MultiTargetParticleFilter"),
                "ParticleFilter Config: num_particles=%zu, observation_sigma=%.2f, decay=%.2f, "
                "min_speed=%.2f, noise_std_pos=%.2f, noise_std_yaw=%.2f, noise_std_yaw_rate=%.2f, noise_std_speed=%.2f",
                num_particles_, observation_sigma_, decay_factor_,
                min_resample_speed_, noise_std_pos_, noise_std_yaw_, noise_std_yaw_rate_, noise_std_speed_);

  if (!map_ptr || !map_ptr->exists("intensity")) {
    RCLCPP_WARN(rclcpp::get_logger("MultiTargetParticleFilter"), "GridMap missing or lacks 'intensity' layer.");
    return;
  }

  // Gather valid positions from the map
  std::vector<grid_map::Position> valid_positions = getValidPositionsFromMap(map_ptr);

  if (valid_positions.empty()) {
    RCLCPP_WARN(rclcpp::get_logger("MultiTargetParticleFilter"), "No valid positions with intensity > 0 found.");
    return;
  }

  // Add new particles at randomly selected valid positions
  // TODO: revisit this section for tracking different sizes of blobs or "ignoring" static blobs
  for (size_t i = 0; i < num_particles_; ++i) {
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

  RCLCPP_INFO(rclcpp::get_logger("MultiTargetParticleFilter"),
              "Initialized with %zu particles.", particles_.size());
}

void MultiTargetParticleFilter::predict(double dt)
{
  RCLCPP_DEBUG(rclcpp::get_logger("MultiTargetParticleFilter"), "Predicting next state for %zu particles with dt = %.3f", particles_.size(), dt);

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

void MultiTargetParticleFilter::resample(std::shared_ptr<grid_map::GridMap> map_ptr)
{
    const size_t n_total = num_particles_;
    const size_t n_seed = static_cast<size_t>(0.001 * n_total); // 0.1% of total particles are seeded
    const size_t n_resample = n_total - n_seed;

    std::vector<Target> new_particles;
    new_particles.reserve(n_total);

    // Step 1: Resample n_resample particles
    std::uniform_real_distribution<double> dist_u(0.0, 1.0);
    double step = 1.0 / static_cast<double>(n_resample);
    double r = dist_u(rng_) * step;
    double c = particles_[0].weight;
    size_t i = 0;

    for (size_t m = 0; m < n_resample; ++m) {
        double U = r + m * step;
        while (U > c && i < particles_.size() - 1) {
            ++i;
            c += particles_[i].weight;
        }
        Target p = particles_[i];
        p.weight = 1.0 / n_total;  // Normalize to total
        new_particles.push_back(p);
    }

    // Step 2: Inject n_seed randomly initialized particles
    std::vector<grid_map::Position> valid_positions = getValidPositionsFromMap(map_ptr);
    std::uniform_real_distribution<double> uniform_01(0.0, 1.0);

    for (size_t m = 0; m < n_seed && !valid_positions.empty(); ++m) {
        const auto& pos = valid_positions[rng_() % valid_positions.size()];
        Target particle;
        particle.x = pos.x();
        particle.y = pos.y();
        particle.speed = initial_max_speed_ * uniform_01(rng_);
        particle.heading = 2.0 * M_PI * uniform_01(rng_);
        particle.yaw_rate = 0.0;
        particle.weight = 1.0 / n_total;
        new_particles.push_back(particle);
    }

    particles_ = std::move(new_particles);

    RCLCPP_DEBUG(rclcpp::get_logger("MultiTargetParticleFilter"),
                 "%zu particles: %zu resampled, %zu seeded.",
                 particles_.size(), n_resample, n_seed);
}

std::vector<grid_map::Position> MultiTargetParticleFilter::getValidPositionsFromMap(const std::shared_ptr<grid_map::GridMap>& map_ptr)
{
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

    return valid_positions;
}


void MultiTargetParticleFilter::updateNoiseDistributions() {
    noise_pos_       = std::normal_distribution<double>(0.0, noise_std_pos_);
    noise_yaw_       = std::normal_distribution<double>(0.0, noise_std_yaw_);
    noise_yaw_rate_  = std::normal_distribution<double>(0.0, noise_std_yaw_rate_);
    noise_speed_     = std::normal_distribution<double>(0.0, noise_std_speed_);
}

const std::vector<Target>& MultiTargetParticleFilter::getParticles()
{
  return particles_;
}

NS_FOOT
