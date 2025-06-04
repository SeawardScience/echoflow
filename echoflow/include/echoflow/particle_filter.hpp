#pragma once

#include <cmath>
#include <memory>
#include <numeric>
#include <random>
#include <vector>
#include <grid_map_ros/grid_map_ros.hpp>
#include "package_defs.hpp"


NS_HEAD

/**
 * @brief Struct for holding the properties of a particle (position, heading, speed, weight).
 *
 */
struct Target {
  double x;
  double y;
  double heading;   // radians
  double speed;     // m/s
  double yaw_rate;  // rad/s
  double weight;    // per-particle weight
};

/**
 * @brief todo
 *
 */
struct Detection {
  double x;
  double y;
};

/**
 * @brief Implements a particle filter for tracking multiple targets.
 */
class MultiTargetParticleFilter {
public:
  /**
   * @brief Construct a new Multi Target Particle Filter object
   *
   * @param num_particles Number of particles to use to initialize the particle filter (default: 500 particles).
   * @param initial_max_speed Initial maximum speed of particles (default: 20.0 m/s).
   */
  explicit MultiTargetParticleFilter(size_t num_particles = 500,
                                     double initial_max_speed = 20.0);
  /**
   * @brief Initialize multi-target particle filter.
   *
   * Spawns particles with random positions and headings around radar returns (i.e. anywhere in grid map
   * where radar intensity > 0).
   *
   * @param map_ptr Shared pointer to GridMap with radar intensity-based targets to track.
   */
  void initialize(std::shared_ptr<grid_map::GridMap> map_ptr);

  /**
   * @brief Predict the new (x,y) position and heading of each particle.
   *
   * @param dt Time interval (delta t) from last particle filter update step.
   */
  void predict(double dt);

  /**
   * @brief Update particle weights using the Euclidean distance transform for each particle.
   *
   * Each particle is re-weighted from 0 to 1 according to a Gaussian function of their
   * Euclidean distance transform. If the particle re-weight results in a weight of 0, the weight is
   * reduced by a given decay factor. Individual particle weights are then normalized by the total
   * weight of all particles.
   *
   * @param map_ptr Shared pointer to GridMap with radar intensity-based targets to track.
   */
  void updateWeights(std::shared_ptr<grid_map::GridMap> map_ptr);

  /**
   * @brief Resample particles using a uniform distribution around current particles.
   *
   * Removes particles with a speed < 3 m/s, unless this removes all existing particles in which case
   * the filter falls back to all particles. Surviving particles are re-sampled using a uniform
   * distribution around each particle.
   *
   */
  void resample(std::shared_ptr<grid_map::GridMap> map_ptr);

  /**
   * @brief updateNoiseDistributions
   */
  void updateNoiseDistributions();

  /**
   * @brief Get particles.
   *
   * @return const std::vector<Target>& Vector of particles on target.
   */
  const std::vector<Target> & getParticles();

  double observation_sigma_;  // Standard deviation for Gaussian weight function
  double decay_factor_;       // Decay factor for particle weight
  double min_resample_speed_; // Minimum speed for resampling particles
  double noise_std_pos_;      // Standard deviation for position noise
  double noise_std_yaw_;      // Standard deviation for yaw noise
  double noise_std_yaw_rate_; // Standard deviation for yaw rate noise
  double noise_std_speed_;    // Standard deviation for speed noise

private:
  std::vector<Target> particles_;
  std::default_random_engine rng_;

  size_t num_particles_;      // Number of particles in the filter
  double initial_max_speed_;  // Initial maximum speed of particles

  std::normal_distribution<double> noise_pos_{0.0, noise_std_pos_};        // Gaussian noise distribution for particle position.
  std::normal_distribution<double> noise_yaw_{0.0, noise_std_yaw_};        // Gaussian noise distribution for particle heading.
  std::normal_distribution<double> noise_yaw_rate_{0.0, noise_std_yaw_rate_};   // Gaussian noise distribution for heading change rate.
  std::normal_distribution<double> noise_speed_{0.0, noise_std_speed_};      // Gaussian noise distribution for particle speed.
};

NS_FOOT
