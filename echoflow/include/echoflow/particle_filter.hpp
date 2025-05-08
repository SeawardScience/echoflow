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
 * @brief TODO
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
 * @brief todo
 *
 */
class MultiTargetParticleFilter {
public:
  /**
   * @brief Construct a new Multi Target Particle Filter object
   *
   * @param num_particles Number of particles to use to initialize the particle filter (default: 500 particles).
   */
  explicit MultiTargetParticleFilter(size_t num_particles = 500);

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
   * @brief Update particle weights TODO
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
  void resample();

  /**
   * @brief Get particles.
   *
   * @return const std::vector<Target>& Vector of particles on target. // todo
   */
  const std::vector<Target> & getParticles();

private:
  std::vector<Target> particles_;
  std::default_random_engine rng_;

  std::normal_distribution<double> noise_pos_{0.0, 0.1};        // Gaussian noise distribution for particle position.
  std::normal_distribution<double> noise_yaw_{0.0, 0.4};        // Gaussian noise distribution for particle heading.
  std::normal_distribution<double> noise_yaw_rate_{0.0, 1.0};   // Gaussian noise distribution for heading change rate.
  std::normal_distribution<double> noise_speed_{0.0, 4.0};      // Gaussian noise distribution for particle speed.
};

NS_FOOT
