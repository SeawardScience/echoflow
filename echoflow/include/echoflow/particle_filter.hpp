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
 * @brief Struct for holding the properties of a particle (x/y position, course, speed, weight, obs_likelihood, age).
 *
 */
struct Target {
  double x;
  double y;
  double course;   // radians
  double speed;     // m/s
  double yaw_rate;  // rad/s
  double weight;    // per-particle weight
  double obs_likelihood;
  double age;       // Age of the particle in seconds
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
   * Spawns particles with random positions and course angles around radar returns
   * (i.e. anywhere in grid map where radar intensity > 0).
   *
   * @param map_ptr Shared pointer to GridMap with radar intensity-based targets to track.
   */
  void initialize(std::shared_ptr<grid_map::GridMap> map_ptr);

  /**
   * @brief Predict the new (x,y) position and course of each particle.
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
   * @param stats_ptr Shared pointer to GridMap with particle statistics.
   */
  void updateWeights(std::shared_ptr<grid_map::GridMap> map_ptr,
                     std::shared_ptr<grid_map::GridMap> stats_ptr,
                     double dt);

  /**
   * @brief Resample particles using a uniform distribution around current particles.
   *
   * Removes particles with a speed < 3 m/s, unless this removes all existing particles in which case
   * the filter falls back to all particles. Surviving particles are re-sampled using a uniform
   * distribution around each particle.
   *
   * @param map_ptr Shared pointer to GridMap with radar intensity-based targets to track.
   * @param stats_ptr Shared pointer to GridMap with particle statistics.
   */
  void resample(std::shared_ptr<grid_map::GridMap> map_ptr,
                std::shared_ptr<grid_map::GridMap> stats_ptr,
                double dt);

  /**
   * @brief Get valid positions from the grid map.
   *
   * Returns a vector of positions where the grid map has valid data (i.e., where radar intensity > 0).
   * This is used for seeding new particles.
   *
   * @param map_ptr Shared pointer to GridMap with radar intensity-based targets to track.
   * @return std::vector<grid_map::Position> Vector of valid positions.
   */
  std::vector<grid_map::Position> getValidPositionsFromMap(const std::shared_ptr<grid_map::GridMap>& map_ptr);

  /**
   * @brief Seeds particles uniformly from a list of valid positions in the grid map.
   *
   * @param valid_positions List of valid positions from the grid map where particles can be seeded.
   * @param n_seed Number of particles to seed.
   * @param output_particles Vector to store the seeded particles.
   */
  void seedUniform(const std::vector<grid_map::Position>& valid_positions,
                   size_t n_seed,
                   std::vector<Target>& output_particles);

  /**
   * @brief Seed particles weighted by grid map particle density.
   *
   * Preferentially seeds particles in areas of lower density using an inverse weight function for particle density.
   *
   * @param valid_positions List of valid positions from the grid map where particles can be seeded.
   * @param n_seed Number of particles to seed.
   * @param stats_ptr Shared pointer to the grid map containing statistics for particle density.
   * @param output_particles Vector to store the seeded particles.
   */
  void seedWeighted(const std::vector<grid_map::Position>& valid_positions,
                    size_t n_seed,
                    const std::shared_ptr<grid_map::GridMap>& stats_ptr,
                    std::vector<Target>& output_particles);

  /**
   * @brief Update the noise distributions for particle motion used in the predict step.
   */
  void updateNoiseDistributions();

  /**
   * @brief Get particles.
   *
   * @return const std::vector<Target>& Vector of particles on target.
   */
  const std::vector<Target> & getParticles();

  double observation_sigma_;  // Standard deviation for Gaussian weight function
  double weight_decay_half_life_;
  double seed_fraction_;      // Fraction of particles to be seeded with random positions
  double noise_std_pos_;      // Standard deviation for position noise
  double noise_std_yaw_;      // Standard deviation for yaw noise
  double noise_std_yaw_rate_; // Standard deviation for yaw rate noise
  double noise_std_speed_;    // Standard deviation for speed noise
  double density_feedback_factor_;   // the density (particles/m^2) at which the weight of a particle will be reduced by half

  void addResampleNoise(Target &p);

private:
  std::vector<Target> particles_;
  std::default_random_engine rng_;

  size_t num_particles_;      // Number of particles in the filter
  double initial_max_speed_;  // Initial maximum speed of particles

  std::normal_distribution<double> noise_pos_{0.0, noise_std_pos_};        // Gaussian noise distribution for particle position.
  std::normal_distribution<double> noise_yaw_{0.0, noise_std_yaw_};        // Gaussian noise distribution for particle course.
  std::normal_distribution<double> noise_yaw_rate_{0.0, noise_std_yaw_rate_};   // Gaussian noise distribution for course change rate.
  std::normal_distribution<double> noise_speed_{0.0, noise_std_speed_};      // Gaussian noise distribution for particle speed.
};

NS_FOOT
