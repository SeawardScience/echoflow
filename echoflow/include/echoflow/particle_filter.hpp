/** Copyright © 2025 Seaward Science. */

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
    double x;            ///< X position of the particle (meters)
    double y;            ///< Y position of the particle (meters)
    double course;       ///< Course of the particle (radians, CCW from +X axis)
    double speed;        ///< Linear speed of the particle (meters per second)
    double yaw_rate;     ///< Yaw rate of the particle (radians per second)
    double weight;       ///< Importance weight of the particle (normalized for resampling)
    double obs_likelihood;///< Raw observation likelihood before weight normalization
    double age;          ///< Age of the particle since last initialization or reseeding (seconds)
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
   * @brief Update particle weights using the Euclidean distance transform, exponential decay, and density feedback.
   *
   * For each particle:
   *   - Computes a new observation likelihood from the "edt" layer of `map_ptr`:
   *     If outside the map or missing layer, a small baseline of 1e-6 is used.
   *   - Updates `particle.obs_likelihood`:
   *     - If the new likelihood exceeds the previous, it is replaced.
   *     - Otherwise, the previous likelihood is multiplied by the decay factor
   *   - Sets `particle.weight = std::max(particle.obs_likelihood, 1e-8)`.
   *   - Retrieves local density from the "particles_per_cell" layer and applies
   *     a logistic penalty multiplying `particle.weight` by this factor.
   * After processing all particles, normalizes their weights so that their sum equals 1.
   *
   * @param map_ptr Shared pointer to GridMap with radar intensity-based targets to track.
   * @param stats_ptr Shared pointer to GridMap with particle statistics.
   * @param dt Time interval (delta t) from last particle filter update step.
   */
  void updateWeights(std::shared_ptr<grid_map::GridMap> map_ptr,
                     std::shared_ptr<grid_map::GridMap> stats_ptr,
                     double dt);

  /**
   * @brief Add Gaussian noise to a particle's position, yaw, yaw rate, and speed.
   *
   * This function modifies the particle in place by adding Gaussian noise to its position,
   * yaw angle, yaw rate, and speed based on the defined noise distributions.
   *
   * @param particle Reference to the particle to which noise will be added.
   */
  void addResampleNoise(Target& particle);

  /**
   * @brief Perform systematic resampling of existing particles and inject new random particles.
   *
   * This method replaces the current particle set by:
   *   - **Systematic resampling** of \(N - M\) particles, drawn proportional to their weights,
   *     adding Gaussian noise to position, heading and speed (via addResampleNoise), preserving
   *     each particle’s age, and resetting its weight to \(1/N\).
   *   - **Injection** of \(M = \lfloor \text{seed_fraction\_} \times dt \times N\rfloor\) new particles
   *     at valid positions from `map_ptr`. Positions are chosen with probability
   *     proportional to the inverse local density (from the “particles_per_cell”
   *     layer of `stats_ptr`) to encourage seeding in less crowded areas i.e. new radar targets.
   *   - New particles receive:
   *     - random speed ∈ [0, initial_max_speed_]
   *     - random course ∈ [0, 2π)
   *     - zero yaw_rate
   *     - weight = \(1/N\)
   *     - age = 0
   *
   * @param map_ptr Shared pointer to GridMap with radar intensity-based targets to track.
   * @param stats_ptr Shared pointer to GridMap with particle statistics.
   * @param dt Time interval (delta t) from last particle filter update step.
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

  double observation_sigma_;        ///< Standard deviation for Gaussian weight function
  double weight_decay_half_life_;   ///< Half-life for exponential decay of particle weights
  double seed_fraction_;            ///< Fraction of particles to be seeded with random positions
  double noise_std_position_;       ///< Standard deviation for position noise
  double noise_std_yaw_;            ///< Standard deviation for yaw noise
  double noise_std_yaw_rate_;       ///< Standard deviation for yaw rate noise
  double noise_std_speed_;          ///< Standard deviation for speed noise
  double density_feedback_factor_;  ///< Density (particles/m^2) at which the weight of a particle will be reduced by half

private:
  std::vector<Target> particles_; ///< Vector of particles in the filter
  std::default_random_engine rng_; ///< Random number generator for particle noise and sampling

  size_t num_particles_;      ///< Number of particles in the filter
  double initial_max_speed_;  ///< Initial maximum speed of particles

  // Gaussian noise distributions for particle position, yaw/course angle, yaw change rate, and speed
  std::normal_distribution<double> noise_position_{0.0, noise_std_position_};
  std::normal_distribution<double> noise_yaw_{0.0, noise_std_yaw_};
  std::normal_distribution<double> noise_yaw_rate_{0.0, noise_std_yaw_rate_};
  std::normal_distribution<double> noise_speed_{0.0, noise_std_speed_};
};

NS_FOOT
