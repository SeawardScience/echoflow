#pragma once

#include <cmath>
#include <numeric>
#include <random>
#include <vector>
#include <grid_map_ros/grid_map_ros.hpp>
#include "package_defs.hpp"
#include "radar_grid_map.hpp"


NS_HEAD

struct Target {
  double x;
  double y;
  double heading;   // radians
  double speed;     // m/s
  double yaw_rate;  // rad/s
  double weight;    // per-particle weight
};

struct Detection {
  double x;
  double y;
};

class MultiTargetParticleFilter {
public:
  MultiTargetParticleFilter(size_t num_particles = 500);

  // TODO: multi-threading changes
  //void initialize(std::shared_ptr<grid_map::GridMap> map_ptr);
  void initialize(std::shared_ptr<ThreadsafeGridMap> map_ptr);
  void predict(double dt);
  //void updateWeights(std::shared_ptr<grid_map::GridMap> map_ptr);
  void updateWeights(std::shared_ptr<ThreadsafeGridMap> map_ptr);
  void resample();
  const std::vector<Target> & getParticles();

private:
  std::vector<Target> particles_;
  std::default_random_engine rng_;

  std::normal_distribution<double> noise_pos_{0.0, 0.1};
  std::normal_distribution<double> noise_yaw_{0.0, 0.4};
  std::normal_distribution<double> noise_yaw_rate_{0.0, 1.0};
  std::normal_distribution<double> noise_speed_{0.0, 4.0};
};


NS_FOOT
