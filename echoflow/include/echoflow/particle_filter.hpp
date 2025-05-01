#pragma once

#include "package_defs.hpp"
#include <vector>
#include <random>
#include <cmath>
#include <numeric>

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

  void initialize(const std::vector<Detection>& detections);
  void predict(double dt);
  void updateWeights(const std::vector<Detection>& detections);
  void resample();
  const std::vector<Target> & getParticles();

private:
  std::vector<Target> particles_;
  std::default_random_engine rng_;

  std::normal_distribution<double> noise_pos_{0.0, 0.1};
  std::normal_distribution<double> noise_yaw_{0.0, 0.5};
  std::normal_distribution<double> noise_speed_{0.0, 1.0};
};


NS_FOOT
