#include "particle_filter.hpp"

NS_HEAD

MultiTargetParticleFilter::MultiTargetParticleFilter(size_t num_particles) {
  particles_.resize(num_particles);
  rng_.seed(std::random_device{}());
}

void MultiTargetParticleFilter::initialize(const std::vector<Detection>& detections) {
  size_t n = particles_.size();
  for (size_t i = 0; i < n; ++i) {
    const auto& d = detections[i % detections.size()];
    particles_[i].x = d.x + noise_pos_(rng_);
    particles_[i].y = d.y + noise_pos_(rng_);
    particles_[i].heading = 2 * M_PI * ((double)rand() / RAND_MAX);
    particles_[i].speed = 20 * ((double)rand() / RAND_MAX);
    particles_[i].yaw_rate = 1.0 * ((double)rand() / RAND_MAX);;
    particles_[i].weight = 1.0 / n;
  }
}

void MultiTargetParticleFilter::predict(double dt) {
  for (auto& p : particles_) {
    double v = p.speed + noise_speed_(rng_);
    double yaw = p.heading;
    double omega = p.yaw_rate + noise_yaw_(rng_);

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

void MultiTargetParticleFilter::updateWeights(const std::vector<Detection>& detections) {
  if(detections.size()==0){
    return;
  }
  const double sigma = 10.0;
  const double denom = 2.0 * sigma * sigma;
  double total_weight = 0.0;

  for (auto& p : particles_) {
    double best_score = 0.0;
    for (const auto& d : detections) {
      double dx = d.x - p.x;
      double dy = d.y - p.y;
      double dist_sq = dx * dx + dy * dy;
      double score = std::exp(-dist_sq / denom);
      if (score > best_score)
        best_score = score;
    }
    p.weight = best_score;
    total_weight += p.weight;
  }

  if (total_weight > 0.0) {
    for (auto& p : particles_) {
      p.weight /= total_weight;
    }
  }
}

void MultiTargetParticleFilter::resample() {
  std::vector<Target> new_particles;
  new_particles.reserve(particles_.size());

  std::uniform_real_distribution<double> dist_u(0.0, 1.0);
  double step = 1.0 / particles_.size();
  double r = dist_u(rng_) * step;
  double c = particles_[0].weight;
  size_t i = 0;

  for (size_t m = 0; m < particles_.size(); ++m) {
    double U = r + m * step;
    while (U > c && i < particles_.size() - 1) {
      i++;
      c += particles_[i].weight;
    }
    new_particles.push_back(particles_[i]);
    new_particles.back().weight = 1.0 / particles_.size();
  }

  particles_ = std::move(new_particles);
}

const std::vector<Target>& MultiTargetParticleFilter::getParticles()
{
  return particles_;
}

NS_FOOT
