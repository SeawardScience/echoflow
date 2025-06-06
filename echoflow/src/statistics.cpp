#include "statistics.hpp"

NS_HEAD

namespace statistics {

float computeSequentialMean(float new_observation, float num_samples, float prior_mean)
{
  if (num_samples < 1) {
    throw std::invalid_argument("Invalid number of particles.");
  }
  return prior_mean + ((new_observation - prior_mean) / num_samples);
}

std::tuple<float, float> computeSequentialVariance(float new_observation,
                                                   float num_samples,
                                                   float prior_mean,
                                                   float new_mean,
                                                   float prior_ssdm)
{
  float new_ssdm = prior_ssdm + (new_observation - prior_mean) * (new_observation - new_mean);

  // Variance is only defined for n > 1 due to division by n-1
  if (num_samples < 2) {
    return { NAN, new_ssdm };
  }

  return { new_ssdm / (num_samples - 1), new_ssdm };
}

std::tuple<float, float> computeSequentialStdDev(float new_observation,
                                                 float num_samples,
                                                 float prior_mean,
                                                 float new_mean,
                                                 float prior_ssdm)
{
  auto [variance, ssdm] = computeSequentialVariance(new_observation, num_samples, prior_mean, new_mean, prior_ssdm);
  return { sqrt(variance), ssdm };
}

float computeCircularMean(float sines_sum, float cosines_sum)
{
  return atan2(sines_sum, cosines_sum);
}

float computeCircularVariance(float sines_sum, float cosines_sum, float num_samples)
{
  return 1 - computeMeanResultantLength(sines_sum, cosines_sum, num_samples);
}

float computeCircularStdDev(float sines_sum, float cosines_sum, float num_samples)
{
  return sqrt(-2.0 * log(computeMeanResultantLength(sines_sum, cosines_sum, num_samples)));
}

float computeMeanResultantLength(float sines_sum, float cosines_sum, float num_samples)
{
  if (num_samples < 1) {
    throw std::invalid_argument("Invalid number of particles.");
  }
  float sines_mean = sines_sum / num_samples;
  float cosines_mean = cosines_sum / num_samples;
  return sqrt(pow(sines_mean, 2) + pow(cosines_mean, 2));
}

}  // namespace statistics

NS_FOOT
