#pragma once

#include <cmath>
#include <complex>
#include <limits>
#include <stdexcept>
#include <string>
#include <tuple>
#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_cv/grid_map_cv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include "package_defs.hpp"

using namespace std::complex_literals;

NS_HEAD

/**
 * @brief Compute the Euclidean distance transform of the radar intensity layer.
 *
 * Uses the built-in grid_map function to convert the intensity layer of the grid map to an OpenCV image.
 * Next uses the OpenCV function distanceTransform to compute the L2 (Euclidean) distance of the radar
 * intensity image, then adds distance image as another layer in the grid map.
 *
 * @param map Grid map to modify.
 * @param intensity_layer Name of intensity layer in grid map.
 * @param distance_layer Name of distance layer in grid map.
 */
void computeEDTFromIntensity(grid_map::GridMap& map,
                             const std::string& intensity_layer,
                             const std::string& distance_layer);

void filterLargeBlobsFromLayer(grid_map::GridMap& map,
                               const std::string& input_layer,
                               const std::string& output_layer,
                               double max_blob_area);

/**
 * @brief Compute the arithmetic mean of a sample given a new observation.
 *
 * Given a new observation \f$x_n\f$, the prior mean of the data \f$\overline{x}_{n-1}\f$,
 * and the total number of observations \f$n\f$, the recurrence relation for computing
 * the sequential mean \f$\overline{x}_n\f$ is as follows:
 *
 * \f$ \overline{x}_n = \overline{x}_{n-1} + \frac{x_n - \overline{x}_{n-1}}{n} \f$
 *
 * @param new_observation New value with which to update the mean.
 * @param num_samples Total number of samples (including current new observation).
 * @param prior_mean Prior mean of the sample data (without new observation).
 * @return Arithmetic mean of the sample.
 */
float computeSequentialMean(float new_observation, float num_samples, float prior_mean);

/**
 * @brief Compute the variance of a sample given a new observation.
 *
 * This function uses Welford's algorithm @cite welford_1962 to compute the new variance of the sample given
 * the new observation. The following recurrence relation computes the unbiased variance of the sample for
 * \f$ n > 1\f$:
 *
 * \f$ s^2_n = s^2_{n-1} + \frac{(x_n - \overline{x}_{n-1})^2}{n} - \frac{s^2_{n-1}}{n-1}\f$
 *
 * Directly using this formula can be numerically unstable, so following Welford's algorithm the
 * sum of squares of deviation from current mean, \f$ M_{2,n} = \sum_\limits{i=1}^{n} (x_i - \overline{x}_n)^2 \f$
 * is used to update the variance:
 *
 * \f$ M_{2,n} = M_{2,n-1} + (x_n - \overline{x}_{n-1})(x_n - \overline{x}_n) \f$
 *
 * The variance returned is: \f$ s^2_n = \frac{M_{2,n}}{n} \f$
 *
 * The function returns both the variance and the updated sum of squares of deviation from the mean
 * \f$ M_{2,n} \f$.
 *
 * @param new_observation New value with which to update the variance.
 * @param num_samples Total number of samples (including current new observation).
 * @param prior_mean Prior mean of the sample data (without new observation).
 * @param new_mean New mean of sample including current observation (computeSequentialMean() should be used to
 *                 compute the mean of the sample with the current observation before computing the variance).
 * @param prior_ssdm Prior sum of squared deviations from the mean (without new observation).
 * @return Variance of sample with new observation, new sum of squared deviations from the mean.
 */
std::tuple<float, float> computeSequentialVariance(float new_observation,
                                                   float num_samples,
                                                   float prior_mean,
                                                   float new_mean,
                                                   float prior_ssdm);

/**
 * @brief Compute the standard deviation of a sample given a new observation.
 *
 * Computes the square root of the variance. See the documentation for the \ref computeSequentialVariance()
 * function for details on how the variance is computed.
 *
 * Returns both standard deviation and the updated sum of squares of deviation from the mean \f$ M_{2,n} \f$.
 *
 * @param new_observation New value with which to update the standard deviation.
 * @param num_samples Total number of samples (including current new observation).
 * @param prior_mean Prior mean of the sample data (without new observation).
 * @param new_mean New mean of sample including current observation (computeSequentialMean() should be used to
 *                 compute the mean of the sample with the current observation before computing the standard deviation).
 * @param prior_ssdm Prior sum of squared deviations from the mean (without new observation) used to compute variance.
 * @return Standard deviation of sample with new observation, new sum of squared deviations from the mean.
 */
std::tuple<float, float> computeSequentialStdDev(float new_observation,
                                                 float num_samples,
                                                 float prior_mean,
                                                 float new_mean,
                                                 float prior_ssdm);

/**
 * @brief Compute the circular mean angle of a sample of angle data.
 *
 * Given \f$n\f$ angles \f$\alpha_1, ..., \alpha_n\f$ measured in radians, their circular mean is defined as
 * (@cite Mardia_1972, section 2.2.2):
 *
 * \f$\overline{\alpha} = \textrm{arg}\biggl(\sum_\limits{j=1}^{n} e^{i\cdot\alpha_j} \biggr)\f$
 *
 * In order to store the Cartesian coordinates of each heading as a real float without an imaginary component,
 * we instead use the arctan2 formulation to compute the mean resultant angle back to polar coordinates for
 * to obtain the mean heading, as follows:
 *
 * \f$\overline{\alpha} = \textrm{atan2}\biggl(\sum\limits_{j=1}^{n} \sin \alpha_j,
 *                                             \sum_\limits{j=1}^{n} \cos \alpha_j \biggr)\f$
 * @param sines_sum Sum of the sines of a sample of angle data.
 * @param cosines_sum Sum of the cosines of a sample of angle data.
 * @return Circular mean of the sample.
 */
float computeCircularMean(float sines_sum, float cosines_sum);

/**
 * @brief Compute the circular variance of a sample of angle data.
 *
 * Given \f$n\f$ angles \f$\alpha_1, ..., \alpha_n\f$ measured in radians, their circular variance \f$S_0\f$ is
 * defined as (@cite Mardia_1972, section 2.3):
 *
 * \f$ S_0 = 1 - \overline{R} \f$
 *
 * where \f$ \overline{R} \f$ is the mean resultant length of the data, as defined in
 * \ref computeMeanResultantLength().
 *
 * @param sines_sum Sum of the sines of a sample of angle data.
 * @param cosines_sum Sum of the cosines of a sample of angle data.
 * @param num_samples Total number of samples in the set of angle data.
 * @return Circular variance of the sample.
 */
float computeCircularVariance(float sines_sum, float cosines_sum, float num_samples);

/**
 * @brief Compute the circular standard deviation of a sample of angle data.
 *
 * Given \f$n\f$ angles \f$\alpha_1, ..., \alpha_n\f$ measured in radians, their circular standard deviation
 * is defined as (@cite Mardia_1972, section 2.3.4, Eq. 2.3.14):
 *
 * \f$ s_0 = \sqrt{-2.0 * \log(\overline{R})} \f$,
 *
 * where \f$ \overline{R} \f$ is the mean resultant length of the data, as defined in
 * \ref computeMeanResultantLength().
 *
 * @param sines_sum Sum of the sines of a sample of angle data.
 * @param cosines_sum Sum of the cosines of a sample of angle data.
 * @param num_samples Total number of samples in the set of angle data.
 * @return Circular standard deviation of the sample.
 */
float computeCircularStdDev(float sines_sum, float cosines_sum, float num_samples);

/**
 * @brief Compute the mean resultant length of a sample of angle data.
 *
 * Given \f$n\f$ angles \f$\alpha_1, ..., \alpha_n\f$ measured in radians, their mean resultant length
 * is defined as (@cite Mardia_1972):
 *
 * \f$\overline{R} = \sqrt{\overline{C}^2 + \overline{S}^2} \f$,
 *
 * where \f$\overline{C} = \frac{1}{n} \sum_\limits{i=1}^{n} \cos \alpha_i \f$ and
 * \f$\overline{S} = \frac{1}{n} \sum_\limits{i=1}^{n} \sin \alpha_i \f$.
 *
 * @param sines_sum Sum of the sines of a sample of angle data.
 * @param cosines_sum Sum of the cosines of a sample of angle data.
 * @param num_samples Total number of samples in the set of angle data.
 * @return Mean resultant length of the sample.
 */
float computeMeanResultantLength(float sines_sum, float cosines_sum, float num_samples);

NS_FOOT
