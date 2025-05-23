#pragma once

#include <cmath>
#include <limits>
#include <string>
#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_cv/grid_map_cv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include "package_defs.hpp"


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

/**
 * @brief Compute the sequential arithmetic mean of a sample given a new observation.
 *
 * Given a new observation \f$x_n\f$, the prior mean of the data \f$\overline{x}_{n-1}\f$,
 * and the total number of observations \f$n\f$,
 * the sequential mean \f$\overline{x}_n\f$ is computed as follows:
 *
 * \f$ \overline{x}_n = \overline{x}_{n-1} + \frac{x_n - \overline{x}_{n-1}}{n} \f$
 *
 * @param num_samples Total number of samples (including new observation).
 * @param new_observation New value to add to the computation of the mean.
 * @param prior_mean Prior mean of the sample data.
 */
float computeSequentialMean(float new_observation, float num_samples, float prior_mean);

/**
 * @brief Compute the circular mean angle on a sample of angle data.
 *
 * Given \f$n\f$ angles \f$\alpha_1, ..., \alpha_n\f$ measured in radians, their circular mean is defined as
 * (@cite Mardia_1972, section 2.2.2):
 *
 * \f$\overline{\alpha}= \textrm{arg}\biggl(\sum_{j=1}^{n} e^{i\cdot\alpha_j} \biggr)\f$
 */
void computeCircularMean();

/**
 * @brief Compute the circular standard deviation on a sample of angle data.
 *
 * Given \f$n\f$ angles \f$\alpha_1, ..., \alpha_n\f$ measured in radians, their circular standard deviation
 * is defined as (@cite Mardia_1972, section 2.3.4, Eq. 2.3.12):
 *
 */
void computeCircularStdDev();

NS_FOOT
