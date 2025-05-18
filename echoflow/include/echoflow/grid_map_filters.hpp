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
 * @brief Compute the circular mean angle on a sample of angle observations.
 *
 * \f$\overline{\alpha}= \textrm{atan2}\biggl(\sum_{j=1}^{n} \sin \alpha_j, \sum_{j=1}^{n} \cos \alpha_j) \biggr)\f$
 *
 * Function is implemented using TODO
 *
 *
 */
void computeCircularMean();

NS_FOOT
