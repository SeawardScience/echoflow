/** Copyright © 2015 Seaward Science. */

#pragma once

#include <limits>
#include <string>
#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_cv/grid_map_cv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include "package_defs.hpp"

NS_HEAD

namespace grid_map_filters {

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
 * @brief TODO
 *
 * @param map
 * @param input_layer
 * @param output_layer
 * @param max_blob_area
 */
void filterLargeBlobsFromLayer(grid_map::GridMap& map,
                               const std::string& input_layer,
                               const std::string& output_layer,
                               double max_blob_area);

}  // namespace grid_map_filters

NS_FOOT
