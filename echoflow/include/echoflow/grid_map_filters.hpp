/** Copyright © 2025 Seaward Science. */

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
 * @brief Filters large blobs from the input map layer and stores filtered result to output map layer.
 *
 * Uses the built-in grid_map function to convert the input layer of the grid map to an OpenCV image,
 * then uses the OpenCV function to perform connected components analysis to find the sizes of the
 * blobs in the image. If the blob is larger than the max blob size, clears the blob from the output
 * layer.
 *
 * @param map Grid map to modify.
 * @param input_layer Name of layer in grid map for which to filter large blobs.
 * @param output_layer Name of layer in grid map to store the filtered output.
 * @param max_blob_area Maximum size of blob to filter.
 */
void filterLargeBlobsFromLayer(grid_map::GridMap& map,
                               const std::string& input_layer,
                               const std::string& output_layer,
                               double max_blob_area);

}  // namespace grid_map_filters

NS_FOOT
