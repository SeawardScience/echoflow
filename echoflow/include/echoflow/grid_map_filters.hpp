#pragma once

#include "package_defs.hpp"

#include <grid_map_core/grid_map_core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <limits>

NS_HEAD

void computeEDTFromIntensity(grid_map::GridMap& map, const std::string& intensity_layer, const std::string& distance_layer)
{
  if (!map.exists(intensity_layer)) {
    throw std::runtime_error("GridMap does not contain intensity layer");
  }

  const auto size = map.getSize();
  const int rows = size.y();  // grid_map is (rows, cols)
  const int cols = size.x();
  const float resolution = map.getResolution();

  // Create a binary OpenCV mask where occupied = 0, free = 255
  cv::Mat binary_mask(rows, cols, CV_8UC1, cv::Scalar(255));

  for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
    const auto& idx = *it;
    float intensity = map.at(intensity_layer, idx);
    if (!std::isnan(intensity) && intensity > 0.0f) {
      binary_mask.at<uchar>(cols - 1 - idx[1], idx[0]) = 0;  // flip y for OpenCV
    }
  }

  // Compute the distance transform (in pixels)
  cv::Mat distance_image;
  cv::distanceTransform(binary_mask, distance_image, cv::DIST_L2, 3);

  // Convert to physical distances in meters
  distance_image *= resolution;

  // Store result in GridMap layer
  map.add(distance_layer, std::numeric_limits<float>::quiet_NaN());

  for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
    const auto& idx = *it;
    float dist = distance_image.at<float>(cols - 1 - idx[1], idx[0]);
    map.at(distance_layer, idx) = dist;
  }
}

NS_FOOT
