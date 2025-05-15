#include "grid_map_filters.hpp"

NS_HEAD

void computeEDTFromIntensity(grid_map::GridMap& map,
                             const std::string& intensity_layer,
                             const std::string& distance_layer)
{
  if (!map.exists(intensity_layer)) {
    throw std::runtime_error("GridMap does not contain intensity layer");
  }

  const float map_resolution = map.getResolution();

  // Convert radar intensity layer to OpenCV image and invert it to
  // create a binary OpenCV mask where occupied = 0, free = 255
  cv::Mat radar_intensity_image;
  grid_map::GridMapCvConverter::toImage<uint8_t, 1>(
          map, intensity_layer, CV_8UC1, radar_intensity_image);
  cv::Mat binary_mask;
  cv::bitwise_not(radar_intensity_image, binary_mask);

  // Compute the distance transform (in pixels)
  cv::Mat distance_image;
  cv::distanceTransform(binary_mask, distance_image, cv::DIST_L2, 3);

  // Convert to physical distances in meters
  distance_image *= map_resolution;

  // Store result in GridMap layer
  if (!map.exists(distance_layer)) {
    map.add(distance_layer, std::numeric_limits<float>::quiet_NaN());
  }

  for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
    const grid_map::Index img_idx(it.getUnwrappedIndex());
    map.at(distance_layer, *it) = distance_image.at<float>(img_idx(0), img_idx(1));
  }
}

NS_FOOT
