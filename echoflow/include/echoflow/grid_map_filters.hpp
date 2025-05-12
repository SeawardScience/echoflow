#pragma once

#include "package_defs.hpp"

#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_cv/grid_map_cv.hpp>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
//#include <opencv2/highgui/highgui.hpp>  // imshow

#include <limits>

NS_HEAD

    void computeEDTFromIntensity(grid_map::GridMap& map, const std::string& intensity_layer, const std::string& distance_layer)
{
  if (!map.exists(intensity_layer)) {
    throw std::runtime_error("GridMap does not contain intensity layer");
  }

  const float resolution = map.getResolution();
  const grid_map::Size size = map.getSize();
  const int rows = size(1);  // rows = y
  const int cols = size(0);  // cols = x

  // Convert the intensity layer to an OpenCV float image
  cv::Mat intensity_image;
  grid_map::GridMapCvConverter::toImage<unsigned char, 1>(
      map, intensity_layer, CV_8UC1, 0.0, 1.0, intensity_image);

  // Convert intensity to binary: occupied (intensity > 0) = 0, else 255
  cv::Mat binary_mask;
  cv::threshold(intensity_image, binary_mask, 0, 255, cv::THRESH_BINARY_INV);

  // cv::imshow("Binary Mask", binary_mask);
  // cv::waitKey(1);

  // Compute the distance transform
  cv::Mat distance_image;
  cv::distanceTransform(binary_mask, distance_image, cv::DIST_L2, 3);

  // Scale to real-world units
  distance_image *= resolution;

  // cv::imshow("distance image", distance_image);
  // cv::waitKey(1);

  if (!map.exists(distance_layer)) {
    map.add(distance_layer, std::numeric_limits<float>::quiet_NaN());
  }

  const grid_map::Index& bufferStart = map.getStartIndex();
  const grid_map::Size& bufferSize = map.getSize();

  for (int y = 0; y < bufferSize(1); ++y) {
    for (int x = 0; x < bufferSize(0); ++x) {
      // Logical index (0,0) is top-left of the cv::Mat
      float val = distance_image.at<float>(y, x);

      // Convert logical index to circular buffer index
      grid_map::Index bufferIdx;
      bufferIdx[0] = (y + bufferStart[0]) % bufferSize[0];
      bufferIdx[1] = (x + bufferStart[1]) % bufferSize[1];

      map.at(distance_layer, bufferIdx) = val;
    }
  }

}

NS_FOOT
