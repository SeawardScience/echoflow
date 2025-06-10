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
  grid_map::GridMapCvConverter::toImage<unsigned char, 1>(
      map, intensity_layer, CV_8UC1, 0.0, 1.0, radar_intensity_image);
  cv::Mat binary_mask;
  cv::threshold(radar_intensity_image, binary_mask, 0, 255, cv::THRESH_BINARY_INV);

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

void filterLargeBlobsFromLayer(grid_map::GridMap& map,
                               const std::string& input_layer,
                               const std::string& output_layer,
                               double max_blob_area)
{
  if (!map.exists(input_layer)) {
    throw std::runtime_error("GridMap does not contain input layer: " + input_layer);
  }

  // Convert input layer to binary OpenCV image
  cv::Mat input_img;
  grid_map::GridMapCvConverter::toImage<unsigned char, 1>(
      map, input_layer, CV_8UC1, 0.0, 1.0, input_img);

  // Connected components analysis
  cv::Mat labels, stats, centroids;
  int num_labels = cv::connectedComponentsWithStats(input_img, labels, stats, centroids, 8, CV_32S);

  // Output image (filtered binary)
  cv::Mat filtered_img = cv::Mat::zeros(input_img.size(), CV_8UC1);

  for (int label = 1; label < num_labels; ++label) {
    int area = stats.at<int>(label, cv::CC_STAT_AREA);
    if (area <= max_blob_area) {
      filtered_img.setTo(255, labels == label);
    }
  }

  // Convert to float [0.0, 1.0]
  cv::Mat filtered_float;
  filtered_img.convertTo(filtered_float, CV_32FC1, 1.0 / 255.0);

  // Add or overwrite output layer
  if (!map.exists(output_layer)) {
    map.add(output_layer, std::numeric_limits<float>::quiet_NaN());
  }

  for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
    const grid_map::Index idx = it.getUnwrappedIndex();
    map.at(output_layer, *it) = filtered_float.at<float>(idx(0), idx(1));
  }
}


float computeSequentialMean(float new_observation, float num_samples, float prior_mean)
{
  if (num_samples < 1) {
    throw std::invalid_argument("Invalid number of particles.");
  }
  if (num_samples == 1) {
    return new_observation;
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

NS_FOOT
