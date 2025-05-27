#pragma once

#include <mutex>
#include <string>
#include <vector>
#include <grid_map_core/GridMap.hpp>
#include "package_defs.hpp"

NS_HEAD

/**
 * @brief Inherits GridMap to provide threadsafe functions to access grid map.
 *
 */
class RadarGridMap : public grid_map::GridMap
{
public:
  /**
   * @brief Construct a new Radar Grid Map object with given layers.
   *
   * @param layers Names of layers to create in the grid map.
   */
  RadarGridMap(const std::vector<std::string>& layers);

  /**
   * @brief Construct a new empty Radar Grid Map object with no layers.
   */
  RadarGridMap();

  /**
   * @brief Locking function to re-center map on the given position.
   *
   * @param position Position to which to move center of map.
   * @return true If map has been moved, false otherwise.
   */
  bool moveMap(const grid_map::Position& position);

  /**
   * @brief Locking function to access cell data at given map index.
   *
   * @param layer Name of layer in which to access cell data.
   * @param index Index of the cell to access.
   * @return float& The data of the cell.
   */
  float& atCell(const std::string& layer, const grid_map::Index& index);

  /**
   * @brief Locking function to get cell data at given map position.
   *
   * @param layer Name of layer in which to access cell data.
   * @param position Position of the map to be accessed.
   * @return float& The data of the cell.
   */
  float& atMapPosition(const std::string& layer, const grid_map::Position& position);

private:
  std::mutex mtx_;
};

NS_FOOT
