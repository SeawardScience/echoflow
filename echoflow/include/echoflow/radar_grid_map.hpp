#pragma once

#include <mutex>
#include <string>
#include <vector>
#include <grid_map_core/GridMap.hpp>
#include <grid_map_core/GridMapMath.hpp>
#include "package_defs.hpp"

NS_HEAD

/**
 * @brief Inherits GridMap to provide threadsafe functions to access grid map.
 *
 */
class ThreadsafeGridMap : private grid_map::GridMap
{
public:
  /**
   * @brief Construct a new Radar Grid Map object with given layers.
   *
   * @param layers Names of layers to create in the grid map.
   */
  ThreadsafeGridMap(const std::vector<std::string>& layers);

  /**
   * @brief Construct a new empty Radar Grid Map object with no layers.
   */
  ThreadsafeGridMap();

  /**
   * @brief Locking function to access cell data at given map index.
   *
   * @param layer Name of layer in which to access cell data.
   * @param index Index of the cell to access.
   * @return float& The data of the cell.
   */
  float& at(const std::string& layer, const grid_map::Index& index);

  /**
   * @brief Locking function to get cell data at given map position.
   *
   * @param layer Name of layer in which to access cell data.
   * @param position Position of the map to be accessed.
   * @return The data of the cell.
   */
  float& atPosition(const std::string& layer, const grid_map::Position& position);

  /**
   * @brief Locking function to get the 2D position of the grid map in the grid map frame.
   *
   * @return Position of the grid map in the grid map frame.
   */
  const grid_map::Position& getPosition() const;

  bool isInside(const grid_map::Position& position) const;

  /**
   * @brief Locking function to re-center map on the given position.
   *
   * @param position Position to which to move center of map.
   * @return true If map has been moved, false otherwise.
   */
  bool move(const grid_map::Position& position);

  /**
   * @brief Locking function to set frame ID of the grid map.
   *
   * @param frameId  The frame ID to set.
   */
  void setFrameId(const std::string& frameId);

  /**
   * @brief Locking function that sets the geometry of the grid map. Clears all the data.
   *
   * @param length Side lengths in x, and y-direction of the grid map [m].
   * @param resolution Cell size in [m/cell].
   * @param position 2D position of the grid map in the grid map frame [m].
   */
  void setGeometry(const grid_map::Length& length,
                   const double resolution,
                   const grid_map::Position& position);

  /**
   * @brief Locking function to get the side length of the grid map.
   *
   * @return Side length of the grid map.
   */
  grid_map::Length& getLength();

  /**
   * @brief Locking function to get the grid map size (rows and cols of the data structure).
   *
   * @return Grid map size.
   */
  const grid_map::Size& getSize();


private:
  std::mutex mtx_;
};

NS_FOOT
