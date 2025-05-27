#include "radar_grid_map.hpp"

NS_HEAD

RadarGridMap::RadarGridMap(const std::vector<std::string>& layers) : grid_map::GridMap(layers)
{
}

RadarGridMap::RadarGridMap() : grid_map::GridMap()
{
}

bool RadarGridMap::moveMap(const grid_map::Position& position)
{
  std::unique_lock<std::mutex> lock(mtx_);
  return this->move(position);
}

float& RadarGridMap::atCell(const std::string& layer, const grid_map::Index& index)
{
  std::unique_lock<std::mutex> lock(mtx_);
  return this->at(layer, index);
}

float& RadarGridMap::atMapPosition(const std::string& layer, const grid_map::Position& position)
{
  std::unique_lock<std::mutex> lock(mtx_);
  return this->atPosition(layer, position);
}

NS_FOOT
