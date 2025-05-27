#include "radar_grid_map.hpp"

NS_HEAD

RadarGridMap::RadarGridMap(const std::vector<std::string>& layers) : grid_map::GridMap(layers)
{
}

RadarGridMap::RadarGridMap() : grid_map::GridMap()
{
}

NS_FOOT
