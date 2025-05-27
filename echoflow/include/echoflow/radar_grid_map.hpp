#pragma once

#include <string>
#include <vector>
#include <grid_map_core/GridMap.hpp>
#include "package_defs.hpp"

NS_HEAD

/**
 * @brief Inherits GridMap to provide threadsafe grid map access functions.
 *
 */
class RadarGridMap : public grid_map::GridMap
{
public:
    /**
     * @brief Construct a new Radar Grid Map object with layer names.
     *
     * @param layers Names of layers to create in the grid map.
     */
    RadarGridMap(const std::vector<std::string>& layers);

    /**
     * @brief Construct a new empty Radar Grid Map object with no layers.
     *
     */
    RadarGridMap();

};

NS_FOOT
