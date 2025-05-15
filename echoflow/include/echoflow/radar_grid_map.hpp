#pragma once

#include <string>
#include <vector>
#include <grid_map_core/GridMap.hpp>
#include "package_defs.hpp"

/**
 * @brief todo
 *
 */
class RadarGridMap : public grid_map::GridMap
{
public:
    /**
     * @brief Construct a new Radar Grid Map object
     *
     * @param layers
     */
    RadarGridMap(const std::vector<std::string>& layers);

    /**
     * @brief Construct a new Radar Grid Map object
     *
     */
    RadarGridMap();

    /**
     * @brief
     *
     */
    void getCellAtPosition(); // todo return type

    /**
     * @brief
     *
     */
    bool setCellAtPosition();

    /**
     * @brief Currently just a stub function called whenever cell is updated.
     * TODO: implement this
     */
    void addToBuffer();

};
