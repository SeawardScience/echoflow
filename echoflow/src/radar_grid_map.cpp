#include "radar_grid_map.hpp"


RadarGridMap::RadarGridMap(const std::vector<std::string>& layers) : grid_map::GridMap(layers)
{
    
}

RadarGridMap::RadarGridMap() : grid_map::GridMap()
{

}

void RadarGridMap::getCellAtPosition()
{

}

bool RadarGridMap::setCellAtPosition()
{

    // Dummy function for later implementation, adds updated cell to buffer for processing
    addToBuffer();
}

void RadarGridMap::addToBuffer() 
{
}
