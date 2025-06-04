#include "radar_grid_map.hpp"

NS_HEAD

ThreadsafeGridMap::ThreadsafeGridMap(const std::vector<std::string>& layers) : grid_map::GridMap(layers)
{
}

ThreadsafeGridMap::ThreadsafeGridMap() : grid_map::GridMap()
{
}


float& ThreadsafeGridMap::at(const std::string& layer, const grid_map::Index& index)
{
  std::unique_lock<std::mutex> lock(mtx_);
  return this->at(layer, index);
}

float& ThreadsafeGridMap::atPosition(const std::string& layer, const grid_map::Position& position)
{
  std::unique_lock<std::mutex> lock(mtx_);
  return this->atPosition(layer, position);
}

const grid_map::Position& ThreadsafeGridMap::getPosition() const
{
  //std::unique_lock<std::mutex> lock(mtx_);
  return this->getPosition();
}

bool ThreadsafeGridMap::isInside(const grid_map::Position& position) const
{
  grid_map::Length& length = this->getLength();
  grid_map::Position& position = getPosition();
  return grid_map::checkIfPositionWithinMap(position, length, getPosition());
}

bool ThreadsafeGridMap::move(const grid_map::Position& position)
{
  std::unique_lock<std::mutex> lock(mtx_);
  return this->move(position);
}

void ThreadsafeGridMap::setFrameId(const std::string& frameId)
{
  std::unique_lock<std::mutex> lock(mtx_);
  this->setFrameId(frameId);
}

void ThreadsafeGridMap::setGeometry(const grid_map::Length& length,
                                    const double resolution,
                                    const grid_map::Position& position)
{
  std::unique_lock<std::mutex> lock(mtx_);
  this->setGeometry(length, resolution, position);
}

grid_map::Length& ThreadsafeGridMap::getLength()
{
  std::unique_lock<std::mutex> lock(mtx_);
  return this->getLength();
}

const grid_map::Size& ThreadsafeGridMap::getSize()
{
  std::unique_lock<std::mutex> lock(mtx_);
  return this->getSize();
}

NS_FOOT
