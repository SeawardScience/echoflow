#include "particle_filter_node.hpp"
#include "radar_grid_map_node.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto filter_node = std::make_shared<echoflow::ParticleFilterNode>();
  auto grid_node = std::make_shared<echoflow::RadarGridMapNode>();

  filter_node->map_ptr_ = grid_node->getMapPtr();

  rclcpp::executors::MultiThreadedExecutor executor;
  //rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(filter_node);
  executor.add_node(grid_node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
