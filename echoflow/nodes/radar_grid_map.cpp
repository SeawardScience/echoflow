/** Copyright © 2025 Seaward Science. */

#include "radar_grid_map_node.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<echoflow::RadarGridMapNode>());
    rclcpp::shutdown();
    return 0;
}
