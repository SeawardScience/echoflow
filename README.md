# echoflow

**echoflow** is a ROS 2 package designed to transform marine radar sector data into a real-time, 2D grid map for navigation and environmental awareness.

The `radar_grid_map` node listens to radar sector messages, applies optional near-field clutter filtering, transforms the data into a global map frame using TF2, and publishes both a detailed `GridMap` and a simplified `OccupancyGrid` suitable for further path planning, SLAM, or tracking algorithms.

![echoflow visualization](docs/media/radar_grid_map_demo.gif)

---

## Installation

To install `echoflow` without compiling manually:

```bash
# Clone the repository into your workspace src/
cd ~/ros2_ws/src
git clone https://github.com/YOUR_ORG/echoflow.git

# Install dependencies
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

---

## Compiling

To build the package and all required dependencies:

```bash
cd ~/ros2_ws
colcon build --packages-select echoflow
source install/setup.bash
```

You may also wish to build with tests and warnings enabled:

```bash
colcon build --packages-select echoflow --cmake-args -DCMAKE_BUILD_TYPE=Release
```

---

## Running the Package

After sourcing your workspace, you can start the node:

```bash
ros2 run echoflow radar_grid_map
```

Make sure appropriate TF data (e.g., `map -> base_link`) and radar sector topics are available.

---

## Nodes

### radar_grid_map Node

This node processes incoming marine radar sectors and builds a continuously updating 2D map.

#### Published Topics

| Topic                  | Message Type                          | Description                         |
|-------------------------|---------------------------------------|-------------------------------------|
| `occupancy_grid`   | `nav_msgs::msg::OccupancyGrid`         | Simplified occupancy representation |

#### Subscribed Topics

| Topic                  | Message Type                          | Description                         |
|-------------------------|---------------------------------------|-------------------------------------|
| `data`                 | `marine_sensor_msgs::msg::RadarSector` | Incoming radar sector scans         |

The above correspond to the standards set in the UNH marine radar messages.  Therefore you should run this node in the namespace of the desired radar channel.  For example.

```
ros2 run echoflow radar_grid_map --ros-args -r __ns:=/aura/perception/sensors/halo_a 
```

#### Parameters

| Parameter Name                     | Type          | Default Value  | Description                                          |
|-------------------------------------|---------------|----------------|------------------------------------------------------|
| `map.frame_id`                     | `std::string` | `"map"`        | The fixed frame for the output grid map.             |
| `map.length`                       | `float`       | `10000.0`      | Length of the map area in meters.                    |
| `map.width`                        | `float`       | `10000.0`      | Width of the map area in meters.                     |
| `map.resolution`                   | `float`       | `10.0`         | Resolution (cell size) of the map in meters.         |
| `map.pub_interval`                 | `float`       | `0.1`          | Time between costmap publication updates (seconds). |
| `filter.near_clutter_range`         | `float`       | `30.0`         | Maximum range in meters for clutter filtering.       |
| `max_queue_size`                   | `int`         | `1000`         | Maximum radar message queue length.                  |

Example configuration snippet:

```yaml
map:
  frame_id: "map"
  length: 10000.0
  width: 10000.0
  resolution: 10.0
  pub_interval: 0.1

filter:
  near_clutter_range: 30.0

max_queue_size: 1000
```

---

## Services

Currently, this node does not provide ROS2 services.

---

## Example Use Cases

- Real-time mapping of harbor environments from marine radar
- Providing occupancy grids for obstacle avoidance on ASVs (Autonomous Surface Vessels)
- Feeding particle filters or SLAM algorithms with radar-derived data
- Situational awareness systems for marine robotics

---

## Contributing

We welcome contributions!  
Please read the [CONTRIBUTING.md](CONTRIBUTING.md) guidelines to get started.

When making contributions, remember:
- Follow the [Semantic Versioning 2.0](https://semver.org/) model.
- Ensure all changes maintain deployability of the `master` branch.
- Keep documentation (README, Doxygen comments) up to date.

---

## License

This project is licensed under the LICENSE provided in this repository.

---

## Credits

Developed by [Seaward Science](https://seaward.science/) for University of New Hampshire Center for Coastal and Ocean Mapping [CCOM](https://www.ccom.unh.edu/)

### Authors
- Dr. Kristohper Krasnosky (lead software engineer)
- Antonella Willby (software developer)


