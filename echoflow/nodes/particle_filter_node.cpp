#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include "particle_filter.hpp"
#include <radar_grid_map_node.hpp>
#include <grid_map_filters.hpp>

NS_HEAD

class ParticleFilterNode : public rclcpp::Node {
public:
  ParticleFilterNode() : Node("particle_filter_node") {
    this->declare_parameter("num_particles", 100000);
    size_t num_particles = this->get_parameter("num_particles").as_int();
    pf_ = std::make_unique<MultiTargetParticleFilter>(num_particles);

    cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("particle_cloud", 10);

    timer_ = create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&ParticleFilterNode::update, this));

    last_update_time_ = now();
  }

  std::shared_ptr<grid_map::GridMap> map_ptr_;

private:
  // void detectionCallback(const geometry_msgs::msg::Point::SharedPtr msg) {
  //   Detection d{msg->x, msg->y};
  //   pending_detections_.push_back(d);
  // }

  void update() {
    auto now_time = now();
    double dt = (now_time - last_update_time_).seconds();
    last_update_time_ = now_time;

    computeEDTFromIntensity(*map_ptr_,"intensity","edt");

    if (!initialized_) {
      pf_->initialize(map_ptr_);
      initialized_ = true;
    }

    if (!initialized_) return;

    pf_->initialize(map_ptr_);

    pf_->predict(dt);
    pf_->updateWeights(map_ptr_);
    pf_->resample();
    pending_detections_.clear();

    publishPointCloud();
  }

  void publishPointCloud() {




    const auto& particles = pf_->getParticles();
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.frame_id = "map";
    cloud.header.stamp = this->get_clock()->now();
    cloud.height = 1;
    cloud.width = particles.size();

    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2Fields(
        7,  // number of fields
        "x", 1, sensor_msgs::msg::PointField::FLOAT32,
        "y", 1, sensor_msgs::msg::PointField::FLOAT32,
        "z", 1, sensor_msgs::msg::PointField::FLOAT32,
        "heading", 1, sensor_msgs::msg::PointField::FLOAT32,
        "speed", 1, sensor_msgs::msg::PointField::FLOAT32,
        "yaw_rate", 1, sensor_msgs::msg::PointField::FLOAT32,
        "weight", 1, sensor_msgs::msg::PointField::FLOAT32);
    modifier.resize(particles.size());

    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
    sensor_msgs::PointCloud2Iterator<float> iter_heading(cloud, "heading");
    sensor_msgs::PointCloud2Iterator<float> iter_speed(cloud, "speed");
    sensor_msgs::PointCloud2Iterator<float> iter_yaw_rate(cloud, "yaw_rate");
    sensor_msgs::PointCloud2Iterator<float> iter_weight(cloud, "weight");

    for (const auto& p : particles) {
      *iter_x = static_cast<float>(p.x);
      *iter_y = static_cast<float>(p.y);
      *iter_z = 0.0f;
      *iter_heading = static_cast<float>(p.heading);
      *iter_speed = static_cast<float>(p.speed);
      *iter_yaw_rate = static_cast<float>(p.yaw_rate);
      *iter_weight = static_cast<float>(p.weight);
      ++iter_x; ++iter_y; ++iter_z;
      ++iter_heading; ++iter_speed; ++iter_yaw_rate; ++iter_weight;
    }

    cloud_pub_->publish(cloud);
  }

  std::unique_ptr<MultiTargetParticleFilter> pf_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr detections_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::vector<Detection> pending_detections_;
  bool initialized_ = false;
  rclcpp::Time last_update_time_;
};

NS_FOOT

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto filter_node = std::make_shared<echoflow::ParticleFilterNode>();
  auto grid_node = std::make_shared<echoflow::RadarGridMapNode>();

  filter_node->map_ptr_ = grid_node->getMapPtr();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(filter_node);
  executor.add_node(grid_node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
