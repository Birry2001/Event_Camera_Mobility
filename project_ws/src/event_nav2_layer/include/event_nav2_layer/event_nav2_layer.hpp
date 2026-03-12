#ifndef EVENT_NAV2_LAYER__EVENT_NAV2_LAYER_HPP_
#define EVENT_NAV2_LAYER__EVENT_NAV2_LAYER_HPP_

#include <mutex>
#include <string>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <nav2_costmap_2d/layer.hpp>
#include <nav2_costmap_2d/layered_costmap.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace event_nav2_layer
{

class EventNav2Layer : public nav2_costmap_2d::Layer
{
public:
  EventNav2Layer();
  void onInitialize() override;
  void updateBounds(
    double robot_x, double robot_y, double robot_yaw, double * min_x, double * min_y,
    double * max_x, double * max_y) override;
  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j, int max_i,
    int max_j) override;
  void reset() override;
  bool isClearable() override {return false;}

private:
  void pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  std::mutex mutex_;
  std::vector<geometry_msgs::msg::Point> points_;
  std::string pointcloud_topic_;
  double max_obstacle_height_{2.0};
  bool enabled_{true};
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
};

}  // namespace event_nav2_layer

#endif  // EVENT_NAV2_LAYER__EVENT_NAV2_LAYER_HPP_
