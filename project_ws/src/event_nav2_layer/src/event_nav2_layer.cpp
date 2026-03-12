#include "event_nav2_layer/event_nav2_layer.hpp"

#include <algorithm>
#include <utility>

#include <geometry_msgs/msg/point.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

namespace event_nav2_layer
{

EventNav2Layer::EventNav2Layer() = default;

void EventNav2Layer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    return;
  }

  // Layer-private parameters: "<layer_name>.enabled", "<layer_name>.pointcloud_topic", etc.
  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("pointcloud_topic", rclcpp::ParameterValue(std::string("/dynamic_obstacles")));
  declareParameter("max_obstacle_height", rclcpp::ParameterValue(2.0));

  node->get_parameter(name_ + ".enabled", enabled_);
  node->get_parameter(name_ + ".pointcloud_topic", pointcloud_topic_);
  node->get_parameter(name_ + ".max_obstacle_height", max_obstacle_height_);

  sub_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
    pointcloud_topic_, rclcpp::SensorDataQoS(),
    std::bind(&EventNav2Layer::pointcloudCallback, this, std::placeholders::_1));

  // This layer does not keep stale "unknown" state between updates.
  current_ = true;
}

void EventNav2Layer::reset()
{
  std::lock_guard<std::mutex> lock(mutex_);
  points_.clear();
}

void EventNav2Layer::pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  if (!enabled_) {
    return;
  }

  std::vector<geometry_msgs::msg::Point> points;
  sensor_msgs::msg::PointCloud2 cloud_out;

  const std::string global_frame = layered_costmap_->getGlobalFrameID();
  try {
    // Transform incoming points to the costmap global frame once, then store them.
    geometry_msgs::msg::TransformStamped transform =
      tf_->lookupTransform(global_frame, msg->header.frame_id, msg->header.stamp);
    tf2::doTransform(*msg, cloud_out, transform);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 2000, "Transform failed for dynamic obstacles: %s", ex.what());
    return;
  }

  for (sensor_msgs::PointCloud2ConstIterator<float> it_x(cloud_out, "x"),
       it_y(cloud_out, "y"), it_z(cloud_out, "z");
       it_x != it_x.end(); ++it_x, ++it_y, ++it_z)
  {
    geometry_msgs::msg::Point p;
    p.x = *it_x;
    p.y = *it_y;
    p.z = *it_z;
    if (p.z <= max_obstacle_height_) {
      points.push_back(p);
    }
  }

  // Atomic swap keeps updateBounds/updateCosts lock sections short.
  std::lock_guard<std::mutex> lock(mutex_);
  points_.swap(points);
}

void EventNav2Layer::updateBounds(
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/, double * min_x, double * min_y,
  double * max_x, double * max_y)
{
  if (!enabled_) {
    return;
  }

  // Expand the dirty window to include all currently tracked obstacle points.
  std::lock_guard<std::mutex> lock(mutex_);
  for (const auto & p : points_) {
    *min_x = std::min(*min_x, p.x);
    *min_y = std::min(*min_y, p.y);
    *max_x = std::max(*max_x, p.x);
    *max_y = std::max(*max_y, p.y);
  }
}

void EventNav2Layer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_) {
    return;
  }

  // Mark each projected point as lethal obstacle in the update window.
  std::lock_guard<std::mutex> lock(mutex_);
  for (const auto & p : points_) {
    unsigned int mx, my;
    if (master_grid.worldToMap(p.x, p.y, mx, my)) {
      if (mx >= static_cast<unsigned int>(min_i) && mx < static_cast<unsigned int>(max_i) &&
        my >= static_cast<unsigned int>(min_j) && my < static_cast<unsigned int>(max_j))
      {
        master_grid.setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);
      }
    }
  }
}

}  // namespace event_nav2_layer

PLUGINLIB_EXPORT_CLASS(event_nav2_layer::EventNav2Layer, nav2_costmap_2d::Layer)
