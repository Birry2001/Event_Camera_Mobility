#pragma once

#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <dv_ros2_msgs/msg/event.hpp>
#include <dv_ros2_msgs/msg/event_array.hpp>

class EventVisualizer : public rclcpp::Node {
public:
    EventVisualizer();
private:
    void show_count_image(const std::vector<std::vector<int>> &count_image,
                          int max_count,
                          const std_msgs::msg::Header &header,
                          const rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr &pub);
    void data_process();
    void event_cb(const dv_ros2_msgs::msg::EventArray::SharedPtr msg);
    void imu_cb(const sensor_msgs::msg::Imu::SharedPtr imu);
    void depth_cb(const sensor_msgs::msg::Image::SharedPtr msg);
    void camera_info_cb(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
    void odom_cb(const nav_msgs::msg::Odometry::SharedPtr msg);
    static int64_t toNs(const builtin_interfaces::msg::Time &t);
    bool get_depth_meters(int u, int v, const sensor_msgs::msg::Image &depth,
                          double &depth_m) const;
    bool get_odom_at(const rclcpp::Time &stamp,
                     tf2::Vector3 &position,
                     tf2::Quaternion &orientation) const;
    bool get_extrinsic_base_to_camera(tf2::Transform &T_b_c);

    std::string events_topic_;
    std::string imu_topic_;
    std::string depth_topic_;
    std::string camera_info_topic_;
    std::string odom_topic_;
    std::string count_image_topic_;
    std::string time_image_topic_;
    std::string time_image_vis_topic_;
    bool publish_time_image_ = true;
    bool publish_time_image_vis_ = true;
    bool sort_events_by_time_ = true;
    bool require_depth_ = true;
    bool require_odom_ = true;
    bool use_imu_rotation_ = true;
    double max_depth_age_ms_ = 50.0;
    double depth_scale_ = 0.001;
    int64_t odom_buffer_ns_ = 2000000000;
    std::string base_frame_;
    std::string camera_frame_;

    rclcpp::Subscription<dv_ros2_msgs::msg::EventArray>::SharedPtr event_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr time_image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr time_image_vis_pub_;

    std::vector<dv_ros2_msgs::msg::Event> event_buffer_;
    std::vector<sensor_msgs::msg::Imu> imu_buffer_;
    std::vector<sensor_msgs::msg::Imu> imu_buffer_snapshot_;
    std::deque<nav_msgs::msg::Odometry> odom_buffer_;
    std::deque<nav_msgs::msg::Odometry> odom_buffer_snapshot_;

    sensor_msgs::msg::Image::SharedPtr depth_image_;
    sensor_msgs::msg::CameraInfo::SharedPtr camera_info_;
    bool has_intrinsics_ = false;
    double fx_ = 0.0;
    double fy_ = 0.0;
    double cx_ = 0.0;
    double cy_ = 0.0;

    std_msgs::msg::Header last_event_header_;
    bool first_event_received_ = true;
    bool require_imu_ahead_   = true;
    int64_t imu_window_ns_    = 3000000;

    std::mutex mtx_;
    std::mutex depth_mtx_;
    std::mutex odom_mtx_;

    tf2_ros::Buffer tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    bool have_extrinsic_ = false;
    tf2::Transform T_b_c_;
};
