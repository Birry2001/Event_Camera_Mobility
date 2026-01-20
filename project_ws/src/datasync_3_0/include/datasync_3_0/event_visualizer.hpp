#pragma once

#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>

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
    static int64_t toNs(const builtin_interfaces::msg::Time &t);

    std::string events_topic_;
    std::string imu_topic_;
    std::string count_image_topic_;
    std::string time_image_topic_;
    std::string time_image_vis_topic_;
    bool publish_time_image_ = true;
    bool publish_time_image_vis_ = true;
    bool publish_raw_count_ = false;
    bool publish_comp_count_ = false;
    std::string raw_count_topic_;
    std::string comp_count_topic_;
    bool sort_events_by_time_ = true;

    rclcpp::Subscription<dv_ros2_msgs::msg::EventArray>::SharedPtr event_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr time_image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr time_image_vis_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr raw_count_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr comp_count_pub_;

    std::vector<dv_ros2_msgs::msg::Event> event_buffer_;
    std::vector<sensor_msgs::msg::Imu> imu_buffer_;
    std::vector<sensor_msgs::msg::Imu> imu_buffer_snapshot_;

    std_msgs::msg::Header last_event_header_;
    bool first_event_received_ = true;
    bool require_imu_ahead_   = true;
    int64_t imu_window_ns_    = 3000000;

    std::mutex mtx_;
};
