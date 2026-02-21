#pragma once

#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>

#include <dv_ros2_msgs/msg/event.hpp>
#include <dv_ros2_msgs/msg/event_array.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>


using namespace std;

class EventVisualizer : public rclcpp::Node {
public:
    EventVisualizer();
private:
    static constexpr int64_t kUnsetTimeNs = std::numeric_limits<int64_t>::min();

    void show_count_image(const std::vector<std::vector<int>> &count_image,
                          int max_count,
                          const std_msgs::msg::Header &header,
                          const rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr &pub);
    void data_process();
    void event_cb(const dv_ros2_msgs::msg::EventArray::SharedPtr msg);
    void imu_cb(const sensor_msgs::msg::Imu::SharedPtr imu);
    static int64_t toNs(const builtin_interfaces::msg::Time &t);
    void init_filter_state();
    std::vector<dv_ros2_msgs::msg::Event> filter_events(
        const std::vector<dv_ros2_msgs::msg::Event> &input_events);
    inline size_t pixel_index(int x, int y) const;
    float compute_isolated_ratio(
        const std::vector<dv_ros2_msgs::msg::Event> &events) const;
    float compute_hot_ratio(
        const std::vector<dv_ros2_msgs::msg::Event> &events,
        int hot_threshold) const;
    float compute_small_component_ratio(
        const std::vector<std::vector<int>> &count_image,
        int min_component_pixels) const;

    std::string events_topic_;
    std::string imu_topic_;
    std::string count_image_topic_;
    std::string time_image_topic_;
    std::string time_image_vis_topic_;
    bool publish_time_image_ = true;
    bool publish_time_image_vis_ = true;
    bool sort_events_by_time_ = true;

    double a_{1.0};
    double b_{0.0};

    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr lambda_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr noise_metrics_pub_;


    rclcpp::Subscription<dv_ros2_msgs::msg::EventArray>::SharedPtr event_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr time_image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr time_image_vis_pub_;

    std::vector<dv_ros2_msgs::msg::Event> event_buffer_;
    std::vector<sensor_msgs::msg::Imu> imu_buffer_;
    std::vector<sensor_msgs::msg::Imu> imu_buffer_snapshot_;

    std_msgs::msg::Header last_event_header_;
    bool first_event_received_ = true;
    bool require_imu_ahead_   = true;
    int64_t imu_window_ns_    = 3000000;

    std::mutex mtx_;

    std::string omega_norm_topic_;

    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr omega_norm_pub_;
    std::string lambda_topic_;
    double time_vis_m_{0.25};
    string events_comp_topic_;
    bool publish_compensated_events_;

    rclcpp::Publisher<dv_ros2_msgs::msg::EventArray>::SharedPtr events_comp_pub_;

    bool prefilter_enable_{true};
    bool prefilter_refractory_enable_{true};
    int64_t prefilter_refractory_us_{600};
    bool prefilter_ba_enable_{true};
    int prefilter_ba_radius_px_{2};
    int64_t prefilter_ba_window_us_{4000};
    int prefilter_ba_min_neighbors_{1};
    bool prefilter_ba_same_polarity_only_{false};
    bool prefilter_ba_support_from_kept_only_{false};
    bool prefilter_hot_pixel_enable_{false};
    int prefilter_hot_pixel_max_events_per_batch_{9};
    bool prefilter_log_stats_{false};

    bool metrics_enable_{true};
    std::string noise_metrics_topic_{"/event_noise_metrics"};
    bool metrics_log_stats_{false};
    int metrics_isolation_radius_px_{1};
    int64_t metrics_isolation_window_us_{2000};
    int metrics_hot_pixel_threshold_{8};
    int metrics_small_component_pixels_{3};
    int metrics_segmentation_min_count_{1};
    double metrics_motion_omega_min_{0.1};

    std::vector<int64_t> last_kept_ts_ns_;
    std::vector<int64_t> last_seen_ts_ns_any_;
    std::vector<int64_t> last_seen_ts_ns_pos_;
    std::vector<int64_t> last_seen_ts_ns_neg_;
    int64_t last_input_event_ts_ns_{kUnsetTimeNs};
    uint64_t prefilter_total_in_{0};
    uint64_t prefilter_total_out_{0};
    uint64_t prefilter_total_drop_refractory_{0};
    uint64_t prefilter_total_drop_ba_{0};
    uint64_t prefilter_total_drop_hot_pixel_{0};
};
