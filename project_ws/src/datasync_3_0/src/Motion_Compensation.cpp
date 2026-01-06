#include <algorithm>
#include <cmath>
#include <cstdint>
#include <mutex>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include <dv_ros2_msgs/msg/event.hpp>
#include <dv_ros2_msgs/msg/event_array.hpp>

using sll = long long int;

namespace {
int height_param = 260;
int width_param = 346;
float focus_param = 6550.0f;
float pixel_size_param = 18.5f;

inline int64_t toNs(const builtin_interfaces::msg::Time &t) {
    return static_cast<int64_t>(t.sec) * 1000000000LL + static_cast<int64_t>(t.nanosec);
}
}  // namespace

class EventVisualizer : public rclcpp::Node {
public:
    EventVisualizer() : Node("datasync_node") {
        declare_parameter<int>("weight_param", width_param);
        declare_parameter<int>("height_param", height_param);
        declare_parameter<double>("focus", focus_param);
        declare_parameter<double>("pixel_size", pixel_size_param);
        declare_parameter<bool>("require_imu_ahead", true);
        declare_parameter<std::string>("events_topic", "events");
        declare_parameter<std::string>("imu_topic", "imu");
        declare_parameter<std::string>("count_image_topic", "count_image");

        get_parameter("weight_param", width_param);
        get_parameter("height_param", height_param);
        get_parameter("focus", focus_param);
        get_parameter("pixel_size", pixel_size_param);
        get_parameter("require_imu_ahead", require_imu_ahead_);
        get_parameter("events_topic", events_topic_);
        get_parameter("imu_topic", imu_topic_);
        get_parameter("count_image_topic", count_image_topic_);

        auto qos = rclcpp::SensorDataQoS();
        event_sub_ = create_subscription<dv_ros2_msgs::msg::EventArray>(
            events_topic_, qos,
            std::bind(&EventVisualizer::event_cb, this, std::placeholders::_1));
        imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
            imu_topic_, qos,
            std::bind(&EventVisualizer::imu_cb, this, std::placeholders::_1));
        image_pub_ = create_publisher<sensor_msgs::msg::Image>(count_image_topic_, 1);

        RCLCPP_INFO(get_logger(), "Motion compensation node ready.");
    }

private:
    void show_count_image(const std::vector<std::vector<int>> &count_image, int max_count) {
        if (max_count <= 0) {
            return;
        }
        cv::Mat image(height_param, width_param, CV_8UC1);
        int scale = (255 / max_count) + 1;
        for (int i = 0; i < height_param; ++i) {
            for (int j = 0; j < width_param; ++j) {
                image.at<uchar>(i, j) = static_cast<uchar>(count_image[i][j] * scale);
            }
        }

        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", image).toImageMsg();
        image_pub_->publish(*msg);
    }

    void data_process() {
        if (imu_buffer_snapshot_.empty() || event_buffer_.empty()) {
            return;
        }

        const int64_t imu_last_ns = toNs(imu_buffer_snapshot_.back().header.stamp);
        const int64_t event_first_ns = toNs(event_buffer_.front().ts);
        if (require_imu_ahead_ && imu_last_ns <= event_first_ns) {
            event_buffer_.clear();
            imu_buffer_snapshot_.clear();
            return;
        }

        float angular_velocity_x = 0.0f;
        float angular_velocity_y = 0.0f;
        float angular_velocity_z = 0.0f;
        int cnt = 0;

        for (const auto &imu : imu_buffer_snapshot_) {
            const int64_t imu_ns = toNs(imu.header.stamp);
            if (imu_ns >= (event_first_ns - 3000000LL)) {
                angular_velocity_x += static_cast<float>(imu.angular_velocity.x);
                angular_velocity_y += static_cast<float>(imu.angular_velocity.y);
                angular_velocity_z += static_cast<float>(imu.angular_velocity.z);
                cnt++;
            }
        }

        if (cnt == 0) {
            event_buffer_.clear();
            imu_buffer_snapshot_.clear();
            return;
        }

        float average_angular_rate_x = angular_velocity_x / static_cast<float>(cnt);
        float average_angular_rate_y = angular_velocity_y / static_cast<float>(cnt);
        float average_angular_rate_z = angular_velocity_z / static_cast<float>(cnt);

        const int64_t t0 = event_first_ns;
        std::vector<std::vector<int>> count_image(height_param, std::vector<int>(width_param));
        std::vector<std::vector<float>> time_image(height_param, std::vector<float>(width_param));

        for (auto &evt : event_buffer_) {
            float time_diff = static_cast<float>(static_cast<double>(toNs(evt.ts) - t0) / 1000000000.0);

            float x_angular = time_diff * average_angular_rate_x;
            float y_angular = time_diff * average_angular_rate_y;
            float z_angular = time_diff * average_angular_rate_z;

            int x = static_cast<int>(evt.x) - width_param / 2;
            int y = static_cast<int>(evt.y) - height_param / 2;

            float pre_x_angle = std::atan(static_cast<float>(y) * pixel_size_param / focus_param);
            float pre_y_angle = std::atan(static_cast<float>(x) * pixel_size_param / focus_param);

            int compen_x = static_cast<int>((x * std::cos(z_angular) - std::sin(z_angular) * y)
                                            - (x - (focus_param * std::tan(pre_y_angle + y_angular) / pixel_size_param))
                                            + width_param / 2);
            int compen_y = static_cast<int>((x * std::sin(z_angular) + std::cos(z_angular) * y)
                                            - (y - (focus_param * std::tan(pre_x_angle - x_angular) / pixel_size_param))
                                            + height_param / 2);

            evt.x = static_cast<uint16_t>(compen_x);
            evt.y = static_cast<uint16_t>(compen_y);

            if (compen_y < height_param && compen_y >= 0 && compen_x < width_param && compen_x >= 0) {
                if (count_image[compen_y][compen_x] < 20) {
                    count_image[compen_y][compen_x]++;
                }
                time_image[compen_y][compen_x] += time_diff;
            }
        }

        int max_count = 0;
        for (int i = 0; i < height_param; ++i) {
            for (int j = 0; j < width_param; ++j) {
                if (count_image[i][j] != 0) {
                    time_image[i][j] /= static_cast<float>(count_image[i][j]);
                    max_count = std::max(max_count, count_image[i][j]);
                }
            }
        }

        show_count_image(count_image, max_count);

        event_buffer_.clear();
        imu_buffer_snapshot_.clear();
    }

    void event_cb(const dv_ros2_msgs::msg::EventArray::SharedPtr msg) {
        if (!first_event_received_) {
            {
                std::lock_guard<std::mutex> lock(mtx_);
                imu_buffer_snapshot_ = imu_buffer_;
                if (!imu_buffer_.empty()) {
                    imu_buffer_.clear();
                }
            }

            if (imu_buffer_snapshot_.empty()) {
                return;
            }

            event_buffer_.reserve(event_buffer_.size() + msg->events.size());
            for (const auto &evt : msg->events) {
                event_buffer_.emplace_back(evt);
            }

            data_process();
        } else {
            first_event_received_ = false;
            if (!imu_buffer_.empty()) {
                imu_buffer_.clear();
            }
            RCLCPP_INFO(get_logger(), "Data aligned! Start processing data...");
        }
    }

    void imu_cb(const sensor_msgs::msg::Imu::SharedPtr imu) {
        if (!first_event_received_) {
            imu_buffer_.emplace_back(*imu);
        }
    }

    std::string events_topic_;
    std::string imu_topic_;
    std::string count_image_topic_;

    rclcpp::Subscription<dv_ros2_msgs::msg::EventArray>::SharedPtr event_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;

    std::vector<dv_ros2_msgs::msg::Event> event_buffer_;
    std::vector<sensor_msgs::msg::Imu> imu_buffer_;
    std::vector<sensor_msgs::msg::Imu> imu_buffer_snapshot_;
    bool first_event_received_ = true;
    bool require_imu_ahead_ = true;
    std::mutex mtx_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<EventVisualizer>();
    rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 3);
    exec.add_node(node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}
