#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <algorithm>
#include <cmath>
#include <mutex>
#include <vector>

#include <dvs_msgs/msg/event.hpp>
#include <dvs_msgs/msg/event_array.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>

class EventVisualizer : public rclcpp::Node {
public:
    EventVisualizer() : Node("datasync_node") {
        this->declare_parameter("weight_param", 346);
        this->declare_parameter("height_param", 260);
        this->declare_parameter("focus", 6550.0);
        this->declare_parameter("pixel_size", 18.5);

        weight_ = this->get_parameter("weight_param").as_int();
        height_ = this->get_parameter("height_param").as_int();
        focus_ = this->get_parameter("focus").as_double();
        pixel_size_ = this->get_parameter("pixel_size").as_double();

        auto event_qos = rclcpp::QoS(rclcpp::KeepLast(1));
        auto imu_qos = rclcpp::QoS(rclcpp::KeepLast(7));

        event_sub_ = this->create_subscription<dvs_msgs::msg::EventArray>(
            "/dvs/events",
            event_qos,
            std::bind(&EventVisualizer::event_cb, this, std::placeholders::_1));
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/dvs/imu",
            imu_qos,
            std::bind(&EventVisualizer::imu_cb, this, std::placeholders::_1));

        image_pub_ = image_transport::create_publisher(this, "/count_image");
    }

private:
    std::vector<dvs_msgs::msg::Event> event_buffer_;
    std::vector<sensor_msgs::msg::Imu> imu_buffer_;
    std::vector<sensor_msgs::msg::Imu> imu_buffer_copy_;
    std::mutex mtx_;
    bool first_event_received_ = true;

    int height_ = 0;
    int weight_ = 0;
    double focus_ = 0.0;
    double pixel_size_ = 0.0;

    rclcpp::Subscription<dvs_msgs::msg::EventArray>::SharedPtr event_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    image_transport::Publisher image_pub_;

    void imu_cb(const sensor_msgs::msg::Imu::SharedPtr imu) {
        if (!first_event_received_) {
            std::lock_guard<std::mutex> lock(mtx_);
            imu_buffer_.emplace_back(*imu);
        }
    }

    void event_cb(const dvs_msgs::msg::EventArray::SharedPtr msg) {
        if (!first_event_received_) {
            {
                std::lock_guard<std::mutex> lock(mtx_);
                imu_buffer_copy_ = imu_buffer_;
                if (!imu_buffer_.empty()) {
                    imu_buffer_.clear();
                }
            }

            if (imu_buffer_copy_.empty()) {
                return;
            }

            for (size_t i = 0; i < msg->events.size(); ++i) {
                event_buffer_.emplace_back(msg->events[i]);
            }

            data_process();
        } else {
            first_event_received_ = false;
            if (!imu_buffer_.empty()) {
                imu_buffer_.clear();
            }
            RCLCPP_INFO(this->get_logger(), "Data aligned!");
            RCLCPP_INFO(this->get_logger(), "Start processing data...");
        }
    }

    void show_count_image(std::vector<std::vector<int>> &count_image, int max_count) {
        if (max_count <= 0) {
            return;
        }
        cv::Mat image(height_, weight_, CV_8UC1);
        int scale = static_cast<int>(255 / max_count) + 1;
        for (int i = 0; i < height_; ++i) {
            for (int j = 0; j < weight_; ++j) {
                image.at<uchar>(i, j) = static_cast<uchar>(count_image[i][j] * scale);
            }
        }

        sensor_msgs::msg::Image::SharedPtr msg =
            cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", image).toImageMsg();
        image_pub_.publish(msg);
    }

    void data_process() {
        if (event_buffer_.empty() || imu_buffer_copy_.empty()) {
            return;
        }

        const int64_t first_event_ts = rclcpp::Time(event_buffer_.front().ts).nanoseconds();
        const int64_t last_imu_ts = rclcpp::Time(imu_buffer_copy_.back().header.stamp).nanoseconds();
        if (last_imu_ts <= first_event_ts) {
            event_buffer_.clear();
            imu_buffer_copy_.clear();
            return;
        }

        float angular_velocity_x = 0.0f;
        float angular_velocity_y = 0.0f;
        float angular_velocity_z = 0.0f;
        int cnt = 0;
        for (size_t i = 0; i < imu_buffer_copy_.size(); ++i) {
            const int64_t imu_ts = rclcpp::Time(imu_buffer_copy_[i].header.stamp).nanoseconds();
            if (imu_ts >= (first_event_ts - 3000000)) {
                angular_velocity_x += static_cast<float>(imu_buffer_copy_[i].angular_velocity.x);
                angular_velocity_y += static_cast<float>(imu_buffer_copy_[i].angular_velocity.y);
                angular_velocity_z += static_cast<float>(imu_buffer_copy_[i].angular_velocity.z);
                cnt++;
            }
        }
        if (cnt == 0) {
            event_buffer_.clear();
            imu_buffer_copy_.clear();
            return;
        }

        const float average_angular_rate_x = angular_velocity_x / static_cast<float>(cnt);
        const float average_angular_rate_y = angular_velocity_y / static_cast<float>(cnt);
        const float average_angular_rate_z = angular_velocity_z / static_cast<float>(cnt);
        const float average_angular_rate = std::sqrt(
            (average_angular_rate_x * average_angular_rate_x) +
            (average_angular_rate_y * average_angular_rate_y) +
            (average_angular_rate_z * average_angular_rate_z));
        (void)average_angular_rate;

        const int64_t t0 = rclcpp::Time(event_buffer_.front().ts).nanoseconds();
        std::vector<std::vector<int>> count_image(height_, std::vector<int>(weight_, 0));
        std::vector<std::vector<float>> time_image(height_, std::vector<float>(weight_, 0.0f));

        for (size_t i = 0; i < event_buffer_.size(); ++i) {
            const int64_t ts = rclcpp::Time(event_buffer_[i].ts).nanoseconds();
            const float time_diff = static_cast<float>(ts - t0) / 1000000000.0f;

            const float x_angular = time_diff * average_angular_rate_x;
            const float y_angular = time_diff * average_angular_rate_y;
            const float z_angular = time_diff * average_angular_rate_z;

            const int x = static_cast<int>(event_buffer_[i].x) - weight_ / 2;
            const int y = static_cast<int>(event_buffer_[i].y) - height_ / 2;

            const float pre_x_angle = std::atan(y * pixel_size_ / focus_);
            const float pre_y_angle = std::atan(x * pixel_size_ / focus_);

            const int compen_x = static_cast<int>(
                (x * std::cos(z_angular) - std::sin(z_angular) * y) -
                (x - (focus_ * std::tan(pre_y_angle + y_angular) / pixel_size_)) +
                weight_ / 2);
            const int compen_y = static_cast<int>(
                (x * std::sin(z_angular) + std::cos(z_angular) * y) -
                (y - (focus_ * std::tan(pre_x_angle - x_angular) / pixel_size_)) +
                height_ / 2);

            event_buffer_[i].x = static_cast<uint16_t>(compen_x);
            event_buffer_[i].y = static_cast<uint16_t>(compen_y);

            if (compen_y < height_ && compen_y >= 0 && compen_x < weight_ && compen_x >= 0) {
                if (count_image[compen_y][compen_x] < 20) {
                    count_image[compen_y][compen_x]++;
                }
                time_image[compen_y][compen_x] += time_diff;
            }
        }

        int max_count = 0;
        float max_time = 0.0f;
        float total_time = 0.0f;
        float average_time = 0.0f;
        int trigger_pixels = 0;

        for (int i = 0; i < height_; ++i) {
            for (int j = 0; j < weight_; ++j) {
                if (count_image[i][j] != 0) {
                    time_image[i][j] /= count_image[i][j];
                    max_count = std::max(max_count, count_image[i][j]);
                    max_time = std::max(max_time, time_image[i][j]);
                    total_time += time_image[i][j];
                    trigger_pixels++;
                }
            }
        }

        if (trigger_pixels > 0) {
            average_time = total_time / trigger_pixels;
        }
        (void)max_time;
        (void)average_time;

        show_count_image(count_image, max_count);

        event_buffer_.clear();
        imu_buffer_copy_.clear();
    }
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
