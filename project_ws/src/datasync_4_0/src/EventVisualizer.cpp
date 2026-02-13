// ======================
// Includes standards C++
// ======================
#include <algorithm>
#include <cmath>
#include <cstdint>

// ======================
// OpenCV + cv_bridge
// ======================
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <sensor_msgs/image_encodings.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "datasync_4_0/event_visualizer.hpp"

// ============================================================================
// Namespace anonyme : parametres globaux et fonctions utilitaires internes
// ============================================================================
namespace {

// Resolution du capteur evenementiel (DAVIS 346)
int height_param = 260;
int width_param  = 346;

// Parametres intrinseques camera
float focus_param      = 6550.0f;
float pixel_size_param = 18.5f;

}  // namespace

int64_t EventVisualizer::toNs(const builtin_interfaces::msg::Time &t) {
    return static_cast<int64_t>(t.sec) * 1000000000LL +
           static_cast<int64_t>(t.nanosec);
}

// ============================================================================
// Constructeur du noeud
// ============================================================================
EventVisualizer::EventVisualizer()
    : Node("datasync_4_0_node"),
      tf_buffer_(this->get_clock()) {

    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(tf_buffer_);

    // ----------------------
    // Declaration parametres ROS
    // ----------------------
    declare_parameter<int>("weight_param", width_param);
    declare_parameter<int>("height_param", height_param);
    declare_parameter<double>("focus", focus_param);
    declare_parameter<double>("pixel_size", pixel_size_param);

    declare_parameter<bool>("require_imu_ahead", false);
    declare_parameter<int64_t>("imu_window_ns", 3000000);

    declare_parameter<std::string>("events_topic", "events");
    declare_parameter<std::string>("imu_topic", "imu");
    declare_parameter<std::string>("depth_topic", "depth");
    declare_parameter<std::string>("camera_info_topic", "camera_info");
    declare_parameter<std::string>("odom_topic", "odom");

    declare_parameter<std::string>("count_image_topic", "count_image");
    declare_parameter<std::string>("time_image_topic", "time_image");
    declare_parameter<std::string>("time_image_vis_topic", "time_image_vis");
    declare_parameter<bool>("publish_time_image", true);
    declare_parameter<bool>("publish_time_image_vis", true);

    declare_parameter<bool>("sort_events_by_time", false);
    declare_parameter<bool>("require_depth", true);
    declare_parameter<bool>("require_odom", true);
    declare_parameter<bool>("use_imu_rotation", true);
    declare_parameter<double>("max_depth_age_ms", 50.0);
    declare_parameter<double>("depth_scale", 0.001);
    declare_parameter<int64_t>("odom_buffer_ns", 2000000000);
    declare_parameter<std::string>("base_frame", "base_link");
    declare_parameter<std::string>("camera_frame", "camera");

    // ----------------------
    // Lecture parametres
    // ----------------------
    get_parameter("weight_param", width_param);
    get_parameter("height_param", height_param);
    get_parameter("focus", focus_param);
    get_parameter("pixel_size", pixel_size_param);
    get_parameter("require_imu_ahead", require_imu_ahead_);
    get_parameter("imu_window_ns", imu_window_ns_);

    get_parameter("events_topic", events_topic_);
    get_parameter("imu_topic", imu_topic_);
    get_parameter("depth_topic", depth_topic_);
    get_parameter("camera_info_topic", camera_info_topic_);
    get_parameter("odom_topic", odom_topic_);

    get_parameter("count_image_topic", count_image_topic_);
    get_parameter("time_image_topic", time_image_topic_);
    get_parameter("time_image_vis_topic", time_image_vis_topic_);
    get_parameter("publish_time_image", publish_time_image_);
    get_parameter("publish_time_image_vis", publish_time_image_vis_);

    get_parameter("sort_events_by_time", sort_events_by_time_);
    get_parameter("require_depth", require_depth_);
    get_parameter("require_odom", require_odom_);
    get_parameter("use_imu_rotation", use_imu_rotation_);
    get_parameter("max_depth_age_ms", max_depth_age_ms_);
    get_parameter("depth_scale", depth_scale_);
    get_parameter("odom_buffer_ns", odom_buffer_ns_);
    get_parameter("base_frame", base_frame_);
    get_parameter("camera_frame", camera_frame_);

    // Fallback intrinsics from focus/pixel size if CameraInfo missing
    fx_ = focus_param / pixel_size_param;
    fy_ = fx_;
    cx_ = static_cast<double>(width_param) / 2.0;
    cy_ = static_cast<double>(height_param) / 2.0;
    has_intrinsics_ = true;

    // ----------------------
    // QoS capteurs standard ROS
    // ----------------------
    auto qos = rclcpp::SensorDataQoS();

    // ----------------------
    // Souscriptions
    // ----------------------
    event_sub_ = create_subscription<dv_ros2_msgs::msg::EventArray>(
        events_topic_, qos,
        std::bind(&EventVisualizer::event_cb, this, std::placeholders::_1));

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        imu_topic_, qos,
        std::bind(&EventVisualizer::imu_cb, this, std::placeholders::_1));

    depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
        depth_topic_, qos,
        std::bind(&EventVisualizer::depth_cb, this, std::placeholders::_1));

    camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        camera_info_topic_, qos,
        std::bind(&EventVisualizer::camera_info_cb, this, std::placeholders::_1));

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, qos,
        std::bind(&EventVisualizer::odom_cb, this, std::placeholders::_1));

    // ----------------------
    // Publishers
    // ----------------------
    image_pub_ = create_publisher<sensor_msgs::msg::Image>(
        count_image_topic_, 1);

    if (publish_time_image_) {
        time_image_pub_ = create_publisher<sensor_msgs::msg::Image>(
            time_image_topic_, 1);
    }
    if (publish_time_image_vis_) {
        time_image_vis_pub_ = create_publisher<sensor_msgs::msg::Image>(
            time_image_vis_topic_, 1);
    }

    RCLCPP_INFO(get_logger(), "datasync_4_0 node ready.");
}

// ============================================================================
// Affiche une image de comptage (nombre d'evenements par pixel)
// ============================================================================
void EventVisualizer::show_count_image(
    const std::vector<std::vector<int>> &count_image,
    int max_count,
    const std_msgs::msg::Header &header,
    const rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr &pub) {
    if (max_count <= 0 || !pub) {
        return;
    }

    cv::Mat image(height_param, width_param, CV_8UC1);
    int scale = (255 / max_count) + 1;

    for (int i = 0; i < height_param; ++i) {
        for (int j = 0; j < width_param; ++j) {
            image.at<uchar>(i, j) =
                static_cast<uchar>(count_image[i][j] * scale);
        }
    }

    auto msg = cv_bridge::CvImage(header, "mono8", image).toImageMsg();
    pub->publish(*msg);
}

// ============================================================================
// Depth helpers
// ============================================================================
bool EventVisualizer::get_depth_meters(
    int u, int v, const sensor_msgs::msg::Image &depth, double &depth_m) const {
    if (u < 0 || v < 0 || u >= static_cast<int>(depth.width) ||
        v >= static_cast<int>(depth.height)) {
        return false;
    }

    if (depth.encoding == sensor_msgs::image_encodings::TYPE_32FC1 ||
        depth.encoding == "32FC1") {
        const auto *row = reinterpret_cast<const float *>(
            &depth.data[v * depth.step]);
        float value = row[u];
        if (!std::isfinite(value) || value <= 0.0f) {
            return false;
        }
        depth_m = static_cast<double>(value);
        return true;
    }

    if (depth.encoding == sensor_msgs::image_encodings::TYPE_16UC1 ||
        depth.encoding == "16UC1") {
        const auto *row = reinterpret_cast<const uint16_t *>(
            &depth.data[v * depth.step]);
        uint16_t value = row[u];
        if (value == 0) {
            return false;
        }
        depth_m = static_cast<double>(value) * depth_scale_;
        return true;
    }

    return false;
}

// ============================================================================
// Odom helpers (interpolation)
// ============================================================================
bool EventVisualizer::get_odom_at(
    const rclcpp::Time &stamp,
    tf2::Vector3 &position,
    tf2::Quaternion &orientation) const {
    if (odom_buffer_snapshot_.size() < 2) {
        return false;
    }

    const int64_t target_ns = stamp.nanoseconds();
    const int64_t first_ns = toNs(odom_buffer_snapshot_.front().header.stamp);
    const int64_t last_ns = toNs(odom_buffer_snapshot_.back().header.stamp);

    if (target_ns < first_ns || target_ns > last_ns) {
        return false;
    }

    for (size_t i = 1; i < odom_buffer_snapshot_.size(); ++i) {
        const auto &prev = odom_buffer_snapshot_[i - 1];
        const auto &next = odom_buffer_snapshot_[i];

        const int64_t prev_ns = toNs(prev.header.stamp);
        const int64_t next_ns = toNs(next.header.stamp);
        if (target_ns <= next_ns) {
            const double span = static_cast<double>(next_ns - prev_ns);
            const double ratio = span > 0.0
                ? static_cast<double>(target_ns - prev_ns) / span
                : 0.0;

            tf2::Vector3 p0(prev.pose.pose.position.x,
                            prev.pose.pose.position.y,
                            prev.pose.pose.position.z);
            tf2::Vector3 p1(next.pose.pose.position.x,
                            next.pose.pose.position.y,
                            next.pose.pose.position.z);

            tf2::Quaternion q0, q1;
            tf2::fromMsg(prev.pose.pose.orientation, q0);
            tf2::fromMsg(next.pose.pose.orientation, q1);

            position = p0.lerp(p1, ratio);
            orientation = q0.slerp(q1, ratio);
            return true;
        }
    }

    return false;
}

bool EventVisualizer::get_extrinsic_base_to_camera(tf2::Transform &T_b_c) {
    if (have_extrinsic_) {
        T_b_c = T_b_c_;
        return true;
    }

    try {
        auto tf_msg = tf_buffer_.lookupTransform(
            base_frame_, camera_frame_, tf2::TimePointZero);
        tf2::Quaternion q;
        tf2::fromMsg(tf_msg.transform.rotation, q);
        tf2::Vector3 t(tf_msg.transform.translation.x,
                       tf_msg.transform.translation.y,
                       tf_msg.transform.translation.z);
        T_b_c = tf2::Transform(q, t);
        T_b_c_ = T_b_c;
        have_extrinsic_ = true;
        return true;
    } catch (const tf2::TransformException &ex) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "TF base->camera unavailable: %s", ex.what());
        return false;
    }
}

// ============================================================================
// Traitement principal
// ============================================================================
void EventVisualizer::data_process() {
    // Verifie qu'on a des donnees
    if (imu_buffer_snapshot_.empty() || event_buffer_.empty()) {
        return;
    }

    const int64_t event_first_ns = toNs(event_buffer_.front().ts);
    const int64_t imu_last_ns = toNs(imu_buffer_snapshot_.back().header.stamp);

    // On ne traite les données que si le message imu le plus récent est plus récent que le premier évènement
    if (imu_last_ns <= event_first_ns) {
        event_buffer_.clear();
        imu_buffer_snapshot_.clear();
        return;
    }
    
    sensor_msgs::msg::Image::SharedPtr depth_msg;
    {
        std::lock_guard<std::mutex> lock(depth_mtx_);
        depth_msg = depth_image_;
    }

    if (require_depth_ && (!depth_msg || !has_intrinsics_)) {
        event_buffer_.clear();
        imu_buffer_snapshot_.clear();
        return;
    }

    const int64_t t_ref_ns = toNs(event_buffer_.back().ts);
    rclcpp::Time t_ref(t_ref_ns, this->get_clock()->get_clock_type());

    if (depth_msg) {
        const double depth_age_ms =
            std::abs(static_cast<double>(t_ref_ns - toNs(depth_msg->header.stamp))) / 1e6;
        if (depth_age_ms > max_depth_age_ms_) {
            if (require_depth_) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                     "Depth too old (%.2f ms)", depth_age_ms);
                event_buffer_.clear();
                imu_buffer_snapshot_.clear();
                return;
            }
        }
    }

    tf2::Transform T_b_c;
    if (require_odom_ && !get_extrinsic_base_to_camera(T_b_c)) {
        event_buffer_.clear();
        imu_buffer_snapshot_.clear();
        return;
    }

    tf2::Transform T_w_c_ref;
    tf2::Vector3 cam_pos_ref;
    tf2::Vector3 cam_z_axis_w(0.0, 0.0, 1.0);

    if (require_odom_) {
        tf2::Vector3 pos_ref;
        tf2::Quaternion ori_ref;
        if (!get_odom_at(t_ref, pos_ref, ori_ref)) {
            event_buffer_.clear();
            imu_buffer_snapshot_.clear();
            return;
        }

        tf2::Transform T_w_b_ref(ori_ref, pos_ref);
        T_w_c_ref = T_w_b_ref * T_b_c;
        cam_pos_ref = T_w_c_ref.getOrigin();
        cam_z_axis_w = T_w_c_ref.getBasis() * tf2::Vector3(0.0, 0.0, 1.0);
    }

    // Moyenne IMU pour rotation
    float avg_wx = 0.0f;
    float avg_wy = 0.0f;
    float avg_wz = 0.0f;
    int imu_cnt = 0;
    if (use_imu_rotation_ && !imu_buffer_snapshot_.empty()) {
        const int64_t imu_min_ns = t_ref_ns - imu_window_ns_;
        for (const auto &imu : imu_buffer_snapshot_) {
            const int64_t imu_ns = toNs(imu.header.stamp);
            if (imu_ns >= imu_min_ns && imu_ns <= t_ref_ns) {
                avg_wx += static_cast<float>(imu.angular_velocity.x);
                avg_wy += static_cast<float>(imu.angular_velocity.y);
                avg_wz += static_cast<float>(imu.angular_velocity.z);
                imu_cnt++;
            }
        }
        if (imu_cnt > 0) {
            avg_wx /= imu_cnt;
            avg_wy /= imu_cnt;
            avg_wz /= imu_cnt;
        } else {
            avg_wx = avg_wy = avg_wz = 0.0f;
        }
    }

    std::vector<std::vector<int>> count_image(
        height_param, std::vector<int>(width_param));
    std::vector<std::vector<float>> time_image(
        height_param, std::vector<float>(width_param));

    if (sort_events_by_time_) {
        std::sort(event_buffer_.begin(), event_buffer_.end(),
                  [](const auto &a, const auto &b) {
                      return toNs(a.ts) < toNs(b.ts);
                  });
    }

    for (const auto &evt : event_buffer_) {
        const int64_t evt_ns = toNs(evt.ts);
        const float dt = static_cast<float>((evt_ns - t_ref_ns) / 1e9);

        if (!depth_msg || !has_intrinsics_) {
            continue;
        }

        double depth_m = 0.0;
        if (!get_depth_meters(static_cast<int>(evt.x), static_cast<int>(evt.y),
                              *depth_msg, depth_m)) {
            continue;
        }

        const double Xc = (static_cast<double>(evt.x) - cx_) * depth_m / fx_;
        const double Yc = (static_cast<double>(evt.y) - cy_) * depth_m / fy_;
        double Zc = depth_m;

        if (require_odom_) {
            tf2::Vector3 pos_evt;
            tf2::Quaternion ori_evt;
            rclcpp::Time evt_time(evt_ns, this->get_clock()->get_clock_type());
            if (!get_odom_at(evt_time, pos_evt, ori_evt)) {
                continue;
            }
            tf2::Transform T_w_b_evt(ori_evt, pos_evt);
            tf2::Transform T_w_c_evt = T_w_b_evt * T_b_c;
            tf2::Vector3 cam_pos_evt = T_w_c_evt.getOrigin();
            tf2::Vector3 delta_world = cam_pos_evt - cam_pos_ref;
            const double delta_z = delta_world.dot(cam_z_axis_w);
            Zc += delta_z;
        }

        if (Zc <= 0.0) {
            continue;
        }

        tf2::Vector3 Pc(Xc, Yc, Zc);
        if (use_imu_rotation_ && imu_cnt > 0) {
            tf2::Quaternion q;
            q.setRPY(avg_wx * dt, avg_wy * dt, avg_wz * dt);
            Pc = tf2::quatRotate(q.inverse(), Pc);
        }

        if (Pc.z() <= 0.0) {
            continue;
        }

        const double u = fx_ * Pc.x() / Pc.z() + cx_;
        const double v = fy_ * Pc.y() / Pc.z() + cy_;

        const int ui = static_cast<int>(std::round(u));
        const int vi = static_cast<int>(std::round(v));

        if (vi >= 0 && vi < height_param && ui >= 0 && ui < width_param) {
            if (count_image[vi][ui] < 20) {
                count_image[vi][ui]++;
            }
            time_image[vi][ui] += std::abs(dt);
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

    show_count_image(count_image, max_count, last_event_header_, image_pub_);

    if (publish_time_image_ && time_image_pub_) {
        cv::Mat time_mat(height_param, width_param, CV_32FC1);
        for (int i = 0; i < height_param; ++i) {
            for (int j = 0; j < width_param; ++j) {
                time_mat.at<float>(i, j) = time_image[i][j];
            }
        }
        auto time_msg = cv_bridge::CvImage(
                            last_event_header_, "32FC1", time_mat)
                            .toImageMsg();
        time_image_pub_->publish(*time_msg);
    }

    if (publish_time_image_vis_ && time_image_vis_pub_) {
        float max_time = 0.0f;
        for (int i = 0; i < height_param; ++i) {
            for (int j = 0; j < width_param; ++j) {
                if (time_image[i][j] > max_time) {
                    max_time = time_image[i][j];
                }
            }
        }

        cv::Mat time_vis(height_param, width_param, CV_8UC1, cv::Scalar(0));
        if (max_time > 0.0f) {
            const float scale = 255.0f / max_time;
            for (int i = 0; i < height_param; ++i) {
                for (int j = 0; j < width_param; ++j) {
                    time_vis.at<uchar>(i, j) = static_cast<uchar>(
                        std::min(255.0f, time_image[i][j] * scale));
                }
            }
        }

        auto vis_msg = cv_bridge::CvImage(
                           last_event_header_, "mono8", time_vis)
                           .toImageMsg();
        time_image_vis_pub_->publish(*vis_msg);
    }

    event_buffer_.clear();
    imu_buffer_snapshot_.clear();
    odom_buffer_snapshot_.clear();
}

// ========================================================================
// Callback evenements
// ========================================================================
void EventVisualizer::event_cb(
    const dv_ros2_msgs::msg::EventArray::SharedPtr msg) {
    if (!first_event_received_) {
        {
            std::lock_guard<std::mutex> lock(mtx_);
            imu_buffer_snapshot_ = imu_buffer_;
            imu_buffer_.clear();
        }
        {
            std::lock_guard<std::mutex> lock(odom_mtx_);
            odom_buffer_snapshot_ = odom_buffer_;
        }

        if (require_odom_ && odom_buffer_snapshot_.empty()) {
            return;
        }

        last_event_header_ = msg->header;
        event_buffer_.reserve(event_buffer_.size() + msg->events.size());
        for (const auto &evt : msg->events) {
            event_buffer_.emplace_back(evt);
        }

        data_process();
    } else {
        first_event_received_ = false;
        {
            std::lock_guard<std::mutex> lock(mtx_);
            imu_buffer_.clear();
        }
        RCLCPP_INFO(get_logger(), "Data aligned! Start processing data...");
    }
}

// ========================================================================
// Callback IMU
// ========================================================================
void EventVisualizer::imu_cb(const sensor_msgs::msg::Imu::SharedPtr imu) {
    if (!first_event_received_) {
        std::lock_guard<std::mutex> lock(mtx_);
        imu_buffer_.emplace_back(*imu);
    }
}

// ========================================================================
// Callback Depth
// ========================================================================
void EventVisualizer::depth_cb(const sensor_msgs::msg::Image::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(depth_mtx_);
    depth_image_ = msg;
}

// ========================================================================
// Callback CameraInfo
// ========================================================================
void EventVisualizer::camera_info_cb(
    const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(depth_mtx_);
    camera_info_ = msg;
    fx_ = msg->k[0];
    fy_ = msg->k[4];
    cx_ = msg->k[2];
    cy_ = msg->k[5];
    has_intrinsics_ = (fx_ > 0.0 && fy_ > 0.0);
}

// ========================================================================
// Callback Odom
// ========================================================================
void EventVisualizer::odom_cb(const nav_msgs::msg::Odometry::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(odom_mtx_);
    odom_buffer_.emplace_back(*msg);

    const int64_t newest_ns = toNs(msg->header.stamp);
    while (!odom_buffer_.empty()) {
        const int64_t oldest_ns = toNs(odom_buffer_.front().header.stamp);
        if (newest_ns - oldest_ns > odom_buffer_ns_) {
            odom_buffer_.pop_front();
        } else {
            break;
        }
    }
}
