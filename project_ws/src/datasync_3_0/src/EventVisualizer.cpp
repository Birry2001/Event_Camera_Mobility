// ======================
// Includes standards C++
// ======================
#include <algorithm>   // std::max, std::min
#include <cmath>       // trigonometrie : sin, cos, atan, tan
#include <cstdint>     // types entiers fixes (int64_t, uint16_t)

// ======================
// OpenCV + cv_bridge
// ======================
#include <cv_bridge/cv_bridge.h>     // Conversion OpenCV <-> ROS Image
#include <opencv2/core.hpp>          // cv::Mat
#include <opencv2/imgproc.hpp>       // traitements d'image

#include "datasync_3_0/event_visualizer.hpp"

// ============================================================================
// Namespace anonyme : parametres globaux et fonctions utilitaires internes
// ============================================================================
namespace {

// Resolution du capteur evenementiel (DAVIS 346)
int height_param = 260;
int width_param  = 346;

// Parametres intrinseques camera
float focus_param       = 6550.0f;  // focale (en um ou unites equivalentes)
float pixel_size_param = 18.5f;     // taille pixel (um)

}  // namespace

int64_t EventVisualizer::toNs(const builtin_interfaces::msg::Time &t) {
    return static_cast<int64_t>(t.sec) * 1000000000LL +
           static_cast<int64_t>(t.nanosec);
}

// ============================================================================
// Constructeur du noeud
// ============================================================================
EventVisualizer::EventVisualizer() : Node("datasync_node") {

    // ----------------------
    // Declaration parametres ROS
    // ----------------------
    declare_parameter<int>("weight_param", width_param);
    declare_parameter<int>("height_param", height_param);
    declare_parameter<double>("focus", focus_param);
    declare_parameter<double>("pixel_size", pixel_size_param);
    declare_parameter<bool>("require_imu_ahead", true);
    declare_parameter<int64_t>("imu_window_ns", 3000000);

    declare_parameter<std::string>("events_topic", "events");
    declare_parameter<std::string>("imu_topic", "imu");
    declare_parameter<std::string>("count_image_topic", "count_image");
    declare_parameter<bool>("publish_raw_count", false);
    declare_parameter<bool>("publish_comp_count", false);
    declare_parameter<std::string>("raw_count_topic", "count_image_raw");
    declare_parameter<std::string>("comp_count_topic", "count_image_comp");
    declare_parameter<std::string>("time_image_topic", "time_image");
    declare_parameter<std::string>("time_image_vis_topic", "time_image_vis");
    declare_parameter<bool>("publish_time_image", true);
    declare_parameter<bool>("publish_time_image_vis", true);
    declare_parameter<bool>("sort_events_by_time", true);

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
    get_parameter("count_image_topic", count_image_topic_);
    get_parameter("publish_raw_count", publish_raw_count_);
    get_parameter("publish_comp_count", publish_comp_count_);
    get_parameter("raw_count_topic", raw_count_topic_);
    get_parameter("comp_count_topic", comp_count_topic_);
    get_parameter("time_image_topic", time_image_topic_);
    get_parameter("publish_time_image", publish_time_image_);
    get_parameter("time_image_vis_topic", time_image_vis_topic_);
    get_parameter("publish_time_image_vis", publish_time_image_vis_);
    get_parameter("sort_events_by_time", sort_events_by_time_);

    // ----------------------
    // QoS capteurs standard ROS (pas de fiabilite stricte, faible latence)
    // ----------------------
    auto qos = rclcpp::SensorDataQoS();

    // ----------------------
    // Souscription aux evenements
    // ----------------------
    event_sub_ = create_subscription<dv_ros2_msgs::msg::EventArray>(
        events_topic_, qos,
        std::bind(&EventVisualizer::event_cb, this, std::placeholders::_1));

    // ----------------------
    // Souscription IMU
    // ----------------------
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        imu_topic_, qos,
        std::bind(&EventVisualizer::imu_cb, this, std::placeholders::_1));

    // ----------------------
    // Publisher image de comptage
    // ----------------------
    image_pub_ = create_publisher<sensor_msgs::msg::Image>(
        count_image_topic_, 1);
    if (publish_raw_count_) {
        raw_count_pub_ = create_publisher<sensor_msgs::msg::Image>(
            raw_count_topic_, 1);
    }
    if (publish_comp_count_) {
        comp_count_pub_ = create_publisher<sensor_msgs::msg::Image>(
            comp_count_topic_, 1);
    }
    if (publish_time_image_) {
        time_image_pub_ = create_publisher<sensor_msgs::msg::Image>(
            time_image_topic_, 1);
    }
    if (publish_time_image_vis_) {
        time_image_vis_pub_ = create_publisher<sensor_msgs::msg::Image>(
            time_image_vis_topic_, 1);
    }

    RCLCPP_INFO(get_logger(), "Motion compensation node ready.");
}

// ============================================================================
// Affiche une image de comptage (nombre d'evenements par pixel)
// ============================================================================
void EventVisualizer::show_count_image(
    const std::vector<std::vector<int>> &count_image,
    int max_count,
    const std_msgs::msg::Header &header,
    const rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr &pub) {
    // Securite : eviter division par zero
    if (max_count <= 0 || !pub) {
        return;
    }

    // Image OpenCV en niveaux de gris (8 bits)
    cv::Mat image(height_param, width_param, CV_8UC1);

    // Mise a l'echelle pour occuper [0,255]
    int scale = (255 / max_count) + 1;

    for (int i = 0; i < height_param; ++i) {
        for (int j = 0; j < width_param; ++j) {
            image.at<uchar>(i, j) =
                static_cast<uchar>(count_image[i][j] * scale);
        }
    }

    // Conversion OpenCV -> ROS Image
    auto msg = cv_bridge::CvImage(header, "mono8", image).toImageMsg();

    pub->publish(*msg);
}

// ============================================================================
// Traitement principal : compensation de mouvement + image evenementielle
// ============================================================================
void EventVisualizer::data_process() {

    // Verifie qu'on a des donnees
    if (imu_buffer_snapshot_.empty() || event_buffer_.empty()) {
        return;
    }

    // Timestamp premier/dernier evenement
    const int64_t event_first_ns = toNs(event_buffer_.front().ts);
    const int64_t event_last_ns = toNs(event_buffer_.back().ts);

    // Timestamp dernier IMU
    const int64_t imu_last_ns =
        toNs(imu_buffer_snapshot_.back().header.stamp);

    // Si exige : IMU doit etre "en avance" sur les evenements
    if (require_imu_ahead_ && imu_last_ns <= event_last_ns) {
        event_buffer_.clear();
        imu_buffer_snapshot_.clear();
        return;
    }

    // ----------------------
    // Lissage IMU (moyenne des vitesses angulaires)
    // ----------------------
    float angular_velocity_x = 0.0f;
    float angular_velocity_y = 0.0f;
    float angular_velocity_z = 0.0f;
    int cnt = 0;

    const int64_t imu_min_ns = event_first_ns - imu_window_ns_;

    for (const auto &imu : imu_buffer_snapshot_) {
        const int64_t imu_ns = toNs(imu.header.stamp);
        if (imu_ns >= imu_min_ns) {
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

    float average_angular_rate_x = angular_velocity_x / cnt;
    float average_angular_rate_y = angular_velocity_y / cnt;
    float average_angular_rate_z = angular_velocity_z / cnt;

    // Temps de reference
    const int64_t t0 = event_first_ns;

    // Image de comptage et image temporelle
    std::vector<std::vector<int>> count_image(
        height_param, std::vector<int>(width_param));

    std::vector<std::vector<float>> time_image(
        height_param, std::vector<float>(width_param));

    // ----------------------
    // Image brute (sans compensation)
    // ----------------------
    std::vector<std::vector<int>> raw_count_image;
    if (publish_raw_count_ || publish_comp_count_) {
        raw_count_image.assign(
            height_param, std::vector<int>(width_param));
    }

    if (sort_events_by_time_) {
        std::sort(event_buffer_.begin(), event_buffer_.end(),
                  [](const auto &a, const auto &b) {
                      return toNs(a.ts) < toNs(b.ts);
                  });
    }

    // ----------------------
    // Boucle sur tous les evenements
    // ----------------------
    for (auto &evt : event_buffer_) {
        if (!raw_count_image.empty()) {
            int rx = static_cast<int>(evt.x);
            int ry = static_cast<int>(evt.y);
            if (ry >= 0 && ry < height_param &&
                rx >= 0 && rx < width_param) {
                if (raw_count_image[ry][rx] < 20) {
                    raw_count_image[ry][rx]++;
                }
            }
        }

        // dt depuis t0 (secondes)
        float time_diff =
            static_cast<float>((toNs(evt.ts) - t0) / 1e9);

        // Rotation induite par la vitesse angulaire moyenne
        const float x_angular = time_diff * average_angular_rate_x;
        const float y_angular = time_diff * average_angular_rate_y;
        const float z_angular = time_diff * average_angular_rate_z;

        // Coordonnees centrees
        int x = static_cast<int>(evt.x) - width_param / 2;
        int y = static_cast<int>(evt.y) - height_param / 2;

        // Angles avant compensation (projection camera)
        float pre_x_angle =
            std::atan(y * pixel_size_param / focus_param);
        float pre_y_angle =
            std::atan(x * pixel_size_param / focus_param);

        // Compensation rotation (yaw + pitch/roll)
        int compen_x =
            static_cast<int>(
                (x * std::cos(z_angular) - std::sin(z_angular) * y)
                - (x - (focus_param *
                        std::tan(pre_y_angle + y_angular)
                        / pixel_size_param))
            + width_param / 2);

        int compen_y =
            static_cast<int>(
                (x * std::sin(z_angular) + std::cos(z_angular) * y)
                - (y - (focus_param *
                        std::tan(pre_x_angle - x_angular)
                        / pixel_size_param))
            + height_param / 2);

        // Mise a jour evenement compense
        evt.x = static_cast<uint16_t>(compen_x);
        evt.y = static_cast<uint16_t>(compen_y);

        // Verification bornes image
        if (compen_y >= 0 && compen_y < height_param &&
            compen_x >= 0 && compen_x < width_param) {

            // Comptage limite pour eviter saturation
            if (count_image[compen_y][compen_x] < 20) {
                count_image[compen_y][compen_x]++;
            }

            // Accumulation temporelle
            time_image[compen_y][compen_x] += time_diff;
        }
    }

    // ----------------------
    // Normalisation temporelle + max
    // ----------------------
    int max_count = 0;
    for (int i = 0; i < height_param; ++i) {
        for (int j = 0; j < width_param; ++j) {
            if (count_image[i][j] != 0) {
                time_image[i][j] /=
                    static_cast<float>(count_image[i][j]);
                max_count =
                    std::max(max_count, count_image[i][j]);
            }
        }
    }

    // Publication image
    show_count_image(count_image, max_count, last_event_header_, image_pub_);

    if (!raw_count_image.empty()) {
        int raw_max = 0;
        for (int i = 0; i < height_param; ++i) {
            for (int j = 0; j < width_param; ++j) {
                raw_max = std::max(raw_max, raw_count_image[i][j]);
            }
        }
        if (publish_raw_count_) {
            show_count_image(raw_count_image, raw_max, last_event_header_, raw_count_pub_);
        }
        if (publish_comp_count_) {
            show_count_image(count_image, max_count, last_event_header_, comp_count_pub_);
        }
    }

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

    // Nettoyage buffers
    event_buffer_.clear();
    imu_buffer_snapshot_.clear();
}

// ========================================================================
// Callback evenements
// ========================================================================
void EventVisualizer::event_cb(const dv_ros2_msgs::msg::EventArray::SharedPtr msg) {

    // Tant que l'alignement n'est pas fait
    if (!first_event_received_) {

        // Snapshot IMU securise
        {
            std::lock_guard<std::mutex> lock(mtx_);
            imu_buffer_snapshot_ = imu_buffer_;
            imu_buffer_.clear();
        }

        if (imu_buffer_snapshot_.empty()) {
            return;
        }

        // Ajout evenements
        last_event_header_ = msg->header;
        event_buffer_.reserve(event_buffer_.size() + msg->events.size());
        for (const auto &evt : msg->events) {
            event_buffer_.emplace_back(evt);
        }

        // Traitement
        data_process();
    } else {
        // Premiere reception -> alignement initial
        first_event_received_ = false;
        imu_buffer_.clear();
        RCLCPP_INFO(get_logger(),
                    "Data aligned! Start processing data...");
    }
}

// ========================================================================
// Callback IMU
// ========================================================================
void EventVisualizer::imu_cb(const sensor_msgs::msg::Imu::SharedPtr imu) {
    if (!first_event_received_) {
        imu_buffer_.emplace_back(*imu);
    }
}
