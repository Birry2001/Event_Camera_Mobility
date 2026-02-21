// ======================
// Includes standards C++
// ======================
#include <algorithm>   // std::max, std::min
#include <array>
#include <cmath>       // trigonometrie : sin, cos, atan, tan
#include <cstdint>     // types entiers fixes (int64_t, uint16_t)
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
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
    declare_parameter<bool>("require_imu_ahead", false);
    declare_parameter<int64_t>("imu_window_ns", 3000000);

    declare_parameter<std::string>("events_topic", "events");
    declare_parameter<std::string>("imu_topic", "imu");
    declare_parameter<std::string>("count_image_topic", "count_image");
    declare_parameter<std::string>("time_image_topic", "time_image");
    declare_parameter<std::string>("time_image_vis_topic", "time_image_vis");
    declare_parameter<bool>("publish_time_image", true);
    declare_parameter<bool>("publish_time_image_vis", true);


    declare_parameter<double>("lambda_a", 1.0);
    declare_parameter<double>("lambda_b", 0.0);
    declare_parameter<std::string>("lambda_topic", "/event_lambda_threshold");

    declare_parameter<std::string>("omega_norm_topic", "/event_omega_norm");
    declare_parameter<double>("time_vis_m", 0.25);   // exemple, à ajuster

    declare_parameter<std::string>("events_comp_topic", "/events_compensated");
    declare_parameter<bool>("publish_compensated_events", true);

    declare_parameter<bool>("prefilter_enable", true);
    declare_parameter<bool>("prefilter_refractory_enable", true);
    declare_parameter<int64_t>("prefilter_refractory_us", 600);
    declare_parameter<bool>("prefilter_ba_enable", true);
    declare_parameter<int>("prefilter_ba_radius_px", 2);
    declare_parameter<int64_t>("prefilter_ba_window_us", 4000);
    declare_parameter<int>("prefilter_ba_min_neighbors", 1);
    declare_parameter<bool>("prefilter_ba_same_polarity_only", false);
    declare_parameter<bool>("prefilter_ba_support_from_kept_only", false);
    declare_parameter<bool>("prefilter_hot_pixel_enable", false);
    declare_parameter<int>("prefilter_hot_pixel_max_events_per_batch", 9);
    declare_parameter<bool>("prefilter_log_stats", false);

    declare_parameter<bool>("metrics_enable", true);
    declare_parameter<std::string>("noise_metrics_topic", "/event_noise_metrics");
    declare_parameter<bool>("metrics_log_stats", false);
    declare_parameter<int>("metrics_isolation_radius_px", 1);
    declare_parameter<int64_t>("metrics_isolation_window_us", 2000);
    declare_parameter<int>("metrics_hot_pixel_threshold", 8);
    declare_parameter<int>("metrics_small_component_pixels", 3);
    declare_parameter<int>("metrics_segmentation_min_count", 1);
    declare_parameter<double>("metrics_motion_omega_min", 0.1);

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
    get_parameter("time_image_topic", time_image_topic_);
    get_parameter("publish_time_image", publish_time_image_);
    get_parameter("time_image_vis_topic", time_image_vis_topic_);
    get_parameter("publish_time_image_vis", publish_time_image_vis_);

    get_parameter("lambda_a", a_);
    get_parameter("lambda_b", b_);
    get_parameter("lambda_topic", lambda_topic_);

    get_parameter("omega_norm_topic", omega_norm_topic_);

    get_parameter("time_vis_m", time_vis_m_);

    get_parameter("events_comp_topic", events_comp_topic_);
    get_parameter("publish_compensated_events", publish_compensated_events_);
    get_parameter("prefilter_enable", prefilter_enable_);
    get_parameter("prefilter_refractory_enable", prefilter_refractory_enable_);
    get_parameter("prefilter_refractory_us", prefilter_refractory_us_);
    get_parameter("prefilter_ba_enable", prefilter_ba_enable_);
    get_parameter("prefilter_ba_radius_px", prefilter_ba_radius_px_);
    get_parameter("prefilter_ba_window_us", prefilter_ba_window_us_);
    get_parameter("prefilter_ba_min_neighbors", prefilter_ba_min_neighbors_);
    get_parameter("prefilter_ba_same_polarity_only", prefilter_ba_same_polarity_only_);
    get_parameter("prefilter_ba_support_from_kept_only", prefilter_ba_support_from_kept_only_);
    get_parameter("prefilter_hot_pixel_enable", prefilter_hot_pixel_enable_);
    get_parameter("prefilter_hot_pixel_max_events_per_batch", prefilter_hot_pixel_max_events_per_batch_);
    get_parameter("prefilter_log_stats", prefilter_log_stats_);
    get_parameter("metrics_enable", metrics_enable_);
    get_parameter("noise_metrics_topic", noise_metrics_topic_);
    get_parameter("metrics_log_stats", metrics_log_stats_);
    get_parameter("metrics_isolation_radius_px", metrics_isolation_radius_px_);
    get_parameter("metrics_isolation_window_us", metrics_isolation_window_us_);
    get_parameter("metrics_hot_pixel_threshold", metrics_hot_pixel_threshold_);
    get_parameter("metrics_small_component_pixels", metrics_small_component_pixels_);
    get_parameter("metrics_segmentation_min_count", metrics_segmentation_min_count_);
    get_parameter("metrics_motion_omega_min", metrics_motion_omega_min_);
    if (prefilter_refractory_us_ < 0) {
        prefilter_refractory_us_ = 0;
    }
    if (prefilter_ba_radius_px_ < 1) {
        prefilter_ba_radius_px_ = 1;
    }
    if (prefilter_ba_window_us_ < 0) {
        prefilter_ba_window_us_ = 0;
    }
    if (prefilter_ba_min_neighbors_ < 1) {
        prefilter_ba_min_neighbors_ = 1;
    }
    if (prefilter_hot_pixel_max_events_per_batch_ < 2) {
        prefilter_hot_pixel_max_events_per_batch_ = 2;
    }
    if (metrics_isolation_radius_px_ < 1) {
        metrics_isolation_radius_px_ = 1;
    }
    if (metrics_isolation_window_us_ < 0) {
        metrics_isolation_window_us_ = 0;
    }
    if (metrics_hot_pixel_threshold_ < 2) {
        metrics_hot_pixel_threshold_ = 2;
    }
    if (metrics_small_component_pixels_ < 1) {
        metrics_small_component_pixels_ = 1;
    }
    if (metrics_segmentation_min_count_ < 1) {
        metrics_segmentation_min_count_ = 1;
    }
    if (metrics_motion_omega_min_ < 0.0) {
        metrics_motion_omega_min_ = 0.0;
    }

    init_filter_state();

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
    if (publish_time_image_) {
        time_image_pub_ = create_publisher<sensor_msgs::msg::Image>(
            time_image_topic_, 1);
    }
    if (publish_time_image_vis_) {
        time_image_vis_pub_ = create_publisher<sensor_msgs::msg::Image>(
            time_image_vis_topic_, 1);
    }
    
    //Publisher pour le seuil de compensation lambda
    lambda_pub_ = create_publisher<std_msgs::msg::Float32>(lambda_topic_, 10);
    if (metrics_enable_) {
        noise_metrics_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>(
            noise_metrics_topic_, 10);
    }

    //Publisher pour la norme d'omega
    omega_norm_pub_ = create_publisher<std_msgs::msg::Float32>(omega_norm_topic_, 10);
    RCLCPP_INFO(get_logger(), "Motion compensation node ready.");
    RCLCPP_INFO(get_logger(),
                "Event prefilter: enable=%s refractory=%s(%ld us) ba=%s(radius=%d, win=%ld us, min_n=%d, same_pol=%s, kept_only=%s) hot=%s(max/batch=%d)",
                prefilter_enable_ ? "true" : "false",
                prefilter_refractory_enable_ ? "true" : "false",
                static_cast<long>(prefilter_refractory_us_),
                prefilter_ba_enable_ ? "true" : "false",
                prefilter_ba_radius_px_,
                static_cast<long>(prefilter_ba_window_us_),
                prefilter_ba_min_neighbors_,
                prefilter_ba_same_polarity_only_ ? "true" : "false",
                prefilter_ba_support_from_kept_only_ ? "true" : "false",
                prefilter_hot_pixel_enable_ ? "true" : "false",
                prefilter_hot_pixel_max_events_per_batch_);
    RCLCPP_INFO(get_logger(),
                "Noise metrics: enable=%s topic=%s iso_r=%d iso_win=%ld us hot_thr=%d small_cc=%d seg_min_count=%d motion_omega_min=%.3f",
                metrics_enable_ ? "true" : "false",
                noise_metrics_topic_.c_str(),
                metrics_isolation_radius_px_,
                static_cast<long>(metrics_isolation_window_us_),
                metrics_hot_pixel_threshold_,
                metrics_small_component_pixels_,
                metrics_segmentation_min_count_,
                metrics_motion_omega_min_);

    if (publish_compensated_events_) {
    events_comp_pub_ = create_publisher<dv_ros2_msgs::msg::EventArray>(events_comp_topic_, qos);
    };

}

inline size_t EventVisualizer::pixel_index(const int x, const int y) const {
    return static_cast<size_t>(y) * static_cast<size_t>(width_param) +
           static_cast<size_t>(x);
}

float EventVisualizer::compute_isolated_ratio(
    const std::vector<dv_ros2_msgs::msg::Event> &events) const {
    if (events.empty()) {
        return 1.0f;
    }
    const int64_t window_ns = metrics_isolation_window_us_ * 1000LL;
    if (window_ns <= 0) {
        return 0.0f;
    }

    const size_t num_pixels = static_cast<size_t>(height_param) *
                              static_cast<size_t>(width_param);
    std::vector<int64_t> last_seen_ns(num_pixels, kUnsetTimeNs);
    size_t isolated_count = 0;

    for (const auto &evt : events) {
        const int x = static_cast<int>(evt.x);
        const int y = static_cast<int>(evt.y);
        if (x < 0 || x >= width_param || y < 0 || y >= height_param) {
            continue;
        }

        const int64_t evt_ns = toNs(evt.ts);
        bool has_support = false;

        for (int dy = -metrics_isolation_radius_px_;
             dy <= metrics_isolation_radius_px_; ++dy) {
            const int ny = y + dy;
            if (ny < 0 || ny >= height_param) {
                continue;
            }
            for (int dx = -metrics_isolation_radius_px_;
                 dx <= metrics_isolation_radius_px_; ++dx) {
                if (dx == 0 && dy == 0) {
                    continue;
                }
                const int nx = x + dx;
                if (nx < 0 || nx >= width_param) {
                    continue;
                }
                const int64_t neigh_ns = last_seen_ns[pixel_index(nx, ny)];
                if (neigh_ns == kUnsetTimeNs || evt_ns < neigh_ns) {
                    continue;
                }
                if ((evt_ns - neigh_ns) <= window_ns) {
                    has_support = true;
                    break;
                }
            }
            if (has_support) {
                break;
            }
        }

        if (!has_support) {
            isolated_count++;
        }
        last_seen_ns[pixel_index(x, y)] = evt_ns;
    }

    return static_cast<float>(isolated_count) /
           static_cast<float>(events.size());
}

float EventVisualizer::compute_hot_ratio(
    const std::vector<dv_ros2_msgs::msg::Event> &events,
    const int hot_threshold) const {
    if (events.empty() || hot_threshold <= 1) {
        return 0.0f;
    }

    const size_t num_pixels = static_cast<size_t>(height_param) *
                              static_cast<size_t>(width_param);
    std::vector<uint16_t> counts(num_pixels, 0);
    for (const auto &evt : events) {
        const int x = static_cast<int>(evt.x);
        const int y = static_cast<int>(evt.y);
        if (x < 0 || x >= width_param || y < 0 || y >= height_param) {
            continue;
        }
        uint16_t &c = counts[pixel_index(x, y)];
        if (c < std::numeric_limits<uint16_t>::max()) {
            c++;
        }
    }

    size_t hot_events = 0;
    for (const auto c : counts) {
        if (c >= static_cast<uint16_t>(hot_threshold)) {
            hot_events += static_cast<size_t>(c);
        }
    }

    return static_cast<float>(hot_events) /
           static_cast<float>(events.size());
}

float EventVisualizer::compute_small_component_ratio(
    const std::vector<std::vector<int>> &count_image,
    const int min_component_pixels) const {
    size_t active_pixels = 0;
    for (int y = 0; y < height_param; ++y) {
        for (int x = 0; x < width_param; ++x) {
            if (count_image[y][x] > 0) {
                active_pixels++;
            }
        }
    }

    if (active_pixels == 0 || min_component_pixels <= 1) {
        return 0.0f;
    }

    const size_t num_pixels = static_cast<size_t>(height_param) *
                              static_cast<size_t>(width_param);
    std::vector<uint8_t> visited(num_pixels, 0);
    std::vector<size_t> stack;
    stack.reserve(512);
    size_t small_pixels = 0;
    constexpr std::array<int, 8> kDx{{-1, 0, 1, -1, 1, -1, 0, 1}};
    constexpr std::array<int, 8> kDy{{-1, -1, -1, 0, 0, 1, 1, 1}};

    for (int sy = 0; sy < height_param; ++sy) {
        for (int sx = 0; sx < width_param; ++sx) {
            const size_t sidx = pixel_index(sx, sy);
            if (visited[sidx] != 0 || count_image[sy][sx] <= 0) {
                continue;
            }

            stack.clear();
            stack.push_back(sidx);
            visited[sidx] = 1;
            size_t comp_size = 0;

            while (!stack.empty()) {
                const size_t idx = stack.back();
                stack.pop_back();
                comp_size++;
                const int cx = static_cast<int>(idx % static_cast<size_t>(width_param));
                const int cy = static_cast<int>(idx / static_cast<size_t>(width_param));

                for (size_t k = 0; k < kDx.size(); ++k) {
                    const int nx = cx + kDx[k];
                    const int ny = cy + kDy[k];
                    if (nx < 0 || nx >= width_param || ny < 0 || ny >= height_param) {
                        continue;
                    }
                    const size_t nidx = pixel_index(nx, ny);
                    if (visited[nidx] != 0 || count_image[ny][nx] <= 0) {
                        continue;
                    }
                    visited[nidx] = 1;
                    stack.push_back(nidx);
                }
            }

            if (comp_size < static_cast<size_t>(min_component_pixels)) {
                small_pixels += comp_size;
            }
        }
    }

    return static_cast<float>(small_pixels) /
           static_cast<float>(active_pixels);
}

void EventVisualizer::init_filter_state() {
    const size_t num_pixels = static_cast<size_t>(height_param) *
                              static_cast<size_t>(width_param);
    last_kept_ts_ns_.assign(num_pixels, kUnsetTimeNs);
    last_seen_ts_ns_any_.assign(num_pixels, kUnsetTimeNs);
    last_seen_ts_ns_pos_.assign(num_pixels, kUnsetTimeNs);
    last_seen_ts_ns_neg_.assign(num_pixels, kUnsetTimeNs);
    last_input_event_ts_ns_ = kUnsetTimeNs;
}

std::vector<dv_ros2_msgs::msg::Event> EventVisualizer::filter_events(
    const std::vector<dv_ros2_msgs::msg::Event> &input_events) {
    if (!prefilter_enable_ || input_events.empty()) {
        return input_events;
    }

    const int64_t refractory_ns = prefilter_refractory_us_ * 1000LL;
    const int64_t ba_window_ns = prefilter_ba_window_us_ * 1000LL;

    std::vector<dv_ros2_msgs::msg::Event> output_events;
    output_events.reserve(input_events.size());

    size_t dropped_refractory = 0;
    size_t dropped_ba = 0;
    size_t dropped_hot_pixel = 0;

    std::vector<uint16_t> batch_pixel_count;
    if (prefilter_hot_pixel_enable_) {
        const size_t num_pixels = static_cast<size_t>(height_param) *
                                  static_cast<size_t>(width_param);
        batch_pixel_count.assign(num_pixels, 0);
        for (const auto &evt : input_events) {
            const int x = static_cast<int>(evt.x);
            const int y = static_cast<int>(evt.y);
            if (x < 0 || x >= width_param || y < 0 || y >= height_param) {
                continue;
            }
            const size_t idx = pixel_index(x, y);
            if (batch_pixel_count[idx] < std::numeric_limits<uint16_t>::max()) {
                batch_pixel_count[idx]++;
            }
        }
    }

    for (const auto &evt : input_events) {
        const int x = static_cast<int>(evt.x);
        const int y = static_cast<int>(evt.y);

        if (x < 0 || x >= width_param || y < 0 || y >= height_param) {
            continue;
        }

        const int64_t evt_ns = toNs(evt.ts);
        if (last_input_event_ts_ns_ != kUnsetTimeNs &&
            evt_ns < last_input_event_ts_ns_) {
            RCLCPP_WARN(get_logger(),
                        "Event timestamp rollback detected (%ld -> %ld), resetting prefilter state.",
                        static_cast<long>(last_input_event_ts_ns_),
                        static_cast<long>(evt_ns));
            init_filter_state();
        }
        last_input_event_ts_ns_ = evt_ns;

        const size_t idx = pixel_index(x, y);
        bool keep = true;

        if (prefilter_hot_pixel_enable_ &&
            !batch_pixel_count.empty() &&
            batch_pixel_count[idx] >= static_cast<uint16_t>(prefilter_hot_pixel_max_events_per_batch_)) {
            keep = false;
            dropped_hot_pixel++;
        }

        if (keep && prefilter_refractory_enable_ && refractory_ns > 0) {
            const int64_t last_kept_ns = last_kept_ts_ns_[idx];
            if (last_kept_ns != kUnsetTimeNs &&
                evt_ns >= last_kept_ns &&
                (evt_ns - last_kept_ns) < refractory_ns) {
                keep = false;
                dropped_refractory++;
            }
        }

        if (keep && prefilter_ba_enable_ && ba_window_ns > 0) {
            int neighbor_count = 0;
            for (int dy = -prefilter_ba_radius_px_; dy <= prefilter_ba_radius_px_; ++dy) {
                const int ny = y + dy;
                if (ny < 0 || ny >= height_param) {
                    continue;
                }
                for (int dx = -prefilter_ba_radius_px_; dx <= prefilter_ba_radius_px_; ++dx) {
                    const int nx = x + dx;
                    if (nx < 0 || nx >= width_param) {
                        continue;
                    }
                    if (dx == 0 && dy == 0) {
                        continue;
                    }

                    const size_t nidx = pixel_index(nx, ny);
                    const int64_t neigh_ns = prefilter_ba_same_polarity_only_
                        ? (evt.polarity ? last_seen_ts_ns_pos_[nidx] : last_seen_ts_ns_neg_[nidx])
                        : last_seen_ts_ns_any_[nidx];
                    if (neigh_ns == kUnsetTimeNs || evt_ns < neigh_ns) {
                        continue;
                    }
                    if ((evt_ns - neigh_ns) <= ba_window_ns) {
                        neighbor_count++;
                        if (neighbor_count >= prefilter_ba_min_neighbors_) {
                            break;
                        }
                    }
                }
                if (neighbor_count >= prefilter_ba_min_neighbors_) {
                    break;
                }
            }

            if (neighbor_count < prefilter_ba_min_neighbors_) {
                keep = false;
                dropped_ba++;
            }
        }

        if (!keep) {
            if (!prefilter_ba_support_from_kept_only_) {
                last_seen_ts_ns_any_[idx] = evt_ns;
                if (evt.polarity) {
                    last_seen_ts_ns_pos_[idx] = evt_ns;
                } else {
                    last_seen_ts_ns_neg_[idx] = evt_ns;
                }
            }
            continue;
        }

        output_events.emplace_back(evt);
        last_seen_ts_ns_any_[idx] = evt_ns;
        if (evt.polarity) {
            last_seen_ts_ns_pos_[idx] = evt_ns;
        } else {
            last_seen_ts_ns_neg_[idx] = evt_ns;
        }
        if (prefilter_refractory_enable_ && refractory_ns > 0) {
            last_kept_ts_ns_[idx] = evt_ns;
        }
    }

    prefilter_total_in_ += input_events.size();
    prefilter_total_out_ += output_events.size();
    prefilter_total_drop_refractory_ += dropped_refractory;
    prefilter_total_drop_ba_ += dropped_ba;
    prefilter_total_drop_hot_pixel_ += dropped_hot_pixel;

    if (prefilter_log_stats_) {
        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Prefilter in=%zu out=%zu drop_ref=%zu drop_ba=%zu drop_hot=%zu keep=%.2f%%",
            input_events.size(),
            output_events.size(),
            dropped_refractory,
            dropped_ba,
            dropped_hot_pixel,
            prefilter_total_in_ > 0
                ? 100.0 * static_cast<double>(prefilter_total_out_) /
                      static_cast<double>(prefilter_total_in_)
                : 0.0);
    }

    return output_events;
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
    cv::Mat image(height_param, width_param, CV_8UC1, cv::Scalar(0));

    const float denom = std::log1p(static_cast<float>(max_count));  // log(1+max)
    for (int i = 0; i < height_param; ++i) {
        for (int j = 0; j < width_param; ++j) {
            const int c = count_image[i][j];
            float v = 0.0f;
            if (denom > 0.0f && c > 0) {
                v = std::log1p(static_cast<float>(c)) / denom; // dans [0,1]
            }
            const int pix = static_cast<int>(v * 255.0f + 0.5f);
            image.at<uchar>(i, j) = static_cast<uchar>(std::min(255, std::max(0, pix)));
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

    const int64_t event_first_ns = toNs(event_buffer_.front().ts);
    const int64_t imu_last_ns = toNs(imu_buffer_snapshot_.back().header.stamp);
    const int64_t t_last_ns = toNs(event_buffer_.back().ts);


    // On ne traite les données que si le message imu le plus récent est plus récent que le premier évènement
    if (imu_last_ns <= event_first_ns) {
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

    
    // Calcul de lambda a partir de la norme de vitesse angulaire
    const float omega_norm = std::sqrt(
        average_angular_rate_x * average_angular_rate_x +
        average_angular_rate_y * average_angular_rate_y +
        average_angular_rate_z * average_angular_rate_z
    );

    //Publication de la norme d'omega
    if (omega_norm_pub_) {
    std_msgs::msg::Float32 om;
    om.data = omega_norm;
    omega_norm_pub_->publish(om);
    };
    const float lambda = static_cast<float>(a_) * omega_norm + static_cast<float>(b_);

    // Temps de reference
    const int64_t t0 = event_first_ns;

    //Calcul de delta t la durrée d'un batch d'évènements pour la normalisation
    const float delta_t = static_cast<float>((t_last_ns - t0) / 1e9);
    if (delta_t <= 0.0f) {
        event_buffer_.clear();
        imu_buffer_snapshot_.clear();
        return;
    }

    const size_t pre_event_count = event_buffer_.size();
    float isolated_ratio_pre = 0.0f;
    float hot_ratio_pre = 0.0f;
    if (metrics_enable_) {
        isolated_ratio_pre = compute_isolated_ratio(event_buffer_);
        hot_ratio_pre = compute_hot_ratio(event_buffer_, metrics_hot_pixel_threshold_);
    }

    // Image de comptage et image temporelle
    std::vector<std::vector<int>> count_image(
        height_param, std::vector<int>(width_param));

    std::vector<std::vector<float>> time_image(
        height_param, std::vector<float>(width_param));

    std::vector<dv_ros2_msgs::msg::Event> compensated_events;
    compensated_events.reserve(event_buffer_.size());

    // ----------------------
    // Boucle sur tous les evenements
    // ----------------------
    for (const auto &evt_in : event_buffer_) {
        // dt depuis t0 (secondes)
        float time_diff =
            static_cast<float>((toNs(evt_in.ts) - t0) / 1e9);

        // Rotation induite par la vitesse angulaire moyenne
        const float x_angular = time_diff * average_angular_rate_x;
        const float y_angular = time_diff * average_angular_rate_y;
        const float z_angular = time_diff * average_angular_rate_z;

        // Coordonnees centrees
        int x = static_cast<int>(evt_in.x) - width_param / 2;
        int y = static_cast<int>(evt_in.y) - height_param / 2;

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

        // Verification bornes image
        if (compen_y < 0 || compen_y >= height_param ||
            compen_x < 0 || compen_x >= width_param) {
            continue;
        }

        dv_ros2_msgs::msg::Event evt = evt_in;
        evt.x = static_cast<uint16_t>(compen_x);
        evt.y = static_cast<uint16_t>(compen_y);
        compensated_events.emplace_back(evt);

        // Comptage limite pour eviter saturation
        if (count_image[compen_y][compen_x] < 20) {
            count_image[compen_y][compen_x]++;
        }

        // Accumulation temporelle
        time_image[compen_y][compen_x] += time_diff;
    }

    event_buffer_.swap(compensated_events);

    // ----------------------
    // Normalisation temporelle + max
    // ----------------------
    int max_count = 0;
    for (int i = 0; i < height_param; ++i) {
        for (int j = 0; j < width_param; ++j) {
            if (count_image[i][j] != 0) {
                time_image[i][j] /=
                    static_cast<float>(count_image[i][j]);

                time_image[i][j] /= delta_t;  // Normalisation de l'image de temporelle par la durrée d'un batch d'évènements
                max_count =
                    std::max(max_count, count_image[i][j]);
            }
        }
    }

    // 1) moyenne globale sur pixels actifs
    double sum_T = 0.0;
    int n_T = 0;
    for (int i = 0; i < height_param; ++i) {
        for (int j = 0; j < width_param; ++j) {
            if (count_image[i][j] != 0) {
                sum_T += time_image[i][j];
                n_T++;
            }
        }
    }
    const float mean_T = (n_T > 0) ? static_cast<float>(sum_T / n_T) : 0.0f;

    // 2) centrage : Tbar = T - mean(T)
    for (int i = 0; i < height_param; ++i) {
        for (int j = 0; j < width_param; ++j) {
            if (count_image[i][j] != 0) {
                time_image[i][j] -= mean_T;
            } else {
                time_image[i][j] = 0.0f;
            }
        }
    }

    // Publication finale de lambda
    if (lambda_pub_) {
        std_msgs::msg::Float32 msg;
        msg.data = lambda;
        lambda_pub_->publish(msg);
    }

    size_t seg_active_pixels = 0;
    size_t seg_fg_pixels = 0;
    uint64_t seg_active_events = 0;
    uint64_t seg_fg_events = 0;
    for (int i = 0; i < height_param; ++i) {
        for (int j = 0; j < width_param; ++j) {
            const int c = count_image[i][j];
            if (c < metrics_segmentation_min_count_) {
                continue;
            }
            seg_active_pixels++;
            seg_active_events += static_cast<uint64_t>(c);
            if (std::abs(time_image[i][j]) > lambda) {
                seg_fg_pixels++;
                seg_fg_events += static_cast<uint64_t>(c);
            }
        }
    }

    // Publication image
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

        cv::Mat time_vis(height_param, width_param, CV_8UC1, cv::Scalar(128));

        const float m = static_cast<float>(time_vis_m_);
        if (m > 0.0f) {
            for (int i = 0; i < height_param; ++i) {
                for (int j = 0; j < width_param; ++j) {
                    if (count_image[i][j] == 0) {
                        continue; // reste gris neutre
                    }
                    float u = time_image[i][j] / m;           // time_image = Tbar (centré)
                    u = std::clamp(u, -1.0f, 1.0f);
                    float pix = 128.0f + 127.0f * u;
                    time_vis.at<uchar>(i, j) = static_cast<uchar>(pix);
                }
            }
        }

        auto vis_msg = cv_bridge::CvImage(last_event_header_, "mono8", time_vis).toImageMsg();
        time_image_vis_pub_->publish(*vis_msg);

    };


    const std::vector<dv_ros2_msgs::msg::Event> *final_events = &event_buffer_;

    if (metrics_enable_ && noise_metrics_pub_) {
        const float isolated_ratio_post = compute_isolated_ratio(*final_events);
        const float hot_ratio_post = compute_hot_ratio(*final_events, metrics_hot_pixel_threshold_);
        const float small_cc_ratio = compute_small_component_ratio(
            count_image, metrics_small_component_pixels_);
        const float fg_pixel_ratio =
            (seg_active_pixels > 0)
                ? static_cast<float>(seg_fg_pixels) / static_cast<float>(seg_active_pixels)
                : 0.0f;
        const float fg_event_ratio =
            (seg_active_events > 0)
                ? static_cast<float>(seg_fg_events) / static_cast<float>(seg_active_events)
                : 0.0f;
        const bool motion_active = omega_norm >= static_cast<float>(metrics_motion_omega_min_);

        float noise_score =
            0.45f * isolated_ratio_post +
            0.35f * hot_ratio_post +
            0.20f * small_cc_ratio;
        if (motion_active && fg_event_ratio < 0.02f) {
            noise_score += (0.02f - fg_event_ratio) * 4.0f;
        }

        std_msgs::msg::Float32MultiArray metrics;
        metrics.data = {
            noise_score,
            isolated_ratio_pre,
            isolated_ratio_post,
            hot_ratio_pre,
            hot_ratio_post,
            small_cc_ratio,
            static_cast<float>(pre_event_count),
            static_cast<float>(final_events->size()),
            fg_event_ratio,
            fg_pixel_ratio,
            static_cast<float>(seg_fg_events),
            static_cast<float>(seg_active_events),
            lambda,
            omega_norm,
            motion_active ? 1.0f : 0.0f,
            static_cast<float>(max_count)
        };
        noise_metrics_pub_->publish(metrics);

        if (metrics_log_stats_) {
            RCLCPP_INFO_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Noise metrics score=%.4f iso(pre/post)=%.3f/%.3f hot(pre/post)=%.3f/%.3f small_cc=%.3f fg_evt=%.3f fg_px=%.3f keep=%zu/%zu motion=%s",
                noise_score,
                isolated_ratio_pre,
                isolated_ratio_post,
                hot_ratio_pre,
                hot_ratio_post,
                small_cc_ratio,
                fg_event_ratio,
                fg_pixel_ratio,
                final_events->size(),
                pre_event_count,
                motion_active ? "true" : "false");
        }
    }

    //Publication des évènements compensés
    if (publish_compensated_events_ && events_comp_pub_) {
        dv_ros2_msgs::msg::EventArray out;
        out.header = last_event_header_;     // même frame/timestamp que le batch reçu
        out.events = *final_events;
        events_comp_pub_->publish(out);
    }


    
    // Nettoyage buffers
    event_buffer_.clear();
    imu_buffer_snapshot_.clear();

}

// ========================================================================
// Callback evenements
// ========================================================================
void EventVisualizer::event_cb(const dv_ros2_msgs::msg::EventArray::SharedPtr msg) {
    if (!msg || msg->events.empty()) {
        return;
    }

    // Tant que l'alignement n'est pas fait
    if (first_event_received_ == false) {

        // Snapshot IMU securise
        {
            std::lock_guard<std::mutex> lock(mtx_);
            imu_buffer_snapshot_ = imu_buffer_;
            imu_buffer_.clear();
        }

        if (imu_buffer_snapshot_.empty()) {
            return;
        }

        const auto filtered_events = filter_events(msg->events);
        if (filtered_events.empty()) {
            return;
        }

        last_event_header_ = msg->header;
        event_buffer_.reserve(event_buffer_.size() + filtered_events.size());
        for (const auto &evt : filtered_events) {
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
        std::lock_guard<std::mutex> lock(mtx_);
        imu_buffer_.emplace_back(*imu);
    }
}
