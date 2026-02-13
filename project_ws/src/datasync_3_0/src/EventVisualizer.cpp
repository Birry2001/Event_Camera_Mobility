// ======================
// Includes standards C++
// ======================
#include <algorithm>   // std::max, std::min
#include <cmath>       // trigonometrie : sin, cos, atan, tan
#include <cstdint>     // types entiers fixes (int64_t, uint16_t)
#include <std_msgs/msg/float32.hpp>
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

    //Publisher pour la norme d'omega
    omega_norm_pub_ = create_publisher<std_msgs::msg::Float32>(omega_norm_topic_, 10);
    RCLCPP_INFO(get_logger(), "Motion compensation node ready.");

    if (publish_compensated_events_) {
    events_comp_pub_ = create_publisher<dv_ros2_msgs::msg::EventArray>(events_comp_topic_, qos);
    };

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

    
    // Calcul de lambda
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

    //publication de lambda
    if (lambda_pub_) {
        std_msgs::msg::Float32 msg;
        msg.data = lambda;
        lambda_pub_->publish(msg);
    }


    // Temps de reference
    const int64_t t0 = event_first_ns;

    //Calcul de delta t la durrée d'un batch d'évènements pour la normalisation
    const float delta_t = static_cast<float>((t_last_ns - t0) / 1e9);
    if (delta_t <= 0.0f) {
        event_buffer_.clear();
        imu_buffer_snapshot_.clear();
        return;
    }

    // Image de comptage et image temporelle
    std::vector<std::vector<int>> count_image(
        height_param, std::vector<int>(width_param));

    std::vector<std::vector<float>> time_image(
        height_param, std::vector<float>(width_param));

    // ----------------------
    // Boucle sur tous les evenements
    // ----------------------
    for (auto &evt : event_buffer_) {
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
                        std::tan(pre_y_angle - y_angular)
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
             count_image[compen_y][compen_x]++;

            // Accumulation temporelle
            if (time_image[compen_y][compen_x] < 20) {
                time_image[compen_y][compen_x]++;
            }
            // time_image[compen_y][compen_x] += time_diff;
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


    //Publication des évènements compensés
    if (publish_compensated_events_ && events_comp_pub_) {
    dv_ros2_msgs::msg::EventArray out;
    out.header = last_event_header_;     // même frame/timestamp que le batch reçu
    out.events = event_buffer_;          // events avec x,y modifiés (compensés)
    events_comp_pub_->publish(out);
    };


    
    // Nettoyage buffers
    event_buffer_.clear();
    imu_buffer_snapshot_.clear();

}

// ========================================================================
// Callback evenements
// ========================================================================
void EventVisualizer::event_cb(const dv_ros2_msgs::msg::EventArray::SharedPtr msg) {

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
        std::lock_guard<std::mutex> lock(mtx_);
        imu_buffer_.emplace_back(*imu);
    }
}
