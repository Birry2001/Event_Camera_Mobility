/*******************************************************
 *  CE FICHIER = VERSION ROS 2 (C++) D’UN “MOTION COMPENSATION NODE”
 *
 *  Objectif global (en langage simple) :
 *   1) Recevoir des événements d’une caméra à événements (DAVIS) sur un topic ROS 2.
 *   2) Recevoir des mesures IMU (vitesse angulaire).
 *   3) Faire une “synchronisation simple” : on prend un snapshot (copie) des IMU
 *      au moment où on reçoit un paquet d’événements.
 *   4) Calculer une vitesse angulaire moyenne sur une fenêtre temporelle récente.
 *   5) Compense (corrige) la position (x,y) des événements en supposant une rotation.
 *   6) Construire :
 *        - une image de comptage (count image) = nb d’événements par pixel
 *        - une image temporelle (time image)   = temps moyen (Δt) par pixel
 *      + versions optionnelles : brute (raw) et compensée (comp)
 *   7) Publier ces images sous forme sensor_msgs/Image (ROS).
 *
 *  Concepts importants pour quelqu’un qui apprend :
 *   - ROS 2 Node / Publishers / Subscriptions / QoS
 *   - Callbacks (fonctions appelées automatiquement à la réception)
 *   - Buffers (vecteurs) et multi-thread => mutex / lock_guard
 *   - OpenCV cv::Mat + conversion avec cv_bridge
 *   - Paramètres ROS 2 (declare_parameter / get_parameter)
 *******************************************************/


// ======================
// Includes standards C++
// ======================
#include <algorithm>   // std::max, std::sort
#include <cmath>       // sin, cos, atan, tan, sqrt...
#include <cstdint>     // int64_t, uint16_t etc. (types entiers “fixes”)
#include <mutex>       // std::mutex, std::lock_guard (protection multi-thread)
#include <vector>      // std::vector (tableau dynamique)

// ======================
// OpenCV + cv_bridge
// ======================
#include <cv_bridge/cv_bridge.h>     // Convertit cv::Mat <-> sensor_msgs::Image
#include <opencv2/core.hpp>          // cv::Mat (structure d’image OpenCV)
#include <opencv2/imgproc.hpp>       // fonctions de traitement image (pas obligatoire ici)

// ======================
// ROS 2
// ======================
#include <rclcpp/rclcpp.hpp>          // API ROS 2 C++ (Node, log, exec...)
#include <sensor_msgs/msg/image.hpp>  // Message ROS Image
#include <sensor_msgs/msg/imu.hpp>    // Message ROS IMU
#include <sensor_msgs/image_encodings.hpp> // strings d’encodage (mono8, 32FC1...)
#include <std_msgs/msg/header.hpp>    // Header ROS (stamp + frame_id)

// ======================
// Messages DAVIS / événements
// ======================
#include <dv_ros2_msgs/msg/event.hpp>       // Un événement (x,y,ts,polarity...)
#include <dv_ros2_msgs/msg/event_array.hpp> // Tableau d’événements + header

// Alias de type : “sll” = long long int
// (peu utilisé ici, mais classique pour manipuler des timestamps)
using sll = long long int;


// ============================================================================
// Namespace anonyme : paramètres globaux et fonctions utilitaires internes
// ============================================================================
//
// Un “namespace anonyme” (namespace { ... }) signifie :
// - ces variables / fonctions sont visibles uniquement dans CE fichier .cpp
// - c’est une manière propre de faire du “privé global”.
//
namespace {

// Résolution du capteur événementiel (DAVIS 346)
int height_param = 260;   // hauteur de l’image en pixels
int width_param  = 346;   // largeur de l’image en pixels

// Paramètres intrinsèques caméra (utilisés dans la formule de compensation)
float focus_param       = 6550.0f;  // focale
float pixel_size_param  = 18.5f;    // taille d’un pixel

// Fonction utilitaire : convertit un timestamp ROS (sec + nanosec)
// en un int64 (nanosecondes totales)
//
// builtin_interfaces::msg::Time contient :
//   - t.sec     (secondes)
//   - t.nanosec (nanosecondes restantes)
// On veut : sec*1e9 + nanosec
//
inline int64_t toNs(const builtin_interfaces::msg::Time &t) {
    return static_cast<int64_t>(t.sec) * 1000000000LL +
           static_cast<int64_t>(t.nanosec);
}

}  // namespace


// ============================================================================
// Classe principale : EventVisualizer
// - Hérite de rclcpp::Node (c’est un “nœud ROS 2”)
// ============================================================================
class EventVisualizer : public rclcpp::Node {
public:
    // ======================
    // Constructeur du nœud
    // ======================
    //
    // EventVisualizer() : Node("datasync_node")
    // -> appelle le constructeur de la classe parent rclcpp::Node
    // -> le nœud s’appellera "datasync_node"
    //
    EventVisualizer() : Node("datasync_node") {

        // ----------------------
        // Déclaration des paramètres ROS 2
        // ----------------------
        //
        // En ROS 2, on “déclare” d’abord les paramètres (avec une valeur par défaut),
        // puis on peut les lire avec get_parameter().
        //
        // NOTE : tes paramètres “weight_param” en fait c’est “width”.
        // Tu as gardé le nom historique “weight_param”.
        //

        declare_parameter<int>("weight_param", width_param);            // largeur image
        declare_parameter<int>("height_param", height_param);           // hauteur image
        declare_parameter<double>("focus", focus_param);                // focale
        declare_parameter<double>("pixel_size", pixel_size_param);      // taille pixel
        declare_parameter<bool>("require_imu_ahead", true);             // exiger IMU plus récent que events
        declare_parameter<int64_t>("imu_window_ns", 3000000);           // fenêtre IMU (ns)

        // Paramètres de topics (noms des topics)
        declare_parameter<std::string>("events_topic", "events");       // topic events
        declare_parameter<std::string>("imu_topic", "imu");             // topic imu
        declare_parameter<std::string>("count_image_topic", "count_image"); // topic principal count

        // Options de publication
        declare_parameter<bool>("publish_raw_count", false);            // publier count “brut”
        declare_parameter<std::string>("raw_count_topic", "count_image_raw");  // nom topic raw

        // Time image (float) et sa visualisation (8 bits)
        declare_parameter<std::string>("time_image_topic", "time_image");
        declare_parameter<std::string>("time_image_vis_topic", "time_image_vis");
        declare_parameter<bool>("publish_time_image", true);
        declare_parameter<bool>("publish_time_image_vis", true);

        // Option : trier les événements par timestamp avant traitement
        declare_parameter<bool>("sort_events_by_time", true);

        // ----------------------
        // Lecture des paramètres (récupération des valeurs effectives)
        // ----------------------
        //
        // get_parameter("nom", variable) copie la valeur du paramètre dans la variable.
        //
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
        get_parameter("raw_count_topic", raw_count_topic_);
        get_parameter("time_image_topic", time_image_topic_);
        get_parameter("publish_time_image", publish_time_image_);
        get_parameter("time_image_vis_topic", time_image_vis_topic_);
        get_parameter("publish_time_image_vis", publish_time_image_vis_);
        get_parameter("sort_events_by_time", sort_events_by_time_);

        // ----------------------
        // QoS capteurs standard ROS 2
        // ----------------------
        //
        // rclcpp::SensorDataQoS() = profil QoS typique pour capteurs :
        // - faible latence
        // - pas forcément “reliable” (ça peut drop si surcharge)
        // - adapté aux flux rapides (IMU, caméra, lidar)
        //
        auto qos = rclcpp::SensorDataQoS();

        // ----------------------
        // Souscription aux événements
        // ----------------------
        //
        // create_subscription<MessageType>(topic, qos, callback)
        // callback ici = std::bind(...), qui permet de lier une méthode membre.
        //
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
        // Publisher image de comptage (principal)
        // ----------------------
        //
        // create_publisher<MessageType>(topic, queue_size)
        //
        image_pub_ = create_publisher<sensor_msgs::msg::Image>(
            count_image_topic_, 1);

        // Publishers optionnels (selon paramètres)
        if (publish_raw_count_) {
            raw_count_pub_ = create_publisher<sensor_msgs::msg::Image>(
                raw_count_topic_, 1);
        }
        if (publish_time_image_) {
            time_image_pub_ = create_publisher<sensor_msgs::msg::Image>(
                time_image_topic_, 1);
        }
        if (publish_time_image_vis_) {
            time_image_vis_pub_ = create_publisher<sensor_msgs::msg::Image>(
                time_image_vis_topic_, 1);
        }

        // Log ROS 2 (équivalent ROS1 ROS_INFO)
        RCLCPP_INFO(get_logger(), "Motion compensation node ready.");
    }

private:
    // ========================================================================
    // show_count_image :
    // - prend une matrice 2D d'entiers (count_image)
    // - convertit en image OpenCV (8 bits)
    // - convertit en ROS sensor_msgs/Image (mono8)
    // - publie sur le publisher "pub"
    // ========================================================================
    //
    // Remarques pédagogiques :
    // - count_image est passé en "const &" : on ne copie pas, et on ne modifie pas.
    // - max_count est la valeur max dans count_image : utilisée pour scaler vers [0..255]
    // - header sert à conserver le timestamp / frame_id cohérents (pris des événements)
    // - pub est un pointeur partagé vers un publisher ROS 2.
    //
    void show_count_image(const std::vector<std::vector<int>> &count_image,
                          int max_count,
                          const std_msgs::msg::Header &header,
                          const rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr &pub) {
        // Sécurité : si max_count <= 0 => division par zéro possible
        // ou si pub == nullptr => pas de publisher valide
        if (max_count <= 0 || !pub) {
            return; // on ne fait rien
        }

        // Création d’une image OpenCV en niveaux de gris 8 bits :
        // - height_param lignes
        // - width_param colonnes
        // - CV_8UC1 = unsigned char (0..255), 1 canal
        cv::Mat image(height_param, width_param, CV_8UC1);

        // Mise à l’échelle :
        // si max_count est grand, on réduit pour que ça tienne dans [0..255]
        // +1 pour éviter scale=0
        int scale = (255 / max_count) + 1;

        // Remplissage pixel par pixel
        for (int i = 0; i < height_param; ++i) {
            for (int j = 0; j < width_param; ++j) {
                // image.at<uchar>(row,col) = valeur du pixel
                // static_cast<uchar> : conversion explicite en unsigned char
                image.at<uchar>(i, j) =
                    static_cast<uchar>(count_image[i][j] * scale);
            }
        }

        // Conversion OpenCV -> ROS :
        // CvImage(header, encoding, image).toImageMsg()
        auto msg = cv_bridge::CvImage(header, "mono8", image).toImageMsg();

        // Publication
        pub->publish(*msg);
    }


    // ========================================================================
    // data_process :
    // Fonction "coeur" :
    //  - vérifie buffers
    //  - calcule moyenne IMU
    //  - compense les events
    //  - construit count_image / time_image
    //  - publie selon options
    //  - nettoie buffers
    // ========================================================================
    void data_process() {

        // Vérifie qu’on a des données (sinon impossible de traiter)
        if (imu_buffer_snapshot_.empty() || event_buffer_.empty()) {
            return;
        }

        // Timestamp du premier et dernier événement (ns)
        // front() = premier élément du vector
        // back()  = dernier élément du vector
        const int64_t event_first_ns = toNs(event_buffer_.front().ts);
        const int64_t event_last_ns  = toNs(event_buffer_.back().ts);

        // Timestamp du dernier IMU dans le snapshot
        const int64_t imu_last_ns =
            toNs(imu_buffer_snapshot_.back().header.stamp);

        // Si require_imu_ahead_ == true :
        // on exige que le dernier IMU soit plus récent que le dernier event
        // -> sinon on considère que la fenêtre IMU n’est pas suffisante.
        if (require_imu_ahead_ && imu_last_ns <= event_last_ns) {
            event_buffer_.clear();
            imu_buffer_snapshot_.clear();
            return;
        }

        // ----------------------
        // Lissage IMU : moyenne des vitesses angulaires
        // ----------------------
        float angular_velocity_x = 0.0f; // somme omega_x
        float angular_velocity_y = 0.0f; // somme omega_y
        float angular_velocity_z = 0.0f; // somme omega_z
        int cnt = 0;                     // nombre d’IMU utilisées

        // On ne garde que les IMU dont le timestamp est >= event_first_ns - imu_window_ns_
        // Exemple : 3 ms avant le premier événement
        const int64_t imu_min_ns = event_first_ns - imu_window_ns_;

        // Parcours de toutes les IMU dans le snapshot
        for (const auto &imu : imu_buffer_snapshot_) {
            const int64_t imu_ns = toNs(imu.header.stamp);

            // Si l’IMU est dans la fenêtre temporelle => on l’utilise
            if (imu_ns >= imu_min_ns) {
                angular_velocity_x += static_cast<float>(imu.angular_velocity.x);
                angular_velocity_y += static_cast<float>(imu.angular_velocity.y);
                angular_velocity_z += static_cast<float>(imu.angular_velocity.z);
                cnt++;
            }
        }

        // Si on n’a pris aucune IMU => impossible de faire une moyenne
        if (cnt == 0) {
            event_buffer_.clear();
            imu_buffer_snapshot_.clear();
            return;
        }

        // Calcul des moyennes
        float average_angular_rate_x = angular_velocity_x / cnt;
        float average_angular_rate_y = angular_velocity_y / cnt;
        float average_angular_rate_z = angular_velocity_z / cnt;

        // Temps de référence : t0 = timestamp du premier event (ns)
        const int64_t t0 = event_first_ns;

        // count_image : image 2D d’int, initialisée à 0
        std::vector<std::vector<int>> count_image(
            height_param, std::vector<int>(width_param));

        // time_image : image 2D de float, initialisée à 0
        std::vector<std::vector<float>> time_image(
            height_param, std::vector<float>(width_param));

        // ----------------------
        // Image brute (sans compensation) : raw_count_image
        // ----------------------
        //
        // On ne la crée que si on a besoin de publier raw ou comp séparément
        //
        std::vector<std::vector<int>> raw_count_image;
        if (publish_raw_count_) {
            raw_count_image.assign(
                height_param, std::vector<int>(width_param));
        }

        // Option : trier les événements par timestamp
        // Utile si l’ordre n’est pas garanti par le driver / message
        if (sort_events_by_time_) {
            std::sort(event_buffer_.begin(), event_buffer_.end(),
                      [](const auto &a, const auto &b) {
                          return toNs(a.ts) < toNs(b.ts);
                      });
        }

        // ----------------------
        // Boucle sur tous les événements
        // ----------------------
        for (auto &evt : event_buffer_) {
            // evt est une référence (auto &) : on peut modifier evt directement

            // 1) Construire la count image brute (raw) si demandée
            if (!raw_count_image.empty()) {
                int rx = static_cast<int>(evt.x); // x brut
                int ry = static_cast<int>(evt.y); // y brut

                // Vérifier bornes image
                if (ry >= 0 && ry < height_param &&
                    rx >= 0 && rx < width_param) {

                    // Comptage limité à 20 pour éviter saturation
                    if (raw_count_image[ry][rx] < 20) {
                        raw_count_image[ry][rx]++;
                    }
                }
            }

            // 2) Δt depuis t0 en secondes
            //
            // toNs(evt.ts) - t0 => nanosecondes
            // /1e9 => secondes
            //
            float time_diff =
                static_cast<float>((toNs(evt.ts) - t0) / 1e9);

            // 3) Angles de rotation estimés :
            // angle = omega_moyen * Δt
            const float x_angular = time_diff * average_angular_rate_x;
            const float y_angular = time_diff * average_angular_rate_y;
            const float z_angular = time_diff * average_angular_rate_z;

            // 4) Coordonnées centrées sur le centre de l’image
            int x = static_cast<int>(evt.x) - width_param / 2;
            int y = static_cast<int>(evt.y) - height_param / 2;

            // 5) Angles avant compensation (modèle projection pinhole simplifié)
            float pre_x_angle =
                std::atan(y * pixel_size_param / focus_param);
            float pre_y_angle =
                std::atan(x * pixel_size_param / focus_param);

            // 6) Compensation (formules spécifiques)
            //
            // - cos/sin(z_angular) : rotation dans le plan (yaw)
            // - tan(pre_* +/- angle) : correction projection liée à x_angular/y_angular
            //
            int compen_x =
                static_cast<int>(
                    (x * std::cos(z_angular) + std::sin(z_angular) * y)
                    - (x - (focus_param *
                            std::tan(pre_y_angle - y_angular)
                            / pixel_size_param))
                + width_param / 2);

            int compen_y =
                static_cast<int>(
                    (x * std::sin(z_angular) - std::cos(z_angular) * y)
                    - (y - (focus_param *
                            std::tan(pre_x_angle - x_angular)
                            / pixel_size_param))
                + height_param / 2);

            // 7) Mise à jour de l’événement (il devient “compensé”)
            //
            // evt.x et evt.y sont en uint16_t dans le message.
            // On cast, attention : si compen_x est négatif, ça “wrap” si on cast sans check.
            // Ici on cast directement, mais ensuite on ne compte que si dans bornes.
            //
            evt.x = static_cast<uint16_t>(compen_x);
            evt.y = static_cast<uint16_t>(compen_y);

            // 8) Mise à jour des images (compensées) si le pixel est valide
            if (compen_y >= 0 && compen_y < height_param &&
                compen_x >= 0 && compen_x < width_param) {

                // count_image (saturée à 20)
                if (count_image[compen_y][compen_x] < 20) {
                    count_image[compen_y][compen_x]++;
                }

                // time_image : somme des time_diff
                time_image[compen_y][compen_x] += time_diff;
            }
        }

        // ----------------------
        // Normalisation temporelle + calcul de max_count
        // ----------------------
        int max_count = 0;

        for (int i = 0; i < height_param; ++i) {
            for (int j = 0; j < width_param; ++j) {
                if (count_image[i][j] != 0) {
                    // moyenne : somme_time / nb_events
                    time_image[i][j] /=
                        static_cast<float>(count_image[i][j]);

                    // max_count : utilisé pour scale vers [0..255]
                    max_count =
                        std::max(max_count, count_image[i][j]);
                }
            }
        }

        // ----------------------
        // Publication de l’image de comptage principale
        // ----------------------
        show_count_image(count_image, max_count, last_event_header_, image_pub_);

        // ----------------------
        // Publication raw / comp optionnelle
        // ----------------------
        if (!raw_count_image.empty()) {

            // Calcul du max pour raw (pour scaler correctement)
            int raw_max = 0;
            for (int i = 0; i < height_param; ++i) {
                for (int j = 0; j < width_param; ++j) {
                    raw_max = std::max(raw_max, raw_count_image[i][j]);
                }
            }

            // Si on a demandé la publication raw
            if (publish_raw_count_) {
                show_count_image(raw_count_image, raw_max,
                                 last_event_header_, raw_count_pub_);
            }

            // Si on a demandé la publication comp séparée
        }

        // ----------------------
        // Publication time_image en float (32FC1) si demandé
        // ----------------------
        if (publish_time_image_ && time_image_pub_) {

            // Mat float 1 canal : CV_32FC1
            cv::Mat time_mat(height_param, width_param, CV_32FC1);

            // Copier la matrice 2D time_image vers cv::Mat
            for (int i = 0; i < height_param; ++i) {
                for (int j = 0; j < width_param; ++j) {
                    time_mat.at<float>(i, j) = time_image[i][j];
                }
            }

            // Conversion en msg ROS avec encoding "32FC1"
            auto time_msg = cv_bridge::CvImage(
                                last_event_header_, "32FC1", time_mat)
                                .toImageMsg();

            // Publication
            time_image_pub_->publish(*time_msg);
        }

        // ----------------------
        // Publication time_image_vis (visualisation 8 bits) si demandé
        // ----------------------
        if (publish_time_image_vis_ && time_image_vis_pub_) {

            // Trouver la valeur max_time pour scaler vers [0..255]
            float max_time = 0.0f;
            for (int i = 0; i < height_param; ++i) {
                for (int j = 0; j < width_param; ++j) {
                    if (time_image[i][j] > max_time) {
                        max_time = time_image[i][j];
                    }
                }
            }

            // Image de visualisation 8 bits initialisée à 0 (noir)
            cv::Mat time_vis(height_param, width_param, CV_8UC1, cv::Scalar(0));

            // Si max_time > 0, on peut scaler correctement
            if (max_time > 0.0f) {
                const float scale = 255.0f / max_time;

                for (int i = 0; i < height_param; ++i) {
                    for (int j = 0; j < width_param; ++j) {
                        // Valeur = time_image * scale
                        // min(255, ...) pour éviter dépassement
                        time_vis.at<uchar>(i, j) = static_cast<uchar>(
                            std::min(255.0f, time_image[i][j] * scale));
                    }
                }
            }

            // Conversion + publication
            auto vis_msg = cv_bridge::CvImage(
                               last_event_header_, "mono8", time_vis)
                               .toImageMsg();

            time_image_vis_pub_->publish(*vis_msg);
        }

        // ----------------------
        // Nettoyage buffers (prépare le prochain batch)
        // ----------------------
        event_buffer_.clear();
        imu_buffer_snapshot_.clear();
    }


    // ========================================================================
    // Callback événements : appelée automatiquement quand un EventArray arrive
    // ========================================================================
    // - Fonction appelée automatiquement à chaque réception d’un message EventArray.
    // - Si l’alignement est déjà fait :
    //     1) prend un snapshot “stable” des IMU (protégé par mutex) puis vide le buffer IMU entrant,
    //     2) stocke les événements reçus + leur header (timestamp cohérent pour les images),
    //     3) lance le traitement principal (data_process : compensation + génération/publication des images).
    // - Si c’est le premier paquet d’événements :
    //     -> ne traite pas, sert juste à “aligner”/initialiser le pipeline (on ignore ce premier batch).

    void event_cb(const dv_ros2_msgs::msg::EventArray::SharedPtr msg) {

        // Tant que l’alignement n’est pas fait :
        // (ici : first_event_received_ == true => on est au tout début)
        if (!first_event_received_) {

            // 1) Snapshot IMU sécurisé (section critique protégée par mutex)
            //
            // lock_guard : verrouille le mutex à l’entrée du bloc { }
            // et le déverrouille automatiquement à la sortie.
            //
            {
                std::lock_guard<std::mutex> lock(mtx_);
                imu_buffer_snapshot_ = imu_buffer_; // copie IMU accumulées
                imu_buffer_.clear();                // vide le buffer “entrant”
            }

            // Si on n'a pas d’IMU => on ne traite pas
            if (imu_buffer_snapshot_.empty()) {
                return;
            }

            // 2) Ajout événements
            //
            // On mémorise aussi le header du message d’événements,
            // car on veut publier des images avec un timestamp cohérent.
            //
            last_event_header_ = msg->header;

            // reserve : optimise (évite des reallocations si on ajoute beaucoup d’événements)
            event_buffer_.reserve(event_buffer_.size() + msg->events.size());

            // Ajoute tous les événements msg->events dans event_buffer_
            for (const auto &evt : msg->events) {
                event_buffer_.emplace_back(evt);
            }

            // 3) Traitement principal
            data_process();

        } else {
            // Première réception d’événements :
            // on “aligne” en ignorant ce premier paquet (comme dans ton ROS1).
            first_event_received_ = false;

            // On nettoie l'IMU accumulée avant alignement
            imu_buffer_.clear();

            RCLCPP_INFO(get_logger(),
                        "Data aligned! Start processing data...");
        }
    }


    // ========================================================================
    // Callback IMU : appelée automatiquement quand un message IMU arrive
    // ========================================================================
    // - Fonction appelée automatiquement à chaque réception d’un message IMU.
    // - Tant que l’alignement initial n’est pas terminé : on ignore les IMU (pour éviter un décalage au démarrage).
    // - Une fois l’alignement fait : on copie chaque mesure IMU reçue et on l’empile dans imu_buffer_
    //   afin qu’elle puisse être utilisée plus tard par event_cb/data_process pour la compensation de mouvement.

    void imu_cb(const sensor_msgs::msg::Imu::SharedPtr imu) {

        // On ne commence à stocker l’IMU qu’après l’alignement initial
        if (!first_event_received_) {
            // On copie le message IMU (déréférencement *imu) et on l’ajoute au buffer
            imu_buffer_.emplace_back(*imu);
        }
    }


    // ======================
    // Attributs privés (variables membres de la classe)
    // ======================
    // Topics et options
    std::string events_topic_;
    std::string imu_topic_;
    std::string count_image_topic_;
    std::string time_image_topic_;
    std::string time_image_vis_topic_;

    bool publish_time_image_ = true;
    bool publish_time_image_vis_ = true;
    bool publish_raw_count_ = false;

    std::string raw_count_topic_;
    bool sort_events_by_time_ = true;

    // Subscribers (abonnements)
    rclcpp::Subscription<dv_ros2_msgs::msg::EventArray>::SharedPtr event_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

    // Publishers (publications)
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr time_image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr time_image_vis_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr raw_count_pub_;

    // Buffers de données
    std::vector<dv_ros2_msgs::msg::Event> event_buffer_;          // événements à traiter
    std::vector<sensor_msgs::msg::Imu> imu_buffer_;              // IMU “entrant”
    std::vector<sensor_msgs::msg::Imu> imu_buffer_snapshot_;     // IMU “snapshot” (copie stable)

    // Header du dernier paquet d’événements (pour timestamp/frame_id des images publiées)
    std_msgs::msg::Header last_event_header_;

    // Alignement initial
    bool first_event_received_ = true;

    // Comportement de sécurité : exiger IMU plus récent
    bool require_imu_ahead_   = true;

    // Fenêtre IMU (en nanosecondes) utilisée pour la moyenne
    int64_t imu_window_ns_    = 3000000;

    // Mutex : protège imu_buffer_ car callback IMU et callback events
    // peuvent tourner en parallèle (executor multi-thread)
    std::mutex mtx_;
};


// ============================================================================
// Main : point d’entrée du programme
// ============================================================================
int main(int argc, char **argv) {

    // Initialise ROS 2 (obligatoire)
    rclcpp::init(argc, argv);

    // Crée une instance du nœud (shared_ptr)
    auto node = std::make_shared<EventVisualizer>();

    // Executor multi-thread :
    // - permet d’exécuter plusieurs callbacks en parallèle
    // - ici on demande 3 threads
    rclcpp::executors::MultiThreadedExecutor exec(
        rclcpp::ExecutorOptions(), 3);

    // Ajoute le nœud à l’exécuteur
    exec.add_node(node);

    // Boucle principale ROS 2 : écoute les topics et appelle les callbacks
    exec.spin();

    // Shutdown propre de ROS 2
    rclcpp::shutdown();

    return 0;
}
