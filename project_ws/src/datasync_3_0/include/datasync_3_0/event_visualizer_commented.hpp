/*******************************************************
 *  FICHIER .H / .HPP (EN-TÊTE) : EventVisualizer
 *
 *  EXPLICATION GLOBALE (à lire en premier)
 *  --------------------------------------
 *  Ce fichier contient la *déclaration* de la classe EventVisualizer.
 *  Il ne contient PAS le code complet des fonctions (ça sera dans un .cpp).
 *
 *  Rôle global de EventVisualizer (tel qu'on l’a vu dans le .cpp) :
 *   - C’est un nœud ROS 2 (hérite de rclcpp::Node)
 *   - Il s’abonne à :
 *       * un topic d’événements (EventArray) venant de la DAVIS
 *       * un topic IMU (sensor_msgs::Imu)
 *   - Il stocke ces données dans des buffers (vectors)
 *   - Il traite les données (data_process) :
 *       * calcule une moyenne de vitesse angulaire IMU
 *       * compense la position des événements
 *       * génère des images (count image, time image...)
 *   - Il publie des images ROS (sensor_msgs::Image)
 *
 *  BUT d’un header :
 *   - Dire au compilateur “voici ce que la classe contient”
 *   - Pour que d’autres fichiers puissent utiliser EventVisualizer
 *   - Séparer :
 *       * Déclarations (dans .hpp)
 *       * Définitions (implémentations) (dans .cpp)
 *******************************************************/


#pragma once
// #pragma once = directive du compilateur
// Elle dit : "n’inclus ce fichier qu’une seule fois dans la compilation"
// Cela évite les erreurs de double définition dues à plusieurs inclusions.

// ----------------------
// Includes C++ standard
// ----------------------
#include <cstdint> // types entiers fixes : int64_t, uint16_t...
#include <mutex>   // std::mutex (protection multi-thread)
#include <string>  // std::string (chaînes de caractères)
#include <vector>  // std::vector (tableau dynamique)

// ----------------------
// Includes ROS 2
// ----------------------
#include <rclcpp/rclcpp.hpp>            // classe Node, publishers, subscribers...
#include <sensor_msgs/msg/imu.hpp>      // message IMU standard
#include <sensor_msgs/msg/image.hpp>    // message Image standard
#include <std_msgs/msg/header.hpp>      // Header (stamp + frame_id)

// ----------------------
// Messages événements DAVIS (dv_ros2)
// ----------------------
#include <dv_ros2_msgs/msg/event.hpp>        // message Event (x,y,ts,polarity...)
#include <dv_ros2_msgs/msg/event_array.hpp>  // message EventArray (header + liste)

// ============================================================================
// Déclaration de la classe
// ============================================================================
//
// "class EventVisualizer : public rclcpp::Node"
// - On déclare une classe C++
// - Elle hérite (:) de rclcpp::Node
//   => donc EventVisualizer EST un Node ROS 2
//   => elle possède toutes les méthodes d’un Node (create_publisher, get_logger, etc.)
//
class EventVisualizer : public rclcpp::Node {
public:
    // ======================
    // Partie "publique"
    // ======================

    // Constructeur :
    // - même nom que la classe
    // - pas de type de retour
    // - ici : EventVisualizer(); (sans arguments)
    //
    // Ce constructeur sera défini dans le .cpp.
    // Il va typiquement :
    // - appeler Node("datasync_node")
    // - déclarer/charger les paramètres
    // - créer subscriptions et publishers
    //
    EventVisualizer();

private:
    // ======================
    // Partie "privée"
    // ======================
    //
    // "private" = accessible seulement à l’intérieur de la classe.
    // L’extérieur ne peut pas appeler ces fonctions ni modifier ces variables directement.
    //
    // C’est bien : ça protège l’état interne de ton nœud.

    // ------------------------------------------------------------------------
    // show_count_image
    // ------------------------------------------------------------------------
    // Cette fonction prend une image de comptage (matrice 2D d'int) et la publie.
    //
    // Paramètres :
    // - count_image : matrice [H][W] avec le nombre d'événements par pixel
    // - max_count   : max(count_image) pour normaliser vers 0..255
    // - header      : header à mettre dans le message publié (timestamp/frame_id)
    // - pub         : publisher ROS 2 sur lequel on publie
    //
    // "const std::vector<std::vector<int>> &" :
    // - const : on ne modifie pas la matrice
    // - & : on évite de recopier (sinon ce serait lourd)
    //
    // "SharedPtr" :
    // - pointer partagé (smart pointer) utilisé partout en ROS 2
    //
    void show_count_image(const std::vector<std::vector<int>> &count_image,
                          int max_count,
                          const std_msgs::msg::Header &header,
                          const rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr &pub);

    // ------------------------------------------------------------------------
    // data_process
    // ------------------------------------------------------------------------
    // Fonction “coeur” :
    // - utilise event_buffer_ et imu_buffer_snapshot_
    // - calcule la compensation
    // - construit count_image / time_image
    // - publie selon les options
    //
    void data_process();

    // ------------------------------------------------------------------------
    // event_cb
    // ------------------------------------------------------------------------
    // Callback événements :
    // - appelée quand un EventArray arrive
    // - empile les events dans event_buffer_
    // - récupère un snapshot IMU
    // - appelle data_process()
    //
    // dv_ros2_msgs::msg::EventArray::SharedPtr msg :
    // - msg est un pointeur partagé vers le message reçu
    //
    void event_cb(const dv_ros2_msgs::msg::EventArray::SharedPtr msg);

    // ------------------------------------------------------------------------
    // imu_cb
    // ------------------------------------------------------------------------
    // Callback IMU :
    // - appelée quand un message IMU arrive
    // - stocke l’IMU dans imu_buffer_
    //
    void imu_cb(const sensor_msgs::msg::Imu::SharedPtr imu);

    // ------------------------------------------------------------------------
    // toNs (fonction utilitaire)
    // ------------------------------------------------------------------------
    // "static" :
    // - appartient à la classe (pas à un objet particulier)
    // - on peut l’appeler sans instance : EventVisualizer::toNs(...)
    //
    // Convertit un builtin_interfaces::msg::Time en nanosecondes int64
    //
    static int64_t toNs(const builtin_interfaces::msg::Time &t);


    // ========================================================================
    // ATTRIBUTS (variables membres)
    // ========================================================================

    // ----------------------
    // Noms de topics (strings)
    // ----------------------
    std::string events_topic_;        // ex: "events"
    std::string imu_topic_;           // ex: "imu"
    std::string count_image_topic_;   // ex: "count_image"
    std::string time_image_topic_;    // ex: "time_image"
    std::string time_image_vis_topic_;// ex: "time_image_vis"

    // ----------------------
    // Options de publication / comportement
    // ----------------------
    //
    // "= true/false" : valeur par défaut
    // Si on ne change pas via paramètres, ces valeurs restent celles-ci.
    //
    bool publish_time_image_ = true;       // publier la time image (float)
    bool publish_time_image_vis_ = true;   // publier la version visualisable (8 bits)
    bool publish_raw_count_ = false;       // publier count image brute
    bool publish_comp_count_ = false;      // publier count image compensée sur topic séparé

    std::string raw_count_topic_;          // topic de count brute
    std::string comp_count_topic_;         // topic de count compensée

    bool sort_events_by_time_ = true;      // trier les events par timestamp avant traitement

    // ----------------------
    // ROS 2 : Subscriptions
    // ----------------------
    //
    // SharedPtr = pointeur partagé.
    // On garde ces objets en membres pour qu'ils restent vivants.
    //
    rclcpp::Subscription<dv_ros2_msgs::msg::EventArray>::SharedPtr event_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

    // ----------------------
    // ROS 2 : Publishers
    // ----------------------
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;         // count_image principal
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr time_image_pub_;    // time_image float
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr time_image_vis_pub_;// time_image en 8 bits
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr raw_count_pub_;     // count brute
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr comp_count_pub_;    // count compensée séparée

    // ----------------------
    // Buffers de données
    // ----------------------
    //
    // On accumule des données car les callbacks arrivent en continu.
    //
    std::vector<dv_ros2_msgs::msg::Event> event_buffer_;     // events à traiter
    std::vector<sensor_msgs::msg::Imu> imu_buffer_;         // IMU “entrant” (rempli par imu_cb)
    std::vector<sensor_msgs::msg::Imu> imu_buffer_snapshot_;// copie stable (utilisée par data_process)

    // ----------------------
    // Dernier header d’événements
    // ----------------------
    //
    // On veut réutiliser le timestamp du dernier paquet d’événements
    // pour publier des images avec un stamp cohérent.
    //
    std_msgs::msg::Header last_event_header_;

    // ----------------------
    // Alignement initial
    // ----------------------
    //
    // Au début on ignore le tout premier paquet d’événements
    // pour être sûr que IMU et events commencent “propres”.
    //
    bool first_event_received_ = true;

    // ----------------------
    // Sécurité temporelle
    // ----------------------
    //
    // require_imu_ahead_ :
    // - si true, on exige que la fin des IMU soit plus récente
    //   que la fin des événements (pour être sûr qu’on a de l’IMU couvrante)
    //
    bool require_imu_ahead_ = true;

    // Fenêtre temporelle IMU (en nanosecondes)
    // Exemple : 3 000 000 ns = 3 ms
    int64_t imu_window_ns_ = 3000000;

    // ----------------------
    // Mutex (sécurité multi-thread)
    // ----------------------
    //
    // Comme on peut utiliser un MultiThreadedExecutor,
    // event_cb et imu_cb peuvent s’exécuter en parallèle.
    // => on doit protéger les buffers partagés (surtout imu_buffer_)
    //
    std::mutex mtx_;
};
