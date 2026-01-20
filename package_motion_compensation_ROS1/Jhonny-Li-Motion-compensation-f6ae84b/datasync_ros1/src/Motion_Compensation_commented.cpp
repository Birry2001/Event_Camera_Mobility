/*******************************************************
 *  Ce fichier C++ est un noeud ROS (ROS1) qui :
 *   1) s'abonne à un topic d'événements DVS : "/dvs/events"
 *   2) s'abonne à un topic IMU : "/dvs/imu"
 *   3) stocke temporairement (buffer) les événements et les IMU
 *   4) calcule une vitesse angulaire moyenne (sur une petite fenêtre IMU)
 *   5) fait une "motion compensation" : corrige la position (x,y) des événements
 *      en supposant que le mouvement est une rotation (issue de l'IMU)
 *   6) construit une "count image" (image de comptage) : combien d'événements par pixel
 *   7) publie cette image sur "/count_image" en tant que sensor_msgs/Image
 *
 *  BUT PÉDAGOGIQUE :
 *  - comprendre ROS (subscribe/publish), buffers, callbacks, mutex
 *  - comprendre comment une image OpenCV est construite puis envoyée en ROS
 *  - comprendre la logique globale du pipeline (events + IMU -> image compensée)
 *******************************************************/

#include <ros/ros.h>               // ROS1 : init, NodeHandle, spin, Publisher/Subscriber, etc.
#include <algorithm>              // std::max (et autres algorithmes)
#include <cmath>                  // fonctions math : sqrt, sin, cos, atan, tan...
#include <cfloat>                 // constantes float (pas utilisé ici explicitement)

#include <iostream>               // std::cout, std::endl
#include <fstream>                // fichiers (pas utilisé ici dans le code donné)

#include <cv_bridge/cv_bridge.h>  // pont entre OpenCV (cv::Mat) et ROS (sensor_msgs/Image)
#include <opencv2/core/core.hpp>  // types OpenCV de base : cv::Mat
#include <opencv2/highgui/highgui.hpp> // affichage / GUI OpenCV (pas utilisé ici)
#include <opencv2/imgproc/imgproc.hpp> // traitement image (pas utilisé ici explicitement)
#include <opencv2/imgproc.hpp>    // idem (souvent redondant)
#include <image_transport/image_transport.h> // transport d'images ROS (pas utilisé ici)

#include <dvs_msgs/Event.h>       // message ROS pour un événement DVS (x,y,ts,polarity...)
#include <dvs_msgs/EventArray.h>  // message ROS contenant un tableau d'événements
#include <sensor_msgs/Imu.h>      // message ROS IMU

// #include <sensor_msgs/PointCloud.h> // commenté : pas utilisé
#include <sensor_msgs/image_encodings.h> // encodages d'images ROS ("mono8", "bgr8"...)
#include "time.h"                 // temps C (peu utile ici, plutôt redondant avec ROS)
#include <boost/thread.hpp>       // threads boost (pas utilisé ici)
#include <mutex>                  // std::mutex (pour protéger des données partagées)

using namespace std;              // évite d'écrire std:: partout (pratique mais parfois déconseillé)

/*******************************************************
 * typedef : crée un alias de type.
 * Ici sll = "signed long long int" (entier 64-bit signé)
 * -> souvent utilisé pour manipuler des timestamps en nanosecondes.
 *******************************************************/
typedef long long int sll;

/*******************************************************
 * VARIABLES GLOBALES
 * (elles vivent pendant toute l'exécution du programme)
 *
 * Attention : les globales simplifient, mais peuvent rendre le code
 * plus difficile à maintenir. Ici, elles servent de buffers.
 *******************************************************/

// Buffer des événements : on accumule les événements reçus avant traitement
std::vector<dvs_msgs::Event> event_buffer;

// Buffer IMU "entrant" : rempli dans le callback imu_cb
std::vector<sensor_msgs::Imu> imu_buffer;

// Buffer IMU "copié" : on copie imu_buffer vers imu_buffer_ avant traitement,
// puis on vide imu_buffer. Cela permet d'isoler les données traitées.
std::vector<sensor_msgs::Imu> imu_buffer_;

// Flag : au début on ignore la première salve d'événements pour "aligner" les buffers
bool first_event_received = true;

// Paramètres "input" (souvent fournis via rosparam)
int height_;        // hauteur image (en pixels)
int weight_;        // largeur image (en pixels) -> note : ici c'est "weight_" mais ça veut dire "width_"
float Focus_;       // focale (valeur utilisée dans les formules)
float pixel_size_;  // taille physique d'un pixel (ex : micromètres, selon calibrage)

/*******************************************************
 * CLASSE PRINCIPALE
 * Un noeud ROS est souvent encapsulé dans une classe :
 * - elle possède ses Publisher/Subscriber
 * - elle gère l'état interne (buffers, mutex, paramètres)
 *******************************************************/
class EventVisualizer {
protected:
    ros::NodeHandle n_;      // "handle" ROS : permet de créer pubs/subs, accéder params, etc.

    // Publisher : pour publier une image sur un topic
    ros::Publisher image_pub;

    // Subscribers : pour recevoir events et imu
    ros::Subscriber event_sub;
    ros::Subscriber imu_sub;

    // Mutex : protège l'accès concurrent aux buffers (thread safety)
    std::mutex mtx;

public:
    /*******************************************************
     * CONSTRUCTEUR
     * EventVisualizer(ros::NodeHandle n) : n_(n)   <-- liste d'initialisation
     * - n_(n) signifie : on initialise le membre n_ avec la valeur n.
     *******************************************************/
    EventVisualizer(ros::NodeHandle n)
        : n_(n) {

        // subscribe(topic, queue_size, callback, this)
        // queue_size = 1 : on veut les données les plus récentes (peu de buffer ROS)
        this->event_sub = this->n_.subscribe(
            "/dvs/events", 1,
            &EventVisualizer::event_cb, // méthode membre comme callback
            this                           // pointeur sur l'objet courant
        );

        // queue_size = 7 pour l'IMU : on laisse un peu plus d'IMU en file
        this->imu_sub = this->n_.subscribe(
            "/dvs/imu", 7,
            &EventVisualizer::imu_cb,
            this
        );

        // advertise<MsgType>(topic, queue_size)
        // -> on annonce qu'on va publier un sensor_msgs::Image sur /count_image
        this->image_pub = n_.advertise<sensor_msgs::Image>("/count_image", 1);
    }

    // Déclaration d'une fonction membre : affiche/publie l'image de comptage
    void show_count_image(std::vector<std::vector<int>>& count_image, int& max_count);

    // Fonction principale de traitement (motion compensation + count image)
    void data_process();

    /*******************************************************
     * CALLBACK EVENTS
     * Elle est appelée automatiquement par ROS à chaque message EventArray reçu.
     *
     * ConstPtr : pointeur constant partagé vers le message ROS
     * -> msg->events est un vector d'événements
     *******************************************************/
    void event_cb(const dvs_msgs::EventArray::ConstPtr& msg) {

        // Si ce n'est pas la toute première salve
        if (first_event_received == false) {

            /*******************************************************
             * SECTION CRITIQUE : on copie imu_buffer vers imu_buffer_
             * pendant que d'autres threads peuvent pousser de l'IMU
             * => on verrouille le mutex.
             *******************************************************/
            mtx.lock();

            // Copie : imu_buffer_ devient une copie de imu_buffer
            imu_buffer_ = imu_buffer;

            // Si imu_buffer n'est pas vide, on le vide (clear)
            if (imu_buffer.size() != 0) {
                imu_buffer.clear();
            }

            // On déverrouille le mutex
            mtx.unlock();

            // Si après copie il n'y a pas d'IMU : impossible de compenser -> on sort
            if (imu_buffer_.size() == 0) {
                return;
            }

            // On ajoute tous les événements du message dans event_buffer
            for (int i = 0; i < msg->events.size(); ++i) {
                // emplace_back : ajoute un élément à la fin (souvent plus efficace)
                event_buffer.emplace_back(msg->events[i]);
            }

            // Appel de la fonction centrale
            data_process();

        } else {
            /*******************************************************
             * Premier message d'événements : on s'en sert pour "aligner"
             * et on ne traite pas (on évite un mauvais couplage events/imu).
             *******************************************************/
            first_event_received = false; // on passe en mode traitement ensuite

            // On vide tout IMU déjà accumulé (sinon décalage temporel)
            if (imu_buffer.size() != 0) {
                imu_buffer.clear();
            }

            std::cout << "Data aligned!" << std::endl;
            std::cout << "Start processing data..." << std::endl;
        }
    }

    /*******************************************************
     * CALLBACK IMU
     * Appelée à chaque message IMU.
     * On stocke l'IMU dans imu_buffer si on a déjà passé l'étape d'alignement.
     *******************************************************/
    void imu_cb(const sensor_msgs::ImuConstPtr& imu) {
        if (first_event_received == false) {
            // On copie le message IMU (*imu) et on le push dans le buffer
            imu_buffer.emplace_back(*imu);
        }
    }
}; // fin classe


/*******************************************************
 * show_count_image :
 * Transforme count_image (vector 2D d'int) en cv::Mat 8-bit
 * puis convertit en sensor_msgs/Image et publie.
 *******************************************************/
void EventVisualizer::show_count_image(std::vector<std::vector<int>>& count_image, int& max_count) {
    using namespace cv; // permet d'écrire Mat au lieu de cv::Mat si on veut

    // Création d'une image OpenCV :
    // Mat(rows, cols, type)
    // CV_8UC1 = 8-bit unsigned (0..255), 1 channel (grayscale)
    cv::Mat image(height_, weight_, CV_8UC1);

    // On veut mapper les counts vers [0..255]
    // scale approx = 255 / max_count
    // +1 pour éviter scale=0 si max_count > 255 etc., et éviter division entière nulle
    int scale = (int)(255 / max_count) + 1;

    // Parcours pixel par pixel
    for (int i = 0; i < height_; ++i) {
        for (int j = 0; j < weight_; ++j) {
            // image.at<uchar>(row,col) : accès à un pixel (uchar = unsigned char)
            image.at<uchar>(i, j) = count_image[i][j] * scale;
        }
    }

    // Conversion OpenCV -> ROS Image :
    // CvImage(header, encoding, mat).toImageMsg()
    sensor_msgs::ImagePtr msg2 =
        cv_bridge::CvImage(std_msgs::Header(), "mono8", image).toImageMsg();

    // Publication du message sur /count_image
    image_pub.publish(*msg2);
}


/*******************************************************
 * data_process :
 * Coeur de l'algorithme.
 * - Vérifie qu'on a assez d'IMU par rapport aux événements.
 * - Calcule une vitesse angulaire moyenne à partir de l'IMU.
 * - Compense (corrige) la position x,y de chaque événement.
 * - Construit une count_image et une time_image.
 * - Publie l'image.
 *******************************************************/
void EventVisualizer::data_process() {

    // Sécurité : on compare le timestamp du dernier IMU au timestamp du premier événement
    // imu_buffer_[last].stamp > event_buffer[0].ts
    // -> si le dernier IMU est plus récent que le premier event, on a une fenêtre IMU couvrante.
    if (imu_buffer_[imu_buffer_.size() - 1].header.stamp.toNSec() > event_buffer[0].ts.toNSec()) {

        // Sommes des vitesses angulaires (rad/s) sur une petite fenêtre
        float angular_velocity_x = 0.0, angular_velocity_y = 0.0, angular_velocity_z = 0.0;

        // Moyennes qui seront calculées
        float average_angular_rate_x, average_angular_rate_y, average_angular_rate_z;

        int cnt = 0; // compteur IMU utilisée dans la moyenne

        // On parcourt les IMU copiées dans imu_buffer_
        for (int i = 0; i < imu_buffer_.size(); ++i) {

            // On ne garde que les IMU dont le timestamp est dans une fenêtre :
            // >= (timestamp_du_premier_event - 3 000 000 ns)
            // 3 000 000 ns = 3 ms
            if (imu_buffer_[i].header.stamp.toNSec() >= (event_buffer[0].ts.toNSec() - 3000000)) {
                angular_velocity_x += imu_buffer_[i].angular_velocity.x;
                angular_velocity_y += imu_buffer_[i].angular_velocity.y;
                angular_velocity_z += imu_buffer_[i].angular_velocity.z;
                cnt++;
            }
        }

        // Calcul des moyennes (attention : division par cnt, on suppose cnt>0)
        average_angular_rate_x = angular_velocity_x / float(cnt);
        average_angular_rate_y = angular_velocity_y / float(cnt);
        average_angular_rate_z = angular_velocity_z / float(cnt);

        // Norme (magnitude) du vecteur vitesse angulaire moyenne
        float average_angular_rate = std::sqrt(
            (average_angular_rate_x * average_angular_rate_x) +
            (average_angular_rate_y * average_angular_rate_y) +
            (average_angular_rate_z * average_angular_rate_z)
        );
        // NOTE : average_angular_rate n'est pas utilisé ensuite dans ce code.

        /*******************************************************
         * Motion compensation :
         * On prend comme référence le premier événement à t0.
         * Pour chaque événement i à temps t :
         *   time_diff = (t - t0) en secondes
         * Puis on estime la rotation accumulée : angle = time_diff * omega_moyenne
         *******************************************************/

        sll t0 = event_buffer[0].ts.toNSec(); // timestamp du premier event (ns)
        float time_diff = 0.0;                // différence de temps en secondes

        // count_image : matrice [height_][weight_] initialisée à 0
        std::vector<std::vector<int>> count_image(height_, std::vector<int>(weight_));

        // time_image : matrice float, on y accumule le temps moyen par pixel
        std::vector<std::vector<float>> time_image(height_, std::vector<float>(weight_));

        // Parcours de tous les événements accumulés
        for (int i = 0; i < event_buffer.size(); ++i) {

            // conversion ns -> secondes : / 1e9
            time_diff = double(event_buffer[i].ts.toNSec() - t0) / 1000000000.0;

            // Angles de rotation estimés sur chaque axe
            float x_angular = time_diff * average_angular_rate_x;
            float y_angular = time_diff * average_angular_rate_y;
            float z_angular = time_diff * average_angular_rate_z;

            // On centre les coordonnées autour de (0,0) au centre de l'image
            // (x,y) ici deviennent des coordonnées relatives au centre
            int x = event_buffer[i].x - weight_ / 2;
            int y = event_buffer[i].y - height_ / 2;

            // Angles de la position initiale (projection pinhole simplifiée)
            // atan( (coord_physique) / focale )
            float pre_x_angel = atan(y * pixel_size_ / Focus_);
            float pre_y_angel = atan(x * pixel_size_ / Focus_);

            /*******************************************************
             * Compensation :
             * compen_x et compen_y calculent la nouvelle position
             * après correction de rotation (z) + correction liée au modèle pinhole
             *
             * IMPORTANT : ces formules sont spécifiques à l'approche utilisée.
             * Ici on applique cos/sin pour la rotation en z, puis des tan/atan
             * pour corriger l'effet sur l'image via focale et pixel_size_.
             *******************************************************/

            int compen_x = (int)(
                (x * cos(z_angular) - sin(z_angular) * y)
                - (x - (Focus_ * tan(pre_y_angel + y_angular) / pixel_size_))
                + weight_ / 2
            );

            int compen_y = (int)(
                (x * sin(z_angular) + cos(z_angular) * y)
                - (y - (Focus_ * tan(pre_x_angel - x_angular) / pixel_size_))
                + height_ / 2
            );

            // On écrase les coordonnées de l'événement avec les coordonnées compensées
            event_buffer[i].x = compen_x;
            event_buffer[i].y = compen_y;

            // On met à jour les images (count/time) seulement si le pixel est dans l'image
            if (compen_y < height_ && compen_y >= 0 && compen_x < weight_ && compen_x >= 0) {

                // Incrémente le compteur au pixel, mais limite à 20 (saturation)
                if (count_image[compen_y][compen_x] < 20)
                    count_image[compen_y][compen_x]++;

                // On accumule time_diff pour ensuite faire une moyenne
                time_image[compen_y][compen_x] += time_diff;
            }
        }

        // Variables de stats (beaucoup ne sont pas utilisées ensuite)
        int max_count = 0;
        float max_time = 0.0;
        float total_time = 0.0;
        float average_time = 0.0;
        int trigger_pixels = 0;

        // Post-traitement : moyenne du time_image par pixel (si count != 0)
        // et calcul de max_count pour le scaling 0..255
        for (int i = 0; i < height_; ++i) {
            for (int j = 0; j < weight_; ++j) {
                if (count_image[i][j] != 0) {
                    time_image[i][j] /= count_image[i][j];           // moyenne temps
                    max_count = std::max(max_count, count_image[i][j]); // max du count
                }
            }
        }

        // Publication de la count image compensée
        show_count_image(count_image, max_count);

        // On libère les buffers pour le prochain batch
        event_buffer.clear();
        imu_buffer_.clear();

    } else {
        /*******************************************************
         * Cas "pas assez d'IMU" pour couvrir les événements :
         * on préfère jeter les buffers plutôt que faire un traitement incohérent.
         *******************************************************/
        if (event_buffer.size() != 0) {
            event_buffer.clear();
        }
        if (imu_buffer_.size() != 0) {
            imu_buffer_.clear();
        }
    }
}


/*******************************************************
 * MAIN
 * Point d'entrée du programme C++.
 * ROS appelle init, crée le noeud, puis attend.
 *******************************************************/
int main(int argc, char** argv) {

    // Initialise ROS (doit être fait avant toute utilisation ROS)
    ros::init(argc, argv, "datasync_node");

    // NodeHandle "global" (namespace public)
    ros::NodeHandle nh;

    // Création de l'objet principal (installe pubs/subs)
    EventVisualizer visualizer(nh);

    // NodeHandle privé "~" : paramètres dans le namespace du noeud
    ros::NodeHandle nh_priv("~");

    /*******************************************************
     * Lecture des paramètres :
     * param<Type>("nom", variable, valeur_par_defaut)
     *******************************************************/
    nh_priv.param<int>("weight_param", weight_, 346);       // largeur par défaut = 346
    nh_priv.param<int>("height_param", height_, 260);       // hauteur par défaut = 260
    nh_priv.param<float>("focus", Focus_, 6550);            // focale par défaut
    nh_priv.param<float>("pixel_size", pixel_size_, 18.5);  // taille pixel par défaut

    /*******************************************************
     * AsyncSpinner :
     * - permet à ROS d'appeler les callbacks dans plusieurs threads.
     * - ici 3 threads => event_cb et imu_cb peuvent être appelés en parallèle.
     *
     * C'est POUR ÇA qu'on a un mutex pour protéger imu_buffer.
     *******************************************************/
    ros::AsyncSpinner spinner(3); // Use 3 threads
    spinner.start();

    // Bloque jusqu'à l'arrêt (Ctrl+C, rosnode kill...)
    ros::waitForShutdown();

    return 0;
}
