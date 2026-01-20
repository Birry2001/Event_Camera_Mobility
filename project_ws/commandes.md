
# Commandes de launch — workspace `/home/etudiant/projets/nochi_nadjib_ws`

Ce mémo liste **où sont les packages**, **à quoi ils servent**, et **quels launch lancer** (avec la commande `ros2 launch ...`).

---

## 1) Prérequis (à faire dans **chaque terminal**)

Avant de lancer un node/launch, tu dois sourcer ROS2 + ton workspace :

```bash
source /opt/ros/humble/setup.bash
source /home/etudiant/projets/nochi_nadjib_ws/install/setup.bash
```

> ✅ Si tu oublies : `ros2 launch ...` ne trouvera pas tes packages (ou prendra des versions système).

---

## 2) Packages & Launch files

### aruco_ros

* **Path package** : `/home/etudiant/projets/nochi_nadjib_ws/src/agilex_ws/src/aruco_ros/aruco_ros`
* **Rôle** : détection de **marqueurs ArUco** et publication des **poses**.

Launch disponibles :

* **double.launch.py**

  * Path : `.../launch/double.launch.py`
  * Cmd :

    ```bash
    ros2 launch aruco_ros double.launch.py
    ```
  * Rôle : démo “double configuration” (selon les paramètres du launch).
* **marker_publisher.launch.py**

  * Cmd :

    ```bash
    ros2 launch aruco_ros marker_publisher.launch.py
    ```
  * Rôle : publie des markers pour visualisation (ex: RViz).
* **single.launch.py**

  * Cmd :

    ```bash
    ros2 launch aruco_ros single.launch.py
    ```
  * Rôle : démo simple avec une caméra.

---

### datasync_3_0

* **Path package** : `/home/etudiant/projets/nochi_nadjib_ws/src/datasync_3_0`
* **Rôle** : **motion compensation** sur événements (IMU + events → images compensées).

Launch :

* **motion_compensation.launch.py**

  * Cmd :

    ```bash
    ros2 launch datasync_3_0 motion_compensation.launch.py
    ```
  * Rôle : lance le nœud de compensation de mouvement.

---

### dv_ros2_accumulation

* **Path package** : `/home/etudiant/projets/nochi_nadjib_ws/src/dv-ros2/dv_ros2_accumulation`
* **Rôle** : accumulation d’events → images.

Launch :

* **accumulator.launch.py**

  * Cmd :

    ```bash
    ros2 launch dv_ros2_accumulation accumulator.launch.py
    ```

---

### dv_ros2_capture

* **Path package** : `/home/etudiant/projets/nochi_nadjib_ws/src/dv-ros2/dv_ros2_capture`
* **Rôle** : driver DV (events, frames, IMU, triggers).

Launch :

* **capture.launch.py**

  * Cmd :

    ```bash
    ros2 launch dv_ros2_capture capture.launch.py
    ```
  * Rôle : capture DV (caméra unique).
* **synchronization.launch.py**

  * Cmd :

    ```bash
    ros2 launch dv_ros2_capture synchronization.launch.py
    ```
  * Rôle : synchronisation multi-caméras.

---

### dv_ros2_imu_bias

* **Path package** : `/home/etudiant/projets/nochi_nadjib_ws/src/dv-ros2/dv_ros2_imu_bias`
* **Rôle** : estimation / correction du biais IMU.

Launch :

* **imu_bias.launch.py**

  * Cmd :

    ```bash
    ros2 launch dv_ros2_imu_bias imu_bias.launch.py
    ```

---

### dv_ros2_tracker

* **Path package** : `/home/etudiant/projets/nochi_nadjib_ws/src/dv-ros2/dv_ros2_tracker`
* **Rôle** : tracking (features / events).

Launch :

* **track.launch.py**

  * Cmd :

    ```bash
    ros2 launch dv_ros2_tracker track.launch.py
    ```

---

### dv_ros2_visualization

* **Path package** : `/home/etudiant/projets/nochi_nadjib_ws/src/dv-ros2/dv_ros2_visualization`
* **Rôle** : visualisation des events.

Launch :

* **all.launch.py**

  * Cmd :

    ```bash
    ros2 launch dv_ros2_visualization all.launch.py
    ```
* **visualization.launch.py**

  * Cmd :

    ```bash
    ros2 launch dv_ros2_visualization visualization.launch.py
    ```
* **doge.launch.py**

  * Cmd :

    ```bash
    ros2 launch dv_ros2_visualization doge.launch.py
    ```

---

### dvs_to_dv_bridge

* **Path package** : `/home/etudiant/projets/nochi_nadjib_ws/src/dvs_to_dv_bridge`
* **Rôle** : bridge `dvs_msgs/EventArray` → `dv_ros2_msgs`.

Launch :

* **bridge.launch.py**

  * Cmd :

    ```bash
    ros2 launch dvs_to_dv_bridge bridge.launch.py
    ```

---

### event_clustering

* **Path package** : `/home/etudiant/projets/nochi_nadjib_ws/src/event_clustering`
* **Rôle** : clustering d’events.

Launch :

* **clustering.launch.py**

  * Cmd :

    ```bash
    ros2 launch event_clustering clustering.launch.py
    ```

---

### event_segmentation

* **Path package** : `/home/etudiant/projets/nochi_nadjib_ws/src/event_segmentation`
* **Rôle** : segmentation d’events.

Launch :

* **segmentation.launch.py**

  * Cmd :

    ```bash
    ros2 launch event_segmentation segmentation.launch.py
    ```

---

### event_tf_static

* **Path package** : `/home/etudiant/projets/nochi_nadjib_ws/src/event_tf_static`
* **Rôle** : publication de TF **statiques**.

Launch :

* **static_tf.launch.py**

  * Cmd :

    ```bash
    ros2 launch event_tf_static static_tf.launch.py
    ```

---

### limo_base

* **Path package** : `/home/etudiant/projets/nochi_nadjib_ws/src/agilex_ws/src/limo_ros2/limo_base`
* **Rôle** : driver châssis (odom, status).

Launch :

* **limo_base.launch.py**

  * Cmd :

    ```bash
    ros2 launch limo_base limo_base.launch.py
    ```
  * Rôle : driver châssis (port série + odom).
* **start_limo.launch.py**

  * Cmd :

    ```bash
    ros2 launch limo_base start_limo.launch.py
    ```
  * Rôle : placeholder (contenu commenté).

---

### limo_bringup

* **Path package** : `/home/etudiant/projets/nochi_nadjib_ws/src/agilex_ws/src/limo_ros2/limo_bringup`
* **Rôle** : bringup robot + navigation (Nav2 / SLAM).

Launch principaux (Humble) :

* **limo_start.launch.py**

  * Cmd :

    ```bash
    ros2 launch limo_bringup limo_start.launch.py
    ```
  * Rôle : châssis + lidar + odom (rf2o).
* **bringup.launch.py**

  * Cmd :

    ```bash
    ros2 launch limo_bringup bringup.launch.py
    ```
  * Rôle : bringup Nav2 générique (localisation + navigation).
* **ekf_odom.launch.py**

  * Cmd :

    ```bash
    ros2 launch limo_bringup ekf_odom.launch.py
    ```
  * Rôle : fusion odom (EKF).
* **limo_nav2_diff.launch.py**

  * Cmd :

    ```bash
    ros2 launch limo_bringup limo_nav2_diff.launch.py
    ```
  * Rôle : Nav2 en diff-drive.
* **limo_nav2_ackermann.launch.py**

  * Cmd :

    ```bash
    ros2 launch limo_bringup limo_nav2_ackermann.launch.py
    ```
  * Rôle : Nav2 en Ackermann.
* **limo_rtab_nav2_diff.launch.py**

  * Cmd :

    ```bash
    ros2 launch limo_bringup limo_rtab_nav2_diff.launch.py
    ```
  * Rôle : RTAB-Map + Nav2 diff.
* **limo_rtab_nav2_ackermann.launch.py**

  * Cmd :

    ```bash
    ros2 launch limo_bringup limo_rtab_nav2_ackermann.launch.py
    ```
  * Rôle : RTAB-Map + Nav2 Ackermann.
* **limo_rtab_rgbd.launch.py**

  * Cmd :

    ```bash
    ros2 launch limo_bringup limo_rtab_rgbd.launch.py
    ```
  * Rôle : RTAB-Map RGB-D.
* **limo_rtab_scan.launch.py**

  * Cmd :

    ```bash
    ros2 launch limo_bringup limo_rtab_scan.launch.py
    ```
  * Rôle : RTAB-Map scan-only.
* **limo_cartographer.launch.py**

  * Cmd :

    ```bash
    ros2 launch limo_bringup limo_cartographer.launch.py
    ```
  * Rôle : Cartographer SLAM.
* **limo_slam_box.launch.py**

  * Cmd :

    ```bash
    ros2 launch limo_bringup limo_slam_box.launch.py
    ```
  * Rôle : SLAM custom (launch perso).
* **rgbdslam_datasets.launch.py**

  * Cmd :

    ```bash
    ros2 launch limo_bringup rgbdslam_datasets.launch.py
    ```
  * Rôle : datasets pour tests RGB-D SLAM.

---

### limo_car

* **Path package** : `/home/etudiant/projets/nochi_nadjib_ws/src/agilex_ws/src/limo_car`
* **Rôle** : modèle/simu LIMO (Gazebo), plutôt Ackermann.

Launch :

* **ackermann.launch.py**

  * Cmd :

    ```bash
    ros2 launch limo_car ackermann.launch.py
    ```
  * Rôle : description ackermann (simu).
* **ackermann_gazebo.launch.py**

  * Cmd :

    ```bash
    ros2 launch limo_car ackermann_gazebo.launch.py
    ```
  * Rôle : Gazebo ackermann.
* **display_ackermann.launch.py**

  * Cmd :

    ```bash
    ros2 launch limo_car display_ackermann.launch.py
    ```
  * Rôle : affichage modèle ackermann.

---

### limo_description

* **Path package** : `/home/etudiant/projets/nochi_nadjib_ws/src/agilex_ws/src/limo_ros2/limo_description`
* **Rôle** : description URDF du robot (diff-drive).

Launch :

* **display_models_diff.launch.py**

  * Cmd :

    ```bash
    ros2 launch limo_description display_models_diff.launch.py
    ```
  * Rôle : affichage modèle diff.
* **gazebo_models_diff.launch.py**

  * Cmd :

    ```bash
    ros2 launch limo_description gazebo_models_diff.launch.py
    ```
  * Rôle : Gazebo diff.

---

### nav2_costmap_2d

* **Path package** : `/home/etudiant/projets/nochi_nadjib_ws/src/agilex_ws/src/navigation2/nav2_costmap_2d`
* **Rôle** : exemples/tests costmap Nav2.

Launch :

* **example.launch**

  * Cmd :

    ```bash
    ros2 launch nav2_costmap_2d example.launch
    ```
  * Rôle : exemple costmap (debug).
* **costmap_map_server.launch.py** (test interne)

  * Cmd :

    ```bash
    ros2 launch nav2_costmap_2d costmap_map_server.launch.py
    ```

---

### nav2_map_server

* **Path package** : `/home/etudiant/projets/nochi_nadjib_ws/src/agilex_ws/src/navigation2/nav2_map_server`
* **Rôle** : serveur/sauvegarde de map (Nav2).

Launch :

* **map_saver_server.launch.py**

  * Cmd :

    ```bash
    ros2 launch nav2_map_server map_saver_server.launch.py
    ```
  * Rôle : service de sauvegarde map.
* **map_saver_node.launch.py** (test)

  * Cmd :

    ```bash
    ros2 launch nav2_map_server map_saver_node.launch.py
    ```
* **map_server_node.launch.py** (test)

  * Cmd :

    ```bash
    ros2 launch nav2_map_server map_server_node.launch.py
    ```

---

### nav2_system_tests

* **Path package** : `/home/etudiant/projets/nochi_nadjib_ws/src/agilex_ws/src/navigation2/nav2_system_tests`
* **Rôle** : tests Nav2.

Launch (test) :

* **test_case_py.launch**

  * Cmd :

    ```bash
    ros2 launch nav2_system_tests test_case_py.launch
    ```

---

### orbbec_camera

* **Path package** : `/home/etudiant/projets/nochi_nadjib_ws/src/agilex_ws/src/OrbbecSDK_ROS2/orbbec_camera`
* **Rôle** : driver caméras Orbbec

```bash
ros2 launch orbbec_camera dabai_d1.launch.py serial_number:=CC1S741013D

```

---

### rf2o_laser_odometry

* **Path package** : `/home/etudiant/projets/nochi_nadjib_ws/src/agilex_ws/src/rf2o_laser_odometry`
* **Rôle** : odométrie 2D à partir de `/scan`.

Launch :

* **rf2o_laser_odometry.launch.py**

  * Cmd :

    ```bash
    ros2 launch rf2o_laser_odometry rf2o_laser_odometry.launch.py
    ```

---

### ydlidar_ros2_driver

* **Path package** : `/home/etudiant/projets/nochi_nadjib_ws/src/agilex_ws/src/ydlidar_ros2_driver`
* **Rôle** : driver LiDAR YDLidar.

Launch :

* **ydlidar.launch.py**

  * Cmd :

    ```bash
    ros2 launch ydlidar_ros2_driver ydlidar.launch.py
    ```

---

### yolov5_ros2

* **Path package** : `/home/etudiant/projets/nochi_nadjib_ws/src/agilex_ws/src/yolov5_ros2`
* **Rôle** : détection d’objets (YOLOv5).

Launch :

* **yolov5_ros2.launch.py**

  * Cmd :

    ```bash
    ros2 launch yolov5_ros2 yolov5_ros2.launch.py
    ```
