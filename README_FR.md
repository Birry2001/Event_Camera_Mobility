# LIMO-DAVIS : Détection d'obstacles mobiles par caméra à évènements (ROS 2)

![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-blue)
![Platform LIMO](https://img.shields.io/badge/Platform-AgileX%20LIMO-orange)
![Sensor DAVIS346](https://img.shields.io/badge/Sensor-DAVIS--346-green)

## 1) Vue d'ensemble

Ce dépôt contient un pipeline ROS 2 complet de **détection d'obstacles mobiles à l'aide d'une caméra à évènements**, inspiré de l'article:

- Chunhui Zhao, Yakun Li, Yang Lyu, *Event-based Real-time Moving Object Detection Based on IMU Ego-motion Compensation*, IEEE ICRA 2023.
- DOI: https://doi.org/10.1109/ICRA48891.2023.10160472
- IEEE Xplore: https://ieeexplore.ieee.org/document/10160472

Le pipeline implémenté comprend trois blocs principaux, inspirés de l'article:

1. **Motion compensation** (ego-motion IMU) via `datasync_3_0`
2. **Segmentation** foreground/background via `event_segmentation`
3. **Clustering** des objets mobiles via `event_clustering_2_0`
4. **Projection Nav2** via `event_nav2_layer` (costmap layer dynamique)

## 2) Contenu global du dépôt

Principaux dossiers utiles:

- `project_ws/src/datasync_3_0`: noeud C++ de motion compensation
- `project_ws/src/event_segmentation`: noeud Python de segmentation
- `project_ws/src/event_clustering_2_0`: noeud Python de clustering
- `project_ws/src/event_nav2_layer`: plugin Nav2 costmap (C++)
- `project_ws/src/bringup`: launch de la chaîne perception
- `project_ws/src/dv-ros2`: dépendances essentielles pour le driver de la caméra à évènements
- `Documentation/`: notes techniques et 
- `package_motion_compensation_ROS1/Jhonny-Li-Motion-compensation-f6ae84b`: implémentation ROS1 de référence (base de migration vers la version ROS2 `datasync_3_0`)
- `thirdparty/dv-processing-1.7.9`: bibliothèque `dv-processing` conservée localement 

## 3) Dépendances et installation 

### 3.1 Prérequis système

- Ubuntu 22.04
- ROS 2 Humble
- `colcon`, `rosdep`, `vcstool`

Installation de base :

```bash
sudo apt update
sudo apt install -y \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool \
  python3-pip \
  build-essential cmake git
```

Initialisation `rosdep` (une seule fois sur la machine):

```bash
sudo rosdep init
rosdep update
```

### 3.2 Dépendances ROS du workspace

Depuis `project_ws`, installer automatiquement les dépendances déclarées dans les `package.xml`:

```bash
cd /home/nochi/NOCHI/M2_PAR/Projet_de_synthese/project_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 3.3 Dépendances `dv-ros2` (essentielles)

Le format d'évènements consommé par `datasync_3_0` repose sur `dv_ros2_msgs::msg::EventArray` (fourni par `dv-ros2`).

Ressources utiles:

- Dépôt `dv-ros2` (Telios): https://github.com/Telios/dv-ros2
- README d'installation `dv-ros2`: https://github.com/Telios/dv-ros2/blob/master/README.md
- Documentation `dv-processing`: https://docs.inivation.com/software/dv-processing/index.html
- PPA iniVation (packages système): https://launchpad.net/~inivation-ppa/+archive/ubuntu/inivation

Selon votre environnement, vous pouvez avoir besoin d'installer `dv-processing`/`libcaer` via le PPA iniVation (voir liens ci-dessus), puis relancer:

```bash
rosdep install --from-paths src --ignore-src -r -y
```

## 4) Build

```bash
cd /home/nochi/NOCHI/M2_PAR/Projet_de_synthese/project_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to bringup event_nav2_layer
source install/setup.bash
```

## 5) Lancement du pipeline

### 5.1 Pipeline de perception complet (compensation + segmentation + clustering)

```bash
cd /home/nochi/NOCHI/M2_PAR/Projet_de_synthese/project_ws
source install/setup.bash
ros2 launch bringup bringup.launch.py
```

### 5.2 Couche Nav2 dynamique (`event_nav2_layer`)

```bash
cd /home/nochi/NOCHI/M2_PAR/Projet_de_synthese/project_ws
source install/setup.bash
ros2 launch event_nav2_layer event_nav2_layer.launch.py
```

Le layer lit par défaut le topic `PointCloud2`:

- `/dynamic_obstacles`

(ce topic est publié par `event_clustering_2_0` dans la version actuelle).

### 5.3 Exemple avec rosbag

Dans un terminal:

```bash
ros2 launch bringup bringup.launch.py
```

Dans un autre terminal:

```bash
source /home/nochi/NOCHI/M2_PAR/Projet_de_synthese/project_ws/install/setup.bash
ros2 bag play <chemin_du_bag> --clock
```


## 6) Auteurs

- Nochi Magouo
- Nadjib Mekelleche
