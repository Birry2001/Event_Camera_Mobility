# LIMO-DAVIS: Mobile Obstacle Detection with an Event Camera (ROS 2)

![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-blue)
![Platform LIMO](https://img.shields.io/badge/Platform-AgileX%20LIMO-orange)
![Sensor DAVIS346](https://img.shields.io/badge/Sensor-DAVIS--346-green)

## 1) Overview

This repository contains a full ROS 2 pipeline for **mobile obstacle detection using an event camera**, inspired by:

- Chunhui Zhao, Yakun Li, Yang Lyu, *Event-based Real-time Moving Object Detection Based on IMU Ego-motion Compensation*, IEEE ICRA 2023.
- DOI: https://doi.org/10.1109/ICRA48891.2023.10160472
- IEEE Xplore: https://ieeexplore.ieee.org/document/10160472

The implemented pipeline includes three main blocks, inspired by the paper:

1. **Motion compensation** (IMU ego-motion) with `datasync_3_0`
2. **Foreground/background segmentation** with `event_segmentation`
3. **Moving object clustering** with `event_clustering_2_0`
4. **Nav2 projection** with `event_nav2_layer` (dynamic costmap layer)

## 2) Global repository content

Main useful directories:

- `project_ws/src/datasync_3_0`: C++ motion compensation node
- `project_ws/src/event_segmentation`: Python segmentation node
- `project_ws/src/event_clustering_2_0`: Python clustering node
- `project_ws/src/event_nav2_layer`: Nav2 costmap plugin (C++)
- `project_ws/src/bringup`: launch for the perception chain
- `project_ws/src/dv-ros2`: essential dependencies for the event camera driver
- `Documentation/`: technical notes and
- `package_motion_compensation_ROS1/Jhonny-Li-Motion-compensation-f6ae84b`: ROS1 reference implementation (migration baseline toward ROS2 `datasync_3_0`)
- `thirdparty/dv-processing-1.7.9`: local copy of the `dv-processing` library

## 3) Dependencies and installation

### 3.1 System prerequisites

- Ubuntu 22.04
- ROS 2 Humble
- `colcon`, `rosdep`, `vcstool`

Base installation:

```bash
sudo apt update
sudo apt install -y \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool \
  python3-pip \
  build-essential cmake git
```

`rosdep` initialization (once per machine):

```bash
sudo rosdep init
rosdep update
```

### 3.2 Workspace ROS dependencies

From `project_ws`, install dependencies declared in `package.xml` files:

```bash
cd /home/nochi/NOCHI/M2_PAR/Projet_de_synthese/project_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 3.3 `dv-ros2` dependencies (essential)

The event format consumed by `datasync_3_0` is `dv_ros2_msgs::msg::EventArray` (provided by `dv-ros2`).

Useful resources:

- `dv-ros2` repository (Telios): https://github.com/Telios/dv-ros2
- `dv-ros2` installation README: https://github.com/Telios/dv-ros2/blob/master/README.md
- `dv-processing` documentation: https://docs.inivation.com/software/dv-processing/index.html
- iniVation PPA (system packages): https://launchpad.net/~inivation-ppa/+archive/ubuntu/inivation

Depending on your setup, you may need to install `dv-processing`/`libcaer` via the iniVation PPA first, then run again:

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

## 5) Run the pipeline

### 5.1 Full perception pipeline (compensation + segmentation + clustering)

```bash
cd /home/nochi/NOCHI/M2_PAR/Projet_de_synthese/project_ws
source install/setup.bash
ros2 launch bringup bringup.launch.py
```

### 5.2 Dynamic Nav2 layer (`event_nav2_layer`)

```bash
cd /home/nochi/NOCHI/M2_PAR/Projet_de_synthese/project_ws
source install/setup.bash
ros2 launch event_nav2_layer event_nav2_layer.launch.py
```

By default, the layer reads the `PointCloud2` topic:

- `/dynamic_obstacles`

(this topic is published by `event_clustering_2_0` in the current version).

### 5.3 Example with rosbag

In one terminal:

```bash
ros2 launch bringup bringup.launch.py
```

In another terminal:

```bash
source /home/nochi/NOCHI/M2_PAR/Projet_de_synthese/project_ws/install/setup.bash
ros2 bag play <bag_path> --clock
```

## 6) Authors

- Nochi Magouo
- Nadjib Mekelleche
