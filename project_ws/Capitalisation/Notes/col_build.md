
## Commande propre à répéter  
*(elle reprend où ça s’est arrêté)*

```bash
source /opt/ros/humble/setup.bash
export CMAKE_PREFIX_PATH=/home/nochi/NOCHI/M2_PAR/Projet_de_synthese/thirdparty/dv-processing-1.7.9/install:$CMAKE_PREFIX_PATH
cd /home/nochi/NOCHI/M2_PAR/Projet_de_synthese/project_ws
colcon build --symlink-install --packages-skip dv_ros2_runtime_modules --allow-overriding nav2_map_server
```

---

## Rapport / recommandations build

### 1) dv-ros2 / dv-processing

* Toujours exporter `CMAKE_PREFIX_PATH` vers
  `thirdparty/dv-processing-1.7.9/install`
* Toujours *skip* `dv_ros2_runtime_modules`
  *(incompatible avec dv-processing 2.x installé en `/usr/local`)*
* Pour éviter les avertissements Boost/PkgConfig, laisser tel quel
  *(non bloquant)*

---

### 2) Nav2 (conflit d’overlay)

* `nav2_map_server` existe déjà dans `/opt/ros/humble`
* Utiliser `--allow-overriding nav2_map_server` si tu veux ton overlay

---

### 3) Gros packages à builder séparément (gain de temps)

Pour itérer vite sur le code :

```bash
colcon build --packages-select datasync_3_0 dvs_to_dv_bridge
```

Pour tout Nav2 :

```bash
colcon build --packages-up-to nav2_bt_navigator
```

---

### 4) Limiter la charge si ça timeout

Réduis le parallélisme :

```bash
colcon build --parallel-workers 2
```

---

### 5) Orbbec camera

Si un jour ça échoue sur warnings / `-Werror`, builder avec GCC-11 :

```bash
CC=gcc-11 CXX=g++-11 colcon build --packages-select orbbec_camera
```

---

### 6) Logs lisibles

```bash
colcon build --event-handlers console_cohesion+
```



