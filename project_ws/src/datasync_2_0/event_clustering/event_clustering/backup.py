import math
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, PointCloud2
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge

# Outils ROS2 pour créer un PointCloud2 à partir d’une liste de points (x,y,z)
from sensor_msgs_py import point_cloud2 as pc2

# cKDTree : structure de voisinage très rapide (utile pour DBSCAN)
from scipy.spatial import cKDTree

# TF2 : pour récupérer la transform (caméra -> base_link) et transformer des points
from tf2_ros import Buffer, TransformListener


def dbscan(points: np.ndarray, eps: float, min_samples: int) -> np.ndarray:
    """
    Implémentation "maison" de DBSCAN (clustering par densité) en 2D/3D.

    Paramètres :
      - points      : tableau (N, D) des points (ici D=2 en pixels)
      - eps         : rayon de voisinage (dans l'unité de points : ici pixels)
      - min_samples : nombre minimal de voisins dans eps pour déclarer un "core point"

    Sortie :
      - labels : tableau (N,) d'entiers
          -1  = bruit (noise)
          0..K-1 = id de cluster
    """
    # Cas trivial : aucun point
    if points.size == 0:
        return np.array([], dtype=np.int32)

    # KD-tree pour accélérer la recherche de voisins dans un rayon eps
    tree = cKDTree(points)

    # labels initialisés à -1 (tout est bruit tant qu'on n'a pas créé de cluster)
    labels = -np.ones(points.shape[0], dtype=np.int32)

    # visited : pour ne pas re-traiter plusieurs fois le même point
    visited = np.zeros(points.shape[0], dtype=bool)

    cluster_id = 0

    # Parcours de tous les points
    for idx in range(points.shape[0]):
        if visited[idx]:
            continue

        visited[idx] = True

        # Indices des voisins dans le rayon eps autour du point idx
        neighbors = tree.query_ball_point(points[idx], eps)

        # Si pas assez de voisins -> point considéré comme bruit
        if len(neighbors) < min_samples:
            labels[idx] = -1
            continue

        # Sinon : on démarre un nouveau cluster
        labels[idx] = cluster_id

        # "seeds" contient les points à explorer (expansion du cluster)
        seeds = list(neighbors)

        # Expansion : on "propage" le cluster aux voisins denses
        while seeds:
            current = seeds.pop()

            # Si pas encore visité, on marque visité et on récupère ses voisins
            if not visited[current]:
                visited[current] = True
                current_neighbors = tree.query_ball_point(points[current], eps)

                # Si current est aussi un core point (assez de voisins),
                # alors on ajoute ses voisins à la liste d’exploration
                if len(current_neighbors) >= min_samples:
                    seeds.extend(current_neighbors)

            # Si current était du bruit (-1), on le rattache au cluster
            if labels[current] == -1:
                labels[current] = cluster_id

        cluster_id += 1

    return labels


class EventClusteringNode(Node):
    """
    Node ROS2 qui :
      1) reçoit un masque binaire d'évènements (image mono8)
      2) extrait les pixels actifs (mask != 0)
      3) clusterise ces pixels avec DBSCAN (en pixels)
      4) projette chaque cluster en 3D (centroïde ou points) dans output_frame
         - soit via une depth image
         - soit via intersection avec un plan de sol (z = plane_z)
         - soit via un simple scaling pixels->mètres (fallback)
      5) publie :
         - un PointCloud2 (points des obstacles dynamiques)
         - un MarkerArray (sphères au centroïde de chaque cluster)
    """

    def __init__(self):
        super().__init__("event_clustering")

        # -----------------------------
        # Paramètres ROS
        # -----------------------------
        self.declare_parameter("mask_topic", "/event_mask")                    # entrée : masque binaire (mono8)
        self.declare_parameter("pointcloud_topic", "/dynamic_obstacles")       # sortie : PointCloud2
        self.declare_parameter("marker_topic", "/dynamic_obstacles_markers")   # sortie : MarkerArray

        # Paramètres DBSCAN (unité : pixels)
        self.declare_parameter("eps_pixels", 4.0)     # rayon voisinage DBSCAN en pixels
        self.declare_parameter("min_samples", 20)     # densité minimale pour un cluster

        # Pour éviter un coût énorme si le masque est très dense
        self.declare_parameter("max_points", 15000)

        # Fallback projection : conversion pixel->m en XY sur un plan z=0
        self.declare_parameter("pixel_scale_m", 0.01)

        # Projection plan de sol via TF + intrinsics
        self.declare_parameter("use_ground_plane", True)
        self.declare_parameter("camera_frame", "event_camera")  # frame optique/caméra
        self.declare_parameter("output_frame", "base_link")     # frame de sortie (Nav2, robot)

        # Intrinsics caméra (modèle pinhole) : pour convertir (u,v) -> rayon 3D
        self.declare_parameter("fx", 300.0)
        self.declare_parameter("fy", 300.0)
        self.declare_parameter("cx", 173.0)
        self.declare_parameter("cy", 130.0)

        # Distorsion radiale/tangentielle (k1,k2,k3,p1,p2) optionnelle
        self.declare_parameter("k1", 0.0)
        self.declare_parameter("k2", 0.0)
        self.declare_parameter("p1", 0.0)
        self.declare_parameter("p2", 0.0)
        self.declare_parameter("k3", 0.0)
        self.declare_parameter("undistort", True)
        self.declare_parameter("undistort_iterations", 5)

        # Plan de sol dans la frame output : z = plane_z
        self.declare_parameter("plane_z", 0.0)

        # TF : utiliser le TF le plus récent ou celui au timestamp du message
        self.declare_parameter("use_latest_tf", True)

        # Publication : uniquement les centroïdes (plus léger) ou tous les points du cluster
        self.declare_parameter("publish_centroids_only", True)

        # Logs debug
        self.declare_parameter("log_stats", False)

        # Option depth : si tu as une image de profondeur alignée (APS depth, etc.)
        self.declare_parameter("use_depth", False)
        self.declare_parameter("depth_topic", "/aps_depth")
        self.declare_parameter("depth_scale", 1.0)  # facteur multiplicatif (ex: mm->m)
        self.declare_parameter("depth_min", 0.1)
        self.declare_parameter("depth_max", 10.0)
        self.declare_parameter("depth_use_median", True)  # depth du cluster = médiane (robuste)

        # -----------------------------
        # Lecture paramètres
        # -----------------------------
        mask_topic = self.get_parameter("mask_topic").get_parameter_value().string_value
        pc_topic = self.get_parameter("pointcloud_topic").get_parameter_value().string_value
        marker_topic = self.get_parameter("marker_topic").get_parameter_value().string_value

        self._eps_pixels = self.get_parameter("eps_pixels").get_parameter_value().double_value
        self._min_samples = self.get_parameter("min_samples").get_parameter_value().integer_value
        self._max_points = self.get_parameter("max_points").get_parameter_value().integer_value
        self._pixel_scale_m = self.get_parameter("pixel_scale_m").get_parameter_value().double_value

        self._use_ground_plane = self.get_parameter("use_ground_plane").get_parameter_value().bool_value
        self._camera_frame = self.get_parameter("camera_frame").get_parameter_value().string_value
        self._output_frame = self.get_parameter("output_frame").get_parameter_value().string_value

        self._fx = self.get_parameter("fx").get_parameter_value().double_value
        self._fy = self.get_parameter("fy").get_parameter_value().double_value
        self._cx = self.get_parameter("cx").get_parameter_value().double_value
        self._cy = self.get_parameter("cy").get_parameter_value().double_value

        self._k1 = self.get_parameter("k1").get_parameter_value().double_value
        self._k2 = self.get_parameter("k2").get_parameter_value().double_value
        self._p1 = self.get_parameter("p1").get_parameter_value().double_value
        self._p2 = self.get_parameter("p2").get_parameter_value().double_value
        self._k3 = self.get_parameter("k3").get_parameter_value().double_value

        self._undistort = self.get_parameter("undistort").get_parameter_value().bool_value
        self._undistort_iterations = self.get_parameter("undistort_iterations").get_parameter_value().integer_value

        self._plane_z = self.get_parameter("plane_z").get_parameter_value().double_value
        self._use_latest_tf = self.get_parameter("use_latest_tf").get_parameter_value().bool_value

        self._centroids_only = self.get_parameter("publish_centroids_only").get_parameter_value().bool_value
        self._log_stats = self.get_parameter("log_stats").get_parameter_value().bool_value

        self._use_depth = self.get_parameter("use_depth").get_parameter_value().bool_value
        self._depth_topic = self.get_parameter("depth_topic").get_parameter_value().string_value
        self._depth_scale = self.get_parameter("depth_scale").get_parameter_value().double_value
        self._depth_min = self.get_parameter("depth_min").get_parameter_value().double_value
        self._depth_max = self.get_parameter("depth_max").get_parameter_value().double_value
        self._depth_use_median = self.get_parameter("depth_use_median").get_parameter_value().bool_value

        # -----------------------------
        # I/O ROS : bridge, TF, pubs/subs
        # -----------------------------
        self._bridge = CvBridge()

        # Buffer TF2 : stocke l’arbre des transforms
        self._tf_buffer = Buffer()
        # Listener : remplit le buffer automatiquement via /tf et /tf_static
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # Publishers : nuage de points + markers
        self._pc_pub = self.create_publisher(PointCloud2, pc_topic, 10)
        self._marker_pub = self.create_publisher(MarkerArray, marker_topic, 10)

        # Subscriber masque
        self.create_subscription(Image, mask_topic, self._on_mask, 10)

        # Subscriber depth optionnel
        self._depth_image = None
        if self._use_depth:
            self.create_subscription(Image, self._depth_topic, self._on_depth, 10)

    def _on_depth(self, msg: Image) -> None:
        """
        Callback depth : stocke la dernière depth image reçue.
        IMPORTANT : ici on ne synchronise pas avec le masque -> on prend la dernière dispo.
        """
        depth_img = self._bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")
        self._depth_image = depth_img

    def _on_mask(self, msg: Image) -> None:
        """
        Callback masque :
          - extrait les pixels actifs
          - DBSCAN en pixels
          - projection 3D + publication nuage + markers
        """
        mask = self._bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")
        if mask is None:
            return

        # Récupère les indices (y,x) des pixels non-nuls (mask != 0)
        ys, xs = np.nonzero(mask)

        # Aucun pixel actif -> publier des sorties vides
        if ys.size == 0:
            if self._log_stats:
                self.get_logger().info("clusters: mask empty")
            self._publish_empty(msg.header)
            return

        # Si trop de pixels -> sous-échantillonnage aléatoire pour limiter CPU
        if ys.size > self._max_points:
            idx = np.random.choice(ys.size, self._max_points, replace=False)
            ys = ys[idx]
            xs = xs[idx]

        # Matrice (N,2) des points pixels : [u=x, v=y]
        points_px = np.column_stack((xs, ys)).astype(np.float32)

        # Clustering DBSCAN en pixels
        labels = dbscan(points_px, self._eps_pixels, self._min_samples)

        # Si dbscan renvoie vide -> rien à publier
        if labels.size == 0:
            if self._log_stats:
                self.get_logger().info("clusters: no labels")
            self._publish_empty(msg.header)
            return

        cloud_points = []      # liste de points (x,y,z) en mètres dans output_frame
        markers = MarkerArray()

        # On réutilise le header de l'image masque, mais on force frame_id = output_frame
        header = msg.header
        header.frame_id = self._output_frame

        # Labels uniques sauf le bruit (-1)
        unique_labels = [l for l in np.unique(labels) if l >= 0]
        if not unique_labels:
            if self._log_stats:
                self.get_logger().info("clusters: no clusters after filtering")
            self._publish_empty(header)
            return

        # Parcours des clusters
        for i, label in enumerate(unique_labels):
            idx = labels == label
            cluster_px = points_px[idx]  # points pixels du cluster
            if cluster_px.size == 0:
                continue

            # Centroïde en pixels (moyenne des u,v)
            centroid_px = np.mean(cluster_px, axis=0)

            # Projection du centroïde en mètres (si possible)
            centroid_m = self._pixel_to_meters(centroid_px, cluster_px, mask.shape, header)
            if centroid_m is None:
                # si impossible (pas de TF, pas d’intersection sol, depth invalide, etc.)
                continue

            # Publication pointcloud : soit seulement centroïdes, soit tous les points du cluster
            if self._centroids_only:
                cloud_points.append(centroid_m)
            else:
                for p in cluster_px:
                    point_m = self._pixel_to_meters(p, cluster_px, mask.shape, header)
                    if point_m is not None:
                        cloud_points.append(point_m)

            # Marker RViz : une sphère au centroïde
            marker = Marker()
            marker.header = header
            marker.ns = "event_clusters"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = float(centroid_m[0])
            marker.pose.position.y = float(centroid_m[1])
            marker.pose.position.z = float(centroid_m[2])
            marker.pose.orientation.w = 1.0

            # Taille de la sphère (m)
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1

            # Couleur (RGBA)
            marker.color.r = 0.1
            marker.color.g = 0.9
            marker.color.b = 0.1
            marker.color.a = 0.8

            markers.markers.append(marker)

        # Construit le PointCloud2 et publie
        cloud_msg = pc2.create_cloud_xyz32(header, cloud_points)
        self._pc_pub.publish(cloud_msg)

        # Publie les markers
        self._marker_pub.publish(markers)

        if self._log_stats:
            self.get_logger().info(
                f"clusters: mask_points={ys.size} clusters={len(unique_labels)} "
                f"cloud_points={len(cloud_points)}"
            )

    def _publish_empty(self, header) -> None:
        """
        Publie :
          - un PointCloud2 vide
          - un MarkerArray vide
        Utile quand il n'y a aucun cluster ou que la donnée est invalide.
        """
        header.frame_id = self._output_frame
        cloud_msg = pc2.create_cloud_xyz32(header, [])
        self._pc_pub.publish(cloud_msg)
        self._marker_pub.publish(MarkerArray())

    def _pixel_to_meters(self, pixel_xy: np.ndarray, cluster_px, shape, header):
        """
        Convertit un pixel (u,v) en un point 3D (x,y,z) dans output_frame.

        Stratégie (priorité) :
          1) Si use_depth et depth dispo : back-projection pinhole + TF
          2) Sinon si use_ground_plane : intersection du rayon caméra avec le plan z=plane_z + TF
          3) Sinon : simple mise à l'échelle pixel->m (fallback sans TF)
        """
        # 1) Depth : donne un z réaliste si depth correcte
        if self._use_depth and self._depth_image is not None:
            point = self._pixel_to_depth(pixel_xy, cluster_px, shape, header)
            if point is not None:
                return point

        # 2) Plan de sol : suppose que l'obstacle est sur le plan z=plane_z (approx)
        if self._use_ground_plane:
            point = self._pixel_to_ground(pixel_xy, header)
            if point is not None:
                return point

        # 3) Fallback : "plan image" scalé (pas géométriquement fidèle)
        return self._pixel_to_scaled(pixel_xy, shape)

    def _pixel_to_depth(self, pixel_xy: np.ndarray, cluster_px, shape, header):
        """
        Projection via depth :
          - lit une profondeur (médiane sur le cluster ou pixel centre)
          - convertit (u,v,depth) -> (x,y,z) dans frame caméra via intrinsics
          - transforme ce point dans output_frame via TF (caméra -> base_link)
        """
        # Vérifications de cohérence
        if self._depth_image is None:
            return None
        if self._depth_image.shape != shape:
            return None

        # Choix de la profondeur :
        # - médiane des profondeurs du cluster (robuste au bruit / trous)
        # - ou profondeur au pixel centroid
        if self._depth_use_median and cluster_px is not None and cluster_px.size > 0:
            xs = cluster_px[:, 0].astype(np.int32)
            ys = cluster_px[:, 1].astype(np.int32)

            depths = self._depth_image[ys, xs]
            depths = depths[np.isfinite(depths)]
            if depths.size == 0:
                return None
            depth = float(np.median(depths))
        else:
            u = int(pixel_xy[0])
            v = int(pixel_xy[1])
            depth = float(self._depth_image[v, u])

        # Rejette les NaN/Inf
        if not np.isfinite(depth):
            return None

        # Applique l'échelle (si depth en mm -> depth_scale=0.001 par ex.)
        depth *= float(self._depth_scale)

        # Filtrage des profondeurs hors plage (capteur)
        if depth < self._depth_min or depth > self._depth_max:
            return None

        # Back-projection pinhole :
        # x = (u - cx)/fx * z
        # y = (v - cy)/fy * z
        # z = depth
        x = (pixel_xy[0] - self._cx) / self._fx * depth
        y = (pixel_xy[1] - self._cy) / self._fy * depth
        z = depth

        # Transforme ce point depuis la frame caméra vers output_frame
        return self._transform_point((x, y, z), header)

    def _transform_point(self, point_xyz, header):
        """
        Transforme un point exprimé dans camera_frame vers output_frame,
        en utilisant TF2 (rotation quaternion + translation).
        """
        try:
            # Soit on prend le TF le plus récent (mode "latest")
            if self._use_latest_tf:
                transform = self._tf_buffer.lookup_transform(
                    self._output_frame, self._camera_frame, rclpy.time.Time()
                )
            else:
                # Soit on prend le TF au timestamp du message (plus cohérent, parfois indisponible)
                transform = self._tf_buffer.lookup_transform(
                    self._output_frame,
                    self._camera_frame,
                    rclpy.time.Time.from_msg(header.stamp),
                )
        except Exception as ex:  # TF manquant / buffer vide / extrapolation
            if self._log_stats:
                self.get_logger().warn(f"TF lookup failed for depth: {ex}")
            return None

        # Décompose la transform : translation (t) + quaternion (q)
        tx = transform.transform.translation.x
        ty = transform.transform.translation.y
        tz = transform.transform.translation.z

        qx = transform.transform.rotation.x
        qy = transform.transform.rotation.y
        qz = transform.transform.rotation.z
        qw = transform.transform.rotation.w

        # Quaternion -> matrice de rotation 3x3
        r = self._quat_to_rot(qx, qy, qz, qw)

        # Applique la rotation + translation : out = R * p + t
        cam_point = np.array(point_xyz, dtype=np.float32)
        out = r.dot(cam_point) + np.array([tx, ty, tz], dtype=np.float32)
        return float(out[0]), float(out[1]), float(out[2])

    def _pixel_to_scaled(self, pixel_xy: np.ndarray, shape) -> tuple:
        """
        Fallback très simple : suppose que l'image est un plan et convertit les pixels en mètres
        autour du centre image, avec un facteur pixel_scale_m.
        -> Ce n'est PAS une vraie projection 3D, juste un mapping 2D.
        """
        height, width = shape
        x = (pixel_xy[0] - (width / 2.0)) * self._pixel_scale_m
        y = (pixel_xy[1] - (height / 2.0)) * self._pixel_scale_m
        return float(x), float(y), 0.0

    def _pixel_to_ground(self, pixel_xy: np.ndarray, header):
        """
        Projection sur un plan de sol z=plane_z (dans output_frame) :

          1) construit le rayon caméra correspondant au pixel (u,v) :
             ray_cam = [(u-cx)/fx, (v-cy)/fy, 1]
             (+ correction distorsion optionnelle)
          2) transforme ce rayon dans output_frame via TF (rotation seulement)
          3) intersecte la demi-droite (origine caméra + t * ray_out) avec le plan z=plane_z

        Retour :
          (x,y,z) dans output_frame ou None si :
            - TF indisponible
            - rayon parallèle au plan
            - intersection derrière la caméra
            - rayon pointe vers le haut (ray_out[2] >= 0)
        """
        # Récupère TF caméra -> output
        try:
            if self._use_latest_tf:
                transform = self._tf_buffer.lookup_transform(
                    self._output_frame, self._camera_frame, rclpy.time.Time()
                )
            else:
                transform = self._tf_buffer.lookup_transform(
                    self._output_frame,
                    self._camera_frame,
                    rclpy.time.Time.from_msg(header.stamp),
                )
        except Exception as ex:
            # Ici le code choisit de "warn" puis de retourner None (et donc fallback scaling)
            self.get_logger().warn(f"TF lookup failed, using scale fallback: {ex}")
            return None

        # Position de la caméra dans output_frame
        tx = transform.transform.translation.x
        ty = transform.transform.translation.y
        tz = transform.transform.translation.z

        # Orientation caméra -> output
        qx = transform.transform.rotation.x
        qy = transform.transform.rotation.y
        qz = transform.transform.rotation.z
        qw = transform.transform.rotation.w

        # Matrice de rotation
        r = self._quat_to_rot(qx, qy, qz, qw)

        # Rayon dans la frame caméra (pinhole)
        ray_cam = np.array(
            [
                (pixel_xy[0] - self._cx) / self._fx,
                (pixel_xy[1] - self._cy) / self._fy,
                1.0,
            ],
            dtype=np.float32,
        )

        # Correction de distorsion (optionnel) sur les coordonnées normalisées (x/z, y/z)
        if self._undistort:
            xu, yu = self._undistort_normalized(ray_cam[0], ray_cam[1])
            ray_cam[0] = xu
            ray_cam[1] = yu

        # Transforme le rayon dans output_frame (rotation seulement)
        ray_out = r.dot(ray_cam)

        # Si le rayon ne va pas "vers le sol" (ici on s'attend à z négatif), on abandonne.
        # (convention : souvent la caméra "voit vers l'avant", donc dépend de tes frames !)
        if abs(ray_out[2]) < 1e-6 or ray_out[2] >= 0.0:
            return None

        # Intersecte avec le plan z = plane_z :
        # tz + t*ray_out[2] = plane_z  -> t = (plane_z - tz) / ray_out[2]
        t = (self._plane_z - tz) / ray_out[2]
        if t <= 0.0:
            # intersection derrière la caméra
            return None

        # Point d'intersection
        x = tx + t * ray_out[0]
        y = ty + t * ray_out[1]
        z = self._plane_z
        return float(x), float(y), float(z)

    @staticmethod
    def _quat_to_rot(qx, qy, qz, qw) -> np.ndarray:
        """
        Convertit un quaternion (x,y,z,w) en matrice de rotation 3x3.
        (Formule standard)
        """
        r00 = 1.0 - 2.0 * (qy * qy + qz * qz)
        r01 = 2.0 * (qx * qy - qz * qw)
        r02 = 2.0 * (qx * qz + qy * qw)
        r10 = 2.0 * (qx * qy + qz * qw)
        r11 = 1.0 - 2.0 * (qx * qx + qz * qz)
        r12 = 2.0 * (qy * qz - qx * qw)
        r20 = 2.0 * (qx * qz - qy * qw)
        r21 = 2.0 * (qy * qz + qx * qw)
        r22 = 1.0 - 2.0 * (qx * qx + qy * qy)
        return np.array([[r00, r01, r02], [r10, r11, r12], [r20, r21, r22]], dtype=np.float32)

    def _undistort_normalized(self, xd: float, yd: float) -> tuple:
        """
        "Undistortion" itératif sur coordonnées normalisées (x = (u-cx)/fx, y = (v-cy)/fy).

        But :
          - Si ton modèle caméra inclut distorsion (k1,k2,k3,p1,p2),
            on veut approx. retrouver (xu,yu) non-distordus tels que :
              (xd,yd) = distort(xu,yu)

        Méthode :
          - On initialise (xu,yu) = (xd,yd)
          - On itère en inversant numériquement le modèle de distorsion
        """
        xu = float(xd)
        yu = float(yd)

        for _ in range(max(1, self._undistort_iterations)):
            r2 = xu * xu + yu * yu
            r4 = r2 * r2
            r6 = r4 * r2

            # facteur radial (1 + k1*r^2 + k2*r^4 + k3*r^6)
            radial = 1.0 + self._k1 * r2 + self._k2 * r4 + self._k3 * r6

            # distorsion tangentielle (p1, p2)
            dx = 2.0 * self._p1 * xu * yu + self._p2 * (r2 + 2.0 * xu * xu)
            dy = self._p1 * (r2 + 2.0 * yu * yu) + 2.0 * self._p2 * xu * yu

            # évite division par zéro
            if radial == 0.0:
                break

            # Mise à jour : approx inverse du modèle
            xu = (xd - dx) / radial
            yu = (yd - dy) / radial

        return xu, yu


def main() -> None:
    """Entrée du node ROS2."""
    rclpy.init()
    node = EventClusteringNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

