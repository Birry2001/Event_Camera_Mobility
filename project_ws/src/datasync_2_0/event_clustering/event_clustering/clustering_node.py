#!/usr/bin/env python3
"""Noeud ROS2 de clustering par composantes connexes sur masque d'evenements.

Pipeline:
1) Lecture d'une image binaire (`/event_mask`).
2) Liaison spatiale optionnelle (dilatation) pour reconnecter des points proches.
3) Extraction des composantes connexes.
4) Filtrage du bruit:
   - filtre spatial minimal (`min_area`)
   - filtre temporel de persistance pour les petites composantes
5) Publication d'une image couleur ne contenant que les contours retenus.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


def color_from_id(k: int) -> tuple:
    """Couleur déterministe BGR (OpenCV)."""
    b = (29 * (k + 1)) % 255
    g = (17 * (k + 1)) % 255
    r = (37 * (k + 1)) % 255
    return (b, g, r)


class ObjectContoursFromMask(Node):
    """Construit des contours d'objets à partir d'un masque d'evenements.

    Le noeud ne fait pas de tracking d'objet complet. Le tracking interne sert
    uniquement a filtrer les petites composantes fugitives ("grain de sable")
    en imposant une persistance sur quelques frames.
    """

    def __init__(self):
        super().__init__("object_contours_from_mask")

        # Topics
        self.declare_parameter("mask_topic", "/event_mask")
        self.declare_parameter("contours_topic", "/object_contours")
        self.declare_parameter("edges_topic", "/object_edges")   # debug optionnel
        self.declare_parameter("publish_edges", False)

        # Liaison spatiale douce
        self.declare_parameter("use_dilate", True)
        self.declare_parameter("kernel", 3)          # 3 recommandé
        self.declare_parameter("kernel_shape", "ellipse")  # "ellipse" ou "rect"
        self.declare_parameter("dilate_iters", 1)    # 1 recommandé

        # Connected components / filtre bruit spatial
        self.declare_parameter("connectivity", 8)    # 8 recommandé
        self.declare_parameter("min_area", 10)       # augmente si trop de bruit

        # Filtre temporel anti-bruit:
        # - applique uniquement aux composantes de petite taille
        # - exige une persistance minimale en nombre de frames
        self.declare_parameter("enable_temporal_noise_filter", True)
        self.declare_parameter("temporal_small_area_max", 80)
        self.declare_parameter("temporal_min_persist_frames", 2)
        self.declare_parameter("temporal_match_distance", 8.0)
        self.declare_parameter("temporal_track_ttl_frames", 2)

        # Contours
        self.declare_parameter("contour_thickness", 2)
        self.declare_parameter("chain_approx", True)  # True = CHAIN_APPROX_SIMPLE

        self._mask_topic = str(self.get_parameter("mask_topic").value)
        self._contours_topic = str(self.get_parameter("contours_topic").value)
        self._edges_topic = str(self.get_parameter("edges_topic").value)
        self._publish_edges = bool(self.get_parameter("publish_edges").value)

        self._use_dilate = bool(self.get_parameter("use_dilate").value)
        self._kernel = int(self.get_parameter("kernel").value)
        self._kernel_shape = str(self.get_parameter("kernel_shape").value).lower()
        self._dilate_iters = int(self.get_parameter("dilate_iters").value)

        self._conn = int(self.get_parameter("connectivity").value)
        self._min_area = int(self.get_parameter("min_area").value)
        self._enable_temporal_noise_filter = bool(
            self.get_parameter("enable_temporal_noise_filter").value
        )
        self._temporal_small_area_max = int(self.get_parameter("temporal_small_area_max").value)
        self._temporal_min_persist_frames = int(
            self.get_parameter("temporal_min_persist_frames").value
        )
        self._temporal_match_distance = float(self.get_parameter("temporal_match_distance").value)
        self._temporal_track_ttl_frames = int(
            self.get_parameter("temporal_track_ttl_frames").value
        )

        self._th = int(self.get_parameter("contour_thickness").value)
        self._chain_approx = bool(self.get_parameter("chain_approx").value)

        # Safety
        if self._conn not in (4, 8):
            self._conn = 8
        self._kernel = max(1, self._kernel)
        self._dilate_iters = max(0, self._dilate_iters)
        self._min_area = max(1, self._min_area)
        self._temporal_small_area_max = max(self._min_area, self._temporal_small_area_max)
        self._temporal_min_persist_frames = max(1, self._temporal_min_persist_frames)
        self._temporal_match_distance = max(0.5, self._temporal_match_distance)
        self._temporal_track_ttl_frames = max(1, self._temporal_track_ttl_frames)
        self._th = max(1, self._th)

        # Etat du filtre temporel:
        # track_id -> {"cx", "cy", "age", "ttl"}
        self._tracks = {}
        self._next_track_id = 0

        self._bridge = CvBridge()
        self._pub_contours = self.create_publisher(Image, self._contours_topic, 10)
        self._pub_edges = self.create_publisher(Image, self._edges_topic, 10)

        self.create_subscription(Image, self._mask_topic, self._on_mask, 10)

        self.get_logger().info(
            f"ObjectContoursFromMask prêt. Sub {self._mask_topic} | Pub {self._contours_topic}"
        )

    def _make_kernel(self):
        """Construit le noyau morphologique utilise pour la dilatation/gradient."""
        if self._kernel_shape == "rect":
            return cv2.getStructuringElement(cv2.MORPH_RECT, (self._kernel, self._kernel))
        # ellipse par défaut (meilleure forme)
        return cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (self._kernel, self._kernel))

    def _on_mask(self, msg: Image) -> None:
        """Callback principale executee a chaque masque entrant.

        Le coeur du traitement se fait en 2 etapes:
        - construction des composantes connexes candidates
        - selection des composantes a conserver (filtre spatial + temporel)
        """
        mask = self._bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")
        if mask is None:
            return

        H, W = mask.shape

        # binaire 0/255
        bin255 = np.where(mask > 0, 255, 0).astype(np.uint8)

        # Liaison spatiale douce (connecter les points proches sans "gonfler" trop)
        if self._use_dilate and self._dilate_iters > 0:
            k = self._make_kernel()
            bin255 = cv2.dilate(bin255, k, iterations=self._dilate_iters)

        # (Optionnel debug) edges bruts via gradient morphologique
        if self._publish_edges:
            k = self._make_kernel()
            edge = cv2.morphologyEx(bin255, cv2.MORPH_GRADIENT, k)
            edge_bgr = np.full((H, W, 3), 255, dtype=np.uint8)
            edge_bgr[edge > 0] = (0, 0, 0)
            out_e = self._bridge.cv2_to_imgmsg(edge_bgr, encoding="bgr8")
            out_e.header = msg.header
            self._pub_edges.publish(out_e)

        # Connected components sur 0/1
        bin01 = (bin255 > 0).astype(np.uint8)
        num, labels, stats, centroids = cv2.connectedComponentsWithStats(
            bin01, connectivity=self._conn
        )

        # Sortie contours colorés
        out = np.full((H, W, 3), 255, dtype=np.uint8)

        mode = cv2.CHAIN_APPROX_SIMPLE if self._chain_approx else cv2.CHAIN_APPROX_NONE

        # `comp_keep[lab] = True` signifie "composante acceptee pour rendu".
        comp_keep = self._compute_kept_components(num, stats, centroids)
        kept = 0
        for lab in range(1, num):
            if not comp_keep[lab]:
                continue

            kept += 1
            color = color_from_id(lab)

            comp = np.where(labels == lab, 255, 0).astype(np.uint8)
            contours, _ = cv2.findContours(comp, cv2.RETR_EXTERNAL, mode)
            if not contours:
                continue

            # Dessin du contour (pas de remplissage)
            cv2.drawContours(out, contours, -1, color, thickness=self._th)

        out_msg = self._bridge.cv2_to_imgmsg(out, encoding="bgr8")
        out_msg.header = msg.header
        self._pub_contours.publish(out_msg)

        # (debug léger si tu veux)
        # self.get_logger().info(f"labels={num} kept={kept} min_area={self._min_area}")

    def _compute_kept_components(self, num, stats, centroids):
        """Retourne un masque booleen des labels a conserver.

        Regles:
        - toujours rejeter les composantes trop petites (`min_area`)
        - garder immediatement les composantes assez grandes
        - pour les petites composantes restantes, exiger une persistance
          temporelle via un mini-tracking centre-sur-centroide
        """
        keep = np.zeros(num, dtype=bool)
        if num <= 1:
            # Rien a matcher sur cette frame: on vieillit les tracks existants.
            self._age_tracks(set())
            return keep

        if not self._enable_temporal_noise_filter:
            for lab in range(1, num):
                keep[lab] = int(stats[lab, cv2.CC_STAT_AREA]) >= self._min_area
            # Si le filtre temporel est coupe, on nettoie son etat interne.
            self._tracks.clear()
            return keep

        active_track_ids = set()
        used_track_ids = set()
        small_components = []

        for lab in range(1, num):
            area = int(stats[lab, cv2.CC_STAT_AREA])
            if area < self._min_area:
                continue

            if area > self._temporal_small_area_max:
                # Les gros objets sont acceptes sans attendre.
                keep[lab] = True
                continue

            cx, cy = float(centroids[lab][0]), float(centroids[lab][1])
            small_components.append((lab, cx, cy))

        for lab, cx, cy in small_components:
            # Associe la composante a un track proche existant, sinon nouveau track.
            track_id = self._match_track(cx, cy, used_track_ids)
            if track_id is None:
                track_id = self._next_track_id
                self._next_track_id += 1
                self._tracks[track_id] = {
                    "cx": cx,
                    "cy": cy,
                    "age": 1,
                    "ttl": self._temporal_track_ttl_frames,
                }
            else:
                tr = self._tracks[track_id]
                tr["cx"] = cx
                tr["cy"] = cy
                tr["age"] += 1
                tr["ttl"] = self._temporal_track_ttl_frames

            used_track_ids.add(track_id)
            active_track_ids.add(track_id)

            # La composante n'est gardee qu'une fois assez persistante.
            if self._tracks[track_id]["age"] >= self._temporal_min_persist_frames:
                keep[lab] = True

        self._age_tracks(active_track_ids)
        return keep

    def _match_track(self, cx, cy, used_track_ids):
        """Trouve le track le plus proche sous un seuil de distance.

        Renvoie l'identifiant du meilleur track compatible, ou `None`.
        """
        best_id = None
        best_dist2 = self._temporal_match_distance * self._temporal_match_distance

        for track_id, tr in self._tracks.items():
            if track_id in used_track_ids:
                continue

            dx = tr["cx"] - cx
            dy = tr["cy"] - cy
            dist2 = dx * dx + dy * dy
            if dist2 <= best_dist2:
                best_dist2 = dist2
                best_id = track_id

        return best_id

    def _age_tracks(self, active_track_ids):
        """Vieillit les tracks absents sur la frame puis supprime les expires."""
        to_delete = []
        for track_id, tr in self._tracks.items():
            if track_id in active_track_ids:
                continue

            tr["ttl"] -= 1
            if tr["ttl"] <= 0:
                to_delete.append(track_id)

        for track_id in to_delete:
            del self._tracks[track_id]


def main():
    rclpy.init()
    node = ObjectContoursFromMask()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
