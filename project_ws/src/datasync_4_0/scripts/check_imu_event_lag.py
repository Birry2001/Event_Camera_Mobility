#!/usr/bin/env python3
"""
ROS 2 node qui estime le décalage temporel (lag) entre :
- un topic IMU (sensor_msgs/Imu)
- un topic d'événements (EventArray) venant soit de dv_ros2_msgs, soit de dvs_msgs

Idée :
1) On extrait deux signaux scalaires au cours du temps :
   - IMU : norme de la vitesse angulaire |ω| (rad/s)
   - Events : taux d'événements (events/s) estimé entre deux messages EventArray
2) On ré-échantillonne les deux signaux sur une même grille temporelle régulière (bin_dt)
3) On cherche le lag qui maximise la corrélation normalisée entre les deux séries
   => le lag estimé donne (approximativement) le retard des events par rapport à l'IMU
"""

import argparse
import math
from typing import List, Tuple

import numpy as np
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu

# Ces imports sont "optionnels" car selon ton setup tu peux avoir dv_ros2_msgs OU dvs_msgs.
# On tente le premier, sinon le second.
try:
    from dv_ros2_msgs.msg import EventArray as DvEventArray
except Exception:  # pragma: no cover - optional dependency
    DvEventArray = None

try:
    from dvs_msgs.msg import EventArray as DvsEventArray
except Exception:  # pragma: no cover - optional dependency
    DvsEventArray = None


def stamp_to_sec(stamp) -> float:
    """Convertit un builtin_interfaces/Time (sec, nanosec) en secondes flottantes."""
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def event_timestamp_to_sec(ev) -> float:
    """
    (Non utilisé dans ce code actuellement)
    Exemple de helper pour extraire un timestamp par événement, si le type d'event possède un champ 'ts'.
    Dans dv_ros2_msgs, un Event peut avoir un champ ts.
    """
    if hasattr(ev, "ts"):
        return stamp_to_sec(ev.ts)
    return 0.0


class LagChecker(Node):
    """
    Node ROS 2 qui collecte des données IMU + events pendant 'duration_s' secondes,
    puis calcule le lag via corrélation sur une grille temporelle commune.
    """

    def __init__(self, args):
        super().__init__("imu_event_lag_checker")

        # Paramètres (via argparse)
        self.imu_topic = args.imu_topic
        self.events_topic = args.events_topic
        self.duration_s = args.duration_s
        self.bin_dt = args.bin_dt
        self.max_lag_s = args.max_lag_s

        # Buffers de données sous la forme (timestamp_en_secondes, valeur)
        # - IMU: valeur = |ω|
        # - Events: valeur = taux events/s
        self.imu_samples: List[Tuple[float, float]] = []
        self.event_samples: List[Tuple[float, float]] = []

        # Pour calculer un taux d'events, on a besoin du timestamp précédent du message EventArray.
        self.prev_event_stamp = None

        # Référence temporelle du début de collecte (premier message reçu : IMU ou Events)
        self.start_time = None

        # Subscription IMU
        # QoS depth=50 : ici un buffer simple suffisant pour un outil de mesure offline.
        self.create_subscription(Imu, self.imu_topic, self.on_imu, 50)

        # Déterminer dynamiquement le type de message Events (selon les dépendances dispo)
        event_msg_type = None
        if DvEventArray is not None:
            event_msg_type = DvEventArray
        elif DvsEventArray is not None:
            event_msg_type = DvsEventArray
        else:
            raise RuntimeError("No EventArray message found (dv_ros2_msgs or dvs_msgs)")

        # Subscription Events
        self.create_subscription(event_msg_type, self.events_topic, self.on_events, 50)

        self.get_logger().info(
            "Collecting data: imu_topic=%s events_topic=%s duration=%.1fs"
            % (self.imu_topic, self.events_topic, self.duration_s)
        )

    def on_imu(self, msg: Imu) -> None:
        """
        Callback IMU :
        - Convertit le header.stamp en secondes
        - Calcule la norme de la vitesse angulaire |ω| = sqrt(wx^2 + wy^2 + wz^2)
        - Ajoute l'échantillon dans imu_samples
        - Vérifie si on a collecté assez longtemps pour terminer
        """
        t = stamp_to_sec(msg.header.stamp)

        # Définir le début de collecte au premier message reçu
        if self.start_time is None:
            self.start_time = t

        omega = msg.angular_velocity
        mag = math.sqrt(omega.x * omega.x + omega.y * omega.y + omega.z * omega.z)

        self.imu_samples.append((t, mag))
        self.maybe_finish(t)

    def on_events(self, msg) -> None:
        """
        Callback Events :
        - Utilise le header.stamp du message EventArray (timestamp de "batch")
        - Estime un taux events/s entre deux messages successifs :
            rate = len(msg.events) / dt
          où dt = t - t_prev
        - Ajoute (t, rate) dans event_samples
        - Vérifie si on a collecté assez longtemps
        """
        t = stamp_to_sec(msg.header.stamp)

        if self.start_time is None:
            self.start_time = t

        # Premier message : pas de dt possible, on initialise juste prev_event_stamp
        if self.prev_event_stamp is None:
            self.prev_event_stamp = t
            return

        dt = t - self.prev_event_stamp

        # Si dt ~ 0 (timestamps identiques ou très proches), on évite division par 0 / valeurs instables
        if dt <= 1e-6:
            self.prev_event_stamp = t
            return

        # Taux moyen sur l'intervalle [t_prev, t]
        rate = float(len(msg.events)) / dt
        self.event_samples.append((t, rate))

        # Mettre à jour le timestamp précédent
        self.prev_event_stamp = t

        self.maybe_finish(t)

    def maybe_finish(self, current_time: float) -> None:
        """
        Arrête la collecte et lance le calcul dès qu'on dépasse duration_s.
        """
        if self.start_time is None:
            return
        if current_time - self.start_time < self.duration_s:
            return

        # On a assez de données : calculer et quitter proprement
        self.compute_lag()
        rclpy.shutdown()

    def compute_lag(self) -> None:
        """
        Calcule le lag via corrélation :
        1) Transforme les listes en numpy arrays
        2) Trouve la portion de temps commune (overlap) entre IMU et Events
        3) Crée une grille temporelle régulière (bin_dt)
        4) Interpole les deux signaux sur cette grille
        5) Retire la moyenne (centrage)
        6) Teste différents lags (de -max_lag_s à +max_lag_s) et choisit celui
           qui maximise la corrélation normalisée (cosine similarity)
        """
        if len(self.imu_samples) < 10 or len(self.event_samples) < 10:
            self.get_logger().error("Not enough data to estimate lag")
            return

        # Séparer temps et valeurs
        imu_t = np.array([s[0] for s in self.imu_samples])
        imu_v = np.array([s[1] for s in self.imu_samples])
        evt_t = np.array([s[0] for s in self.event_samples])
        evt_v = np.array([s[1] for s in self.event_samples])

        # Fenêtre temporelle où les deux signaux existent (intersection des intervalles)
        t0 = max(imu_t.min(), evt_t.min())
        t1 = min(imu_t.max(), evt_t.max())
        if t1 <= t0:
            self.get_logger().error("No overlapping time range")
            return

        # Grille régulière : on va comparer les signaux point-à-point sur cette grille
        grid = np.arange(t0, t1, self.bin_dt)

        # Interpolation linéaire pour obtenir des séries échantillonnées au même rythme
        imu_grid = np.interp(grid, imu_t, imu_v)
        evt_grid = np.interp(grid, evt_t, evt_v)

        # Centrage (retirer la moyenne) : important pour corrélation, on veut comparer les variations
        imu_grid -= np.mean(imu_grid)
        evt_grid -= np.mean(evt_grid)

        # Convertit le lag max (secondes) en nombre de "bins" (pas bin_dt)
        max_lag_bins = int(self.max_lag_s / self.bin_dt)

        best_lag = 0.0
        best_corr = -1e9

        # On balaye tous les décalages possibles
        # lag_bins < 0 : on "avance" IMU par rapport aux events (ou inverse selon convention)
        # lag_bins > 0 : on "retarde" IMU par rapport aux events
        for lag_bins in range(-max_lag_bins, max_lag_bins + 1):
            # On aligne les deux séries selon le lag en tronquant les bords
            if lag_bins < 0:
                # Ex: lag_bins=-3 -> imu_grid[3:] aligné avec evt_grid[:-3] (via slicing ci-dessous)
                a = imu_grid[-lag_bins:]
                b = evt_grid[: len(a)]
            elif lag_bins > 0:
                a = imu_grid[: -lag_bins]
                b = evt_grid[lag_bins:]
            else:
                a = imu_grid
                b = evt_grid

            # Éviter les corrélations sur trop peu d'échantillons
            if len(a) < 10:
                continue

            # Corrélation normalisée ~ cosine similarity :
            # dot(a,b) / (||a|| ||b||)
            # +1e-9 pour éviter division par zéro si norme quasi nulle
            corr = float(np.dot(a, b)) / (np.linalg.norm(a) * np.linalg.norm(b) + 1e-9)

            if corr > best_corr:
                best_corr = corr
                best_lag = lag_bins * self.bin_dt

        # Convention :
        # Ici, un lag positif signifie que la série "events" est en retard par rapport à l'IMU
        # (autrement dit, il faut décaler les events vers le futur pour mieux matcher l'IMU).
        self.get_logger().info(
            "Estimated lag: %.4fs (events lag IMU if positive), corr=%.3f"
            % (best_lag, best_corr)
        )


def main():
    """
    Point d'entrée CLI :
    - parse les arguments
    - init ROS 2
    - spin le node jusqu'à ce qu'il s'arrête tout seul après duration_s
    """
    parser = argparse.ArgumentParser(description="Estimate IMU-event time lag.")
    parser.add_argument("--imu-topic", default="/dvs/imu")
    parser.add_argument("--events-topic", default="/events")
    parser.add_argument("--duration-s", type=float, default=10.0)
    parser.add_argument("--bin-dt", type=float, default=0.01)
    parser.add_argument("--max-lag-s", type=float, default=0.1)
    args = parser.parse_args()

    rclpy.init()
    node = LagChecker(args)
    rclpy.spin(node)
    node.destroy_node()


if __name__ == "__main__":
    main()
