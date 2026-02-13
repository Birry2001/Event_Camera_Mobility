import numpy as np
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import Float32

from cv_bridge import CvBridge

# message_filters permet de synchroniser plusieurs topics (ici time_image et count_image)
# en approx. temps (tolérance "slop"), ce qui est pratique quand ils ne tombent pas
# EXACTEMENT au même timestamp.
from message_filters import Subscriber, ApproximateTimeSynchronizer

# NOTE : ndimage est importé mais pas utilisé dans ce code (morpho open/close non implémentées ici).
from scipy import ndimage


class EventSegmentationNode(Node):
    def __init__(self):
        super().__init__("event_segmentation")

        # -----------------------------
        # Paramètres ROS (configurables via YAML / ligne de commande)
        # -----------------------------
        self.declare_parameter("time_image_topic", "/time_image")   # Image float32 : info de "temps" par pixel
        self.declare_parameter("count_image_topic", "/count_image") # Image uint8 : nb d'évènements par pixel
        self.declare_parameter("mask_topic", "/event_mask")         # Image mono8 : masque binaire en sortie (0/255)

        # Log de stats debug
        self.declare_parameter("log_stats", False)

        # Paramètres morphologiques (déclarés mais PAS utilisés dans ce script tel quel)
        self.declare_parameter("morph_open", True)
        self.declare_parameter("morph_close", True)
        self.declare_parameter("morph_kernel", 3)

        self.declare_parameter("min_count", 1)

        # -----------------------------
        # Lecture des paramètres
        # -----------------------------
        time_topic = self.get_parameter("time_image_topic").get_parameter_value().string_value
        count_topic = self.get_parameter("count_image_topic").get_parameter_value().string_value
        mask_topic = self.get_parameter("mask_topic").get_parameter_value().string_value

        self._log_stats = self.get_parameter("log_stats").get_parameter_value().bool_value

        self._morph_open = self.get_parameter("morph_open").get_parameter_value().bool_value
        self._morph_close = self.get_parameter("morph_close").get_parameter_value().bool_value
        self._morph_kernel = int(self.get_parameter("morph_kernel").get_parameter_value().integer_value)
        
        self._min_count = self.get_parameter("min_count").get_parameter_value().integer_value

        # -----------------------------
        # Outils OpenCV <-> ROS Image
        # -----------------------------
        self._bridge = CvBridge()

        # Publisher du masque binaire en sortie
        self._mask_pub = self.create_publisher(Image, mask_topic, 10)

        # -----------------------------
        # Seuil dynamique externe ("lambda") : permet de forcer le seuil à une valeur publiée ailleurs
        # -----------------------------
        self.declare_parameter("lambda_topic", "/event_lambda_threshold")
        lambda_topic = self.get_parameter("lambda_topic").get_parameter_value().string_value

        # Dernière valeur reçue sur lambda_topic (None tant qu’on n’a rien reçu)
        self._lambda_latest = None
        self.create_subscription(Float32, lambda_topic, self._on_lambda, 10)

        # -----------------------------
        # Subscribers synchronisés : time_image + count_image
        # -----------------------------
        time_sub = Subscriber(self, Image, time_topic)
        count_sub = Subscriber(self, Image, count_topic)

        # Synchronisation approximative :
        # - queue_size: taille de buffer
        # - slop: tolérance temporelle max (en secondes)
        sync = ApproximateTimeSynchronizer([time_sub, count_sub], queue_size=10, slop=0.05)
        sync.registerCallback(self._on_images)

    def _on_lambda(self, msg: Float32) -> None:
        """Callback quand on reçoit un nouveau seuil dynamique 'lambda'."""
        self._lambda_latest = float(msg.data)

    def _on_images(self, time_msg: Image, count_msg: Image) -> None:
        """
        Callback appelée quand on a une paire (time_image, count_image) synchronisée.

        Entrées :
        - time_msg  : Image ROS contenant une "time image" (float32 par pixel)
        - count_msg : Image ROS contenant une "count image" (uint8 par pixel)

        Sortie :
        - Publie sur self._mask_pub une image mono8 (0 ou 255) représentant le masque binaire
            des pixels considérés comme "foreground" (dynamiques / intéressants) selon :
            1) un filtre de validité par min_count
            2) un seuil lambda (publié sur un autre topic)            
        """
        # Convertit les messages ROS Image en matrices numpy.
        # - "32FC1"  : 32-bit float, 1 canal (time_img)
        # - "mono8"  : 8-bit unsigned, 1 canal (count_img)
        time_img = self._bridge.imgmsg_to_cv2(time_msg, desired_encoding="32FC1")
        count_img = self._bridge.imgmsg_to_cv2(count_msg, desired_encoding="mono8")

        # Sécurité : si conversion ratée (message vide / encodage inattendu)
        if time_img is None or count_img is None:
            return

        # Masque des pixels "actifs" selon count_image :
        count_mask = count_img >= self._min_count
 

        # 1) Aucun pixel actif -> publier un masque entièrement vide (tout à 0).
        #    Ça évite de publier un bruit arbitraire quand il n'y a rien d'intéressant.
        if not np.any(count_mask):
            empty = np.zeros(count_img.shape, dtype=np.uint8)
            out_msg = self._bridge.cv2_to_imgmsg(empty, encoding="mono8")
            out_msg.header = time_msg.header  # on conserve le timestamp/frame d'origine
            self._mask_pub.publish(out_msg)
            return

        # 2) Tant qu’on n’a pas reçu de valeur de lambda (seuil dynamique),
        #    on publie vide (choix "safe").
        #    Alternative possible (fallback) : utiliser un seuil statistique comme avant.
        if self._lambda_latest is None:
            empty = np.zeros(count_img.shape, dtype=np.uint8)
            out_msg = self._bridge.cv2_to_imgmsg(empty, encoding="mono8")
            out_msg.header = time_msg.header
            self._mask_pub.publish(out_msg)
            return

        # 3) Seuillage "article-like" :
        #    On prend le seuil directement depuis lambda (déjà calculé ailleurs).
        threshold = float(self._lambda_latest)

        # Construction du masque foreground :
        # - time_img > threshold : pixels dont la "valeur de temps" dépasse le seuil
        # - & count_mask         : et qui sont suffisamment actifs (min_count)
        #
        # Remarque : si time_img a été "centré" (valeurs pouvant être négatives),
        # alors un filtre du type (time_img >= self._min_time) avec min_time=0.0
        # peut supprimer beaucoup de pixels. D'où le commentaire sur min_time.
        fg_mask = (time_img > threshold) & count_mask  # + (time_img >= self._min_time) si besoin

        if self._morph_open or self._morph_close:
            k = max(1, self._morph_kernel)
            structure = np.ones((k, k), dtype=bool)
            if self._morph_open:
                fg_mask = ndimage.binary_opening(fg_mask, structure=structure)
            if self._morph_close:
                fg_mask = ndimage.binary_closing(fg_mask, structure=structure)

        # 4) Post-traitement morphologique (optionnel) :
        #    Ici tu peux mettre des opérations pour :
        #      - enlever le bruit (opening)
        #      - combler des trous (closing)
        #    Ex : ndimage.binary_opening / binary_closing avec un noyau.
        #    (Pas implémenté dans ce snippet : à insérer ici si tu le gardes.)

        # Conversion du masque booléen vers une image mono8 :
        # - False -> 0
        # - True  -> 255
        mask_uint8 = (fg_mask.astype(np.uint8) * 255)

        # Création et publication du message Image ROS de sortie.
        out_msg = self._bridge.cv2_to_imgmsg(mask_uint8, encoding="mono8")
        out_msg.header = time_msg.header  # timestamp/frame aligné sur time_image
        self._mask_pub.publish(out_msg)

        # 5) Log optionnel : utile pour vérifier que le pipeline réagit et que lambda est cohérent.
        if self._log_stats:
            self.get_logger().info(
                f"mask: count={int(np.count_nonzero(count_mask))} "
                f"fg={int(np.count_nonzero(fg_mask))} "
                f"lambda={threshold:.6f}"
            )





def main() -> None:
    # Initialisation ROS 2
    rclpy.init()

    # Création du node
    node = EventSegmentationNode()

    # Boucle d’exécution (callbacks)
    rclpy.spin(node)

    # Nettoyage
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
