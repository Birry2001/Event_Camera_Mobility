import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class EventDenoiseNode(Node):
    def __init__(self) -> None:
        super().__init__("event_denoise")

        self.declare_parameter("input_topic", "/count_image")
        self.declare_parameter("output_topic", "/count_image_denoised")
        self.declare_parameter("min_value", 5)
        self.declare_parameter("median_blur", True)
        self.declare_parameter("median_kernel", 3)
        self.declare_parameter("morph_open", True)
        self.declare_parameter("morph_close", True)
        self.declare_parameter("morph_kernel", 3)

        input_topic = self.get_parameter("input_topic").get_parameter_value().string_value
        output_topic = self.get_parameter("output_topic").get_parameter_value().string_value

        self._min_value = int(self.get_parameter("min_value").get_parameter_value().integer_value)
        self._median_blur = self.get_parameter("median_blur").get_parameter_value().bool_value
        self._median_kernel = int(self.get_parameter("median_kernel").get_parameter_value().integer_value)
        self._morph_open = self.get_parameter("morph_open").get_parameter_value().bool_value
        self._morph_close = self.get_parameter("morph_close").get_parameter_value().bool_value
        self._morph_kernel = int(self.get_parameter("morph_kernel").get_parameter_value().integer_value)

        self._bridge = CvBridge()
        self._pub = self.create_publisher(Image, output_topic, 10)
        self._sub = self.create_subscription(Image, input_topic, self._on_image, 10)

        self.get_logger().info(
            f"Denoise node listening on {input_topic}, publishing {output_topic}"
        )

    def _on_image(self, msg: Image) -> None:
        img = self._bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")
        if img is None:
            return

        work = img
        if self._median_blur:
            k = max(1, self._median_kernel)
            if k % 2 == 0:
                k += 1
            work = cv2.medianBlur(work, k)

        mask = work >= self._min_value

        if self._morph_open or self._morph_close:
            k = max(1, self._morph_kernel)
            kernel = np.ones((k, k), dtype=np.uint8)
            mask_u8 = (mask.astype(np.uint8) * 255)
            if self._morph_open:
                mask_u8 = cv2.morphologyEx(mask_u8, cv2.MORPH_OPEN, kernel)
            if self._morph_close:
                mask_u8 = cv2.morphologyEx(mask_u8, cv2.MORPH_CLOSE, kernel)
            mask = mask_u8 > 0

        out = np.where(mask, img, 0).astype(np.uint8)
        out_msg = self._bridge.cv2_to_imgmsg(out, encoding="mono8")
        out_msg.header = msg.header
        self._pub.publish(out_msg)


def main() -> None:
    rclpy.init()
    node = EventDenoiseNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
