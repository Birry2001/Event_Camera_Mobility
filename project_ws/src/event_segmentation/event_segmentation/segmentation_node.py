#!/usr/bin/env python3
"""
Segmentation stage of the event pipeline.

Inputs:
- /time_image (float32 image from motion compensation)
- /count_image (event density image)
- /event_lambda_threshold (dynamic threshold published by motion compensation)

Outputs:
- /event_mask (mono8, foreground candidates)
- /event_segmentation_vis (BGR visualization)

Rule (paper-inspired):
foreground <=> |time_image| > lambda AND count_image >= min_count
"""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import Float32

from cv_bridge import CvBridge
from message_filters import Subscriber, ApproximateTimeSynchronizer


class EventSegmentationNode(Node):
    def __init__(self):
        super().__init__("event_segmentation")

        # Inputs
        self.declare_parameter("time_image_topic", "/time_image")
        self.declare_parameter("count_image_topic", "/count_image")
        self.declare_parameter("lambda_topic", "/event_lambda_threshold")

        # Outputs
        self.declare_parameter("mask_topic", "/event_mask")  # mono8 0/255
        self.declare_parameter("vis_topic", "/event_segmentation_vis")  # bgr8 like paper Fig.1(c)
        self.declare_parameter("time_abs_max_topic", "/time_abs_max_topic")

        # Gating
        self.declare_parameter("min_count", 1)
        self.declare_parameter("log_stats", False)

        # Paper-style colors (OpenCV BGR)
        self.declare_parameter("vis_background_white", True)
        self.declare_parameter("vis_bg_bgr", [255, 0, 0])   # blue = static background
        self.declare_parameter("vis_fg_bgr", [0, 0, 255])   # red  = dynamic candidates
        self.declare_parameter("vis_point_radius_px", 1)    # visualization only

        # Read params
        time_topic = str(self.get_parameter("time_image_topic").value)
        count_topic = str(self.get_parameter("count_image_topic").value)
        lambda_topic = str(self.get_parameter("lambda_topic").value)

        mask_topic = str(self.get_parameter("mask_topic").value)
        vis_topic = str(self.get_parameter("vis_topic").value)
        time_abs_max_topic = str(self.get_parameter("time_abs_max_topic").value)

        self._min_count = max(1, int(self.get_parameter("min_count").value))
        self._log_stats = bool(self.get_parameter("log_stats").value)

        self._vis_background_white = bool(self.get_parameter("vis_background_white").value)
        self._vis_bg_bgr = np.array(self.get_parameter("vis_bg_bgr").value, dtype=np.uint8).reshape(1, 1, 3)
        self._vis_fg_bgr = np.array(self.get_parameter("vis_fg_bgr").value, dtype=np.uint8).reshape(1, 1, 3)
        self._vis_point_radius = max(0, int(self.get_parameter("vis_point_radius_px").value))
        self._vis_kernel = None
        if self._vis_point_radius > 0:
            k = 2 * self._vis_point_radius + 1
            self._vis_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))

        self._bridge = CvBridge()
        self._mask_pub = self.create_publisher(Image, mask_topic, 10)
        self._vis_pub = self.create_publisher(Image, vis_topic, 10)
        self._time_abs_max_pub = self.create_publisher(Float32, time_abs_max_topic, 10)

        # Lambda is asynchronous; we always use the latest value received.
        self._lambda_latest = None
        self.create_subscription(Float32, lambda_topic, self._on_lambda, 10)

        # Synchronize time/count images from the same batch window.
        time_sub = Subscriber(self, Image, time_topic)
        count_sub = Subscriber(self, Image, count_topic)
        sync = ApproximateTimeSynchronizer([time_sub, count_sub], queue_size=10, slop=0.05)
        sync.registerCallback(self._on_images)

    def _on_lambda(self, msg: Float32) -> None:
        self._lambda_latest = float(msg.data)

    def _publish_zero(self, shape_hw, header) -> None:
        # mask = 0
        mask = np.zeros(shape_hw, dtype=np.uint8)
        mask_msg = self._bridge.cv2_to_imgmsg(mask, encoding="mono8")
        mask_msg.header = header
        self._mask_pub.publish(mask_msg)

        # vis = white (or black) background only
        if self._vis_background_white:
            vis = np.full((shape_hw[0], shape_hw[1], 3), 255, dtype=np.uint8)
        else:
            vis = np.zeros((shape_hw[0], shape_hw[1], 3), dtype=np.uint8)
        vis_msg = self._bridge.cv2_to_imgmsg(vis, encoding="bgr8")
        vis_msg.header = header
        self._vis_pub.publish(vis_msg)

    def _on_images(self, time_msg: Image, count_msg: Image) -> None:
        time_img = self._bridge.imgmsg_to_cv2(time_msg, desired_encoding="32FC1")
        count_img = self._bridge.imgmsg_to_cv2(count_msg, desired_encoding="mono8")
        if time_img is None or count_img is None:
            return

        h, w = count_img.shape
        count_mask = count_img >= self._min_count

        # If lambda is not ready yet (startup) or no active pixels, publish an empty result.
        if not np.any(count_mask) or self._lambda_latest is None:
            self._publish_zero((h, w), time_msg.header)
            return

        # time_img is expected to be the centered/normalized temporal residual map.
        residual = np.abs(time_img)
        active_values = residual[count_mask]
        if active_values.size == 0:
            self._publish_zero((h, w), time_msg.header)
            return

        lambda_thr = abs(float(self._lambda_latest))
        # Foreground candidates = high residual time values among sufficiently active pixels.
        fg_mask = (residual > lambda_thr) & count_mask
        bg_mask = count_mask & (~fg_mask)

        # publish abs max for monitoring
        max_msg = Float32()
        max_msg.data = float(np.max(active_values))
        self._time_abs_max_pub.publish(max_msg)

        # publish mono mask (dynamic candidates)
        mask_uint8 = (fg_mask.astype(np.uint8) * 255)
        out_mask = self._bridge.cv2_to_imgmsg(mask_uint8, encoding="mono8")
        out_mask.header = time_msg.header
        self._mask_pub.publish(out_mask)

        # publish paper-style vis: white background, bg=blue, fg=red
        if self._vis_background_white:
            vis = np.full((h, w, 3), 255, dtype=np.uint8)
        else:
            vis = np.zeros((h, w, 3), dtype=np.uint8)

        bg_vis_mask = bg_mask
        fg_vis_mask = fg_mask
        if self._vis_kernel is not None:
            # Visualization only: enlarge sparse event pixels to improve readability.
            bg_vis_mask = cv2.dilate(bg_mask.astype(np.uint8), self._vis_kernel, iterations=1) > 0
            fg_vis_mask = cv2.dilate(fg_mask.astype(np.uint8), self._vis_kernel, iterations=1) > 0
            bg_vis_mask = bg_vis_mask & (~fg_vis_mask)

        vis[bg_vis_mask] = self._vis_bg_bgr
        vis[fg_vis_mask] = self._vis_fg_bgr  # fg overrides bg

        out_vis = self._bridge.cv2_to_imgmsg(vis, encoding="bgr8")
        out_vis.header = time_msg.header
        self._vis_pub.publish(out_vis)

        if self._log_stats:
            self.get_logger().info(
                "seg: active_px=%d bg_px=%d fg_px=%d lambda=%.5f thr=%.5f",
                int(np.count_nonzero(count_mask)),
                int(np.count_nonzero(bg_mask)),
                int(np.count_nonzero(fg_mask)),
                float(self._lambda_latest),
                float(lambda_thr),
            )


def main() -> None:
    rclpy.init()
    node = EventSegmentationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
