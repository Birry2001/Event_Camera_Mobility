import rclpy
from rclpy.node import Node
from rclpy.time import Time

import numpy as np

from dv_ros2_msgs.msg import EventArray
from sensor_msgs.msg import Image


class HotPixelMaskNode(Node):
    def __init__(self):
        super().__init__("hot_pixel_mask_node")

        self.declare_parameter("input_topic", "/events")
        self.declare_parameter("output_topic", "/events_filtered")
        self.declare_parameter("publish_unfiltered_during_calibration", True)
        self.declare_parameter("publish_mask_image", True)
        self.declare_parameter("mask_image_topic", "/hot_pixel_mask")
        self.declare_parameter("calibration_duration_s", 2.0)
        self.declare_parameter("rate_threshold", 400.0)
        self.declare_parameter("min_count", 50)

        self.input_topic = self.get_parameter("input_topic").value
        self.output_topic = self.get_parameter("output_topic").value
        self.publish_unfiltered = self.get_parameter(
            "publish_unfiltered_during_calibration"
        ).value
        self.publish_mask_image = self.get_parameter("publish_mask_image").value
        self.mask_image_topic = self.get_parameter("mask_image_topic").value
        self.calibration_duration_s = float(
            self.get_parameter("calibration_duration_s").value
        )
        self.rate_threshold = float(self.get_parameter("rate_threshold").value)
        self.min_count = int(self.get_parameter("min_count").value)

        self.events_sub = self.create_subscription(
            EventArray, self.input_topic, self.on_events, 10
        )
        self.events_pub = self.create_publisher(EventArray, self.output_topic, 10)

        self.mask_pub = None
        if self.publish_mask_image:
            self.mask_pub = self.create_publisher(Image, self.mask_image_topic, 10)

        self.counts = None
        self.mask = None
        self.height = None
        self.width = None
        self.start_time_ns = None
        self.calibrating = True

        self.get_logger().info(
            "Hot pixel mask: calibrating for %.2fs, rate_threshold=%.2f, min_count=%d"
            % (self.calibration_duration_s, self.rate_threshold, self.min_count)
        )

    def _msg_time_ns(self, msg: EventArray) -> int:
        stamp = msg.header.stamp
        if stamp.sec != 0 or stamp.nanosec != 0:
            return stamp.sec * 1_000_000_000 + stamp.nanosec
        now = self.get_clock().now()
        return int(now.nanoseconds)

    def _finalize_mask(self, end_time_ns: int) -> None:
        duration_ns = max(1, end_time_ns - self.start_time_ns)
        duration_s = duration_ns / 1_000_000_000.0
        rates = self.counts.astype(np.float32) / duration_s
        self.mask = (rates >= self.rate_threshold) & (self.counts >= self.min_count)
        self.calibrating = False

        hot_count = int(np.sum(self.mask))
        self.get_logger().info(
            "Hot pixel mask ready: %d hot pixels (duration %.3fs)"
            % (hot_count, duration_s)
        )

        if self.mask_pub is not None:
            img = Image()
            img.header.stamp = self.get_clock().now().to_msg()
            img.header.frame_id = ""
            img.height = int(self.height)
            img.width = int(self.width)
            img.encoding = "mono8"
            img.is_bigendian = False
            img.step = int(self.width)
            img.data = (self.mask.astype(np.uint8) * 255).tobytes()
            self.mask_pub.publish(img)

    def _filter_events(self, msg: EventArray) -> EventArray:
        if self.mask is None:
            return msg

        filtered = EventArray()
        filtered.header = msg.header
        filtered.height = msg.height
        filtered.width = msg.width

        events_out = []
        mask = self.mask
        for ev in msg.events:
            if not mask[ev.y, ev.x]:
                events_out.append(ev)
        filtered.events = events_out
        return filtered

    def on_events(self, msg: EventArray) -> None:
        if self.counts is None:
            self.height = msg.height
            self.width = msg.width
            self.counts = np.zeros((self.height, self.width), dtype=np.int32)

        msg_time_ns = self._msg_time_ns(msg)
        if self.start_time_ns is None:
            self.start_time_ns = msg_time_ns

        if self.calibrating:
            for ev in msg.events:
                self.counts[ev.y, ev.x] += 1

            elapsed_s = (msg_time_ns - self.start_time_ns) / 1_000_000_000.0
            if elapsed_s >= self.calibration_duration_s:
                self._finalize_mask(msg_time_ns)

            if self.publish_unfiltered:
                self.events_pub.publish(msg)
            return

        filtered = self._filter_events(msg)
        self.events_pub.publish(filtered)


def main():
    rclpy.init()
    node = HotPixelMaskNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
