import rclpy
from rclpy.node import Node

import numpy as np

from dv_ros2_msgs.msg import EventArray
from sensor_msgs.msg import Image


class EventRateFilterNode(Node):
    def __init__(self):
        super().__init__("event_rate_filter_node")

        self.declare_parameter("input_topic", "/events")
        self.declare_parameter("output_topic", "/events_rate_filtered")
        self.declare_parameter("rate_threshold", 200.0)
        self.declare_parameter("ema_alpha", 0.2)
        self.declare_parameter("min_dt_s", 0.001)
        self.declare_parameter("warmup_frames", 5)
        self.declare_parameter("publish_mask_image", True)
        self.declare_parameter("mask_image_topic", "/event_rate_mask")

        self.input_topic = self.get_parameter("input_topic").value
        self.output_topic = self.get_parameter("output_topic").value
        self.rate_threshold = float(self.get_parameter("rate_threshold").value)
        self.ema_alpha = float(self.get_parameter("ema_alpha").value)
        self.min_dt_s = float(self.get_parameter("min_dt_s").value)
        self.warmup_frames = int(self.get_parameter("warmup_frames").value)
        self.publish_mask_image = self.get_parameter("publish_mask_image").value
        self.mask_image_topic = self.get_parameter("mask_image_topic").value

        self.events_sub = self.create_subscription(
            EventArray, self.input_topic, self.on_events, 10
        )
        self.events_pub = self.create_publisher(EventArray, self.output_topic, 10)

        self.mask_pub = None
        if self.publish_mask_image:
            self.mask_pub = self.create_publisher(Image, self.mask_image_topic, 10)

        self.height = None
        self.width = None
        self.ema_rate = None
        self.last_time_ns = None
        self.frame_count = 0

        self.get_logger().info(
            "Event rate filter: threshold=%.2f ev/s, ema_alpha=%.2f"
            % (self.rate_threshold, self.ema_alpha)
        )

    def _msg_time_ns(self, msg: EventArray) -> int:
        stamp = msg.header.stamp
        if stamp.sec != 0 or stamp.nanosec != 0:
            return stamp.sec * 1_000_000_000 + stamp.nanosec
        now = self.get_clock().now()
        return int(now.nanoseconds)

    def _publish_mask(self, mask: np.ndarray) -> None:
        if self.mask_pub is None:
            return
        img = Image()
        img.header.stamp = self.get_clock().now().to_msg()
        img.header.frame_id = ""
        img.height = int(self.height)
        img.width = int(self.width)
        img.encoding = "mono8"
        img.is_bigendian = False
        img.step = int(self.width)
        img.data = (mask.astype(np.uint8) * 255).tobytes()
        self.mask_pub.publish(img)

    def on_events(self, msg: EventArray) -> None:
        if self.height is None:
            self.height = msg.height
            self.width = msg.width
            self.ema_rate = np.zeros((self.height, self.width), dtype=np.float32)

        msg_time_ns = self._msg_time_ns(msg)
        if self.last_time_ns is None:
            self.last_time_ns = msg_time_ns

        dt_s = (msg_time_ns - self.last_time_ns) / 1_000_000_000.0
        if dt_s <= 0.0:
            dt_s = self.min_dt_s
        elif dt_s < self.min_dt_s:
            dt_s = self.min_dt_s
        self.last_time_ns = msg_time_ns

        if not msg.events:
            self.events_pub.publish(msg)
            return

        xs = np.fromiter((ev.x for ev in msg.events), dtype=np.int32)
        ys = np.fromiter((ev.y for ev in msg.events), dtype=np.int32)
        flat = ys * self.width + xs
        counts = np.bincount(flat, minlength=self.height * self.width).reshape(
            (self.height, self.width)
        )
        rate = counts.astype(np.float32) / dt_s
        self.ema_rate = self.ema_alpha * rate + (1.0 - self.ema_alpha) * self.ema_rate

        mask = self.ema_rate >= self.rate_threshold
        self.frame_count += 1

        if self.frame_count <= self.warmup_frames:
            # During warmup, pass-through but still publish mask for inspection.
            self._publish_mask(mask)
            self.events_pub.publish(msg)
            return

        if np.any(mask):
            keep = ~(mask[ys, xs])
            events_out = [ev for ev, k in zip(msg.events, keep) if k]
        else:
            events_out = list(msg.events)

        filtered = EventArray()
        filtered.header = msg.header
        filtered.height = msg.height
        filtered.width = msg.width
        filtered.events = events_out

        self._publish_mask(mask)
        self.events_pub.publish(filtered)


def main():
    rclpy.init()
    node = EventRateFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
