#!/usr/bin/env python3
import argparse
import math
from typing import List, Tuple

import numpy as np
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu

try:
    from dv_ros2_msgs.msg import EventArray as DvEventArray
except Exception:  # pragma: no cover - optional dependency
    DvEventArray = None

try:
    from dvs_msgs.msg import EventArray as DvsEventArray
except Exception:  # pragma: no cover - optional dependency
    DvsEventArray = None


def stamp_to_sec(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def event_timestamp_to_sec(ev) -> float:
    # dv_ros2_msgs/Event has field ts
    if hasattr(ev, "ts"):
        return stamp_to_sec(ev.ts)
    return 0.0


class LagChecker(Node):
    def __init__(self, args):
        super().__init__("imu_event_lag_checker")
        self.imu_topic = args.imu_topic
        self.events_topic = args.events_topic
        self.duration_s = args.duration_s
        self.bin_dt = args.bin_dt
        self.max_lag_s = args.max_lag_s

        self.imu_samples: List[Tuple[float, float]] = []
        self.event_samples: List[Tuple[float, float]] = []

        self.prev_event_stamp = None
        self.start_time = None

        self.create_subscription(Imu, self.imu_topic, self.on_imu, 50)

        event_msg_type = None
        if DvEventArray is not None:
            event_msg_type = DvEventArray
        elif DvsEventArray is not None:
            event_msg_type = DvsEventArray
        else:
            raise RuntimeError("No EventArray message found (dv_ros2_msgs or dvs_msgs)")

        self.create_subscription(event_msg_type, self.events_topic, self.on_events, 50)

        self.get_logger().info(
            "Collecting data: imu_topic=%s events_topic=%s duration=%.1fs"
            % (self.imu_topic, self.events_topic, self.duration_s)
        )

    def on_imu(self, msg: Imu) -> None:
        t = stamp_to_sec(msg.header.stamp)
        if self.start_time is None:
            self.start_time = t
        omega = msg.angular_velocity
        mag = math.sqrt(omega.x * omega.x + omega.y * omega.y + omega.z * omega.z)
        self.imu_samples.append((t, mag))
        self.maybe_finish(t)

    def on_events(self, msg) -> None:
        t = stamp_to_sec(msg.header.stamp)
        if self.start_time is None:
            self.start_time = t
        if self.prev_event_stamp is None:
            self.prev_event_stamp = t
            return
        dt = t - self.prev_event_stamp
        if dt <= 1e-6:
            self.prev_event_stamp = t
            return
        rate = float(len(msg.events)) / dt
        self.event_samples.append((t, rate))
        self.prev_event_stamp = t
        self.maybe_finish(t)

    def maybe_finish(self, current_time: float) -> None:
        if self.start_time is None:
            return
        if current_time - self.start_time < self.duration_s:
            return
        self.compute_lag()
        rclpy.shutdown()

    def compute_lag(self) -> None:
        if len(self.imu_samples) < 10 or len(self.event_samples) < 10:
            self.get_logger().error("Not enough data to estimate lag")
            return

        imu_t = np.array([s[0] for s in self.imu_samples])
        imu_v = np.array([s[1] for s in self.imu_samples])
        evt_t = np.array([s[0] for s in self.event_samples])
        evt_v = np.array([s[1] for s in self.event_samples])

        t0 = max(imu_t.min(), evt_t.min())
        t1 = min(imu_t.max(), evt_t.max())
        if t1 <= t0:
            self.get_logger().error("No overlapping time range")
            return

        grid = np.arange(t0, t1, self.bin_dt)
        imu_grid = np.interp(grid, imu_t, imu_v)
        evt_grid = np.interp(grid, evt_t, evt_v)

        imu_grid -= np.mean(imu_grid)
        evt_grid -= np.mean(evt_grid)

        max_lag_bins = int(self.max_lag_s / self.bin_dt)
        best_lag = 0.0
        best_corr = -1e9

        for lag_bins in range(-max_lag_bins, max_lag_bins + 1):
            if lag_bins < 0:
                a = imu_grid[-lag_bins:]
                b = evt_grid[: len(a)]
            elif lag_bins > 0:
                a = imu_grid[: -lag_bins]
                b = evt_grid[lag_bins:]
            else:
                a = imu_grid
                b = evt_grid
            if len(a) < 10:
                continue
            corr = float(np.dot(a, b)) / (np.linalg.norm(a) * np.linalg.norm(b) + 1e-9)
            if corr > best_corr:
                best_corr = corr
                best_lag = lag_bins * self.bin_dt

        # Positive lag means events lag behind IMU.
        self.get_logger().info(
            "Estimated lag: %.4fs (events lag IMU if positive), corr=%.3f"
            % (best_lag, best_corr)
        )


def main():
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
