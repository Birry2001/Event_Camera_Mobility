#!/usr/bin/env python3
"""Event clustering 2.0 (clustering only), aligned with paper section B.

Reference:
  Event-based Real-time Moving Object Detection Based On IMU Ego-motion Compensation
  Section B "Objects Clustering", Eq. (12):
      C_ij(p, v) = wp * ||p_i - p_j|| + wv * ||v_i - v_j||
  where p=(x, y, t) and v is a 2D optical-flow-like velocity.

This node does NOT do segmentation.
It expects a dynamic mask produced upstream, and performs:
  1) event selection by input mask (optional)
  2) median denoising on the input mask (optional)
  3) joint DBSCAN clustering with position+velocity metric
  4) publication of one color image containing the clusters
"""

from collections import deque

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from dv_ros2_msgs.msg import EventArray
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


def to_sec(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


class EventClustering20Node(Node):
    def __init__(self) -> None:
        super().__init__("event_clustering_2_0")

        # Inputs/output.
        self.declare_parameter("input_mask_topic", "/event_mask")
        self.declare_parameter("events_comp_topic", "/events_compensated")
        self.declare_parameter("clusters_color_topic", "/event_clusters_color")

        # Synchronization and logs.
        self.declare_parameter("sync_queue_size", 10)
        self.declare_parameter("sync_slop", 0.05)
        self.declare_parameter("log_stats", True)

        # Event selection from upstream segmentation.
        self.declare_parameter("use_input_mask", True)
        self.declare_parameter("input_mask_min_value", 1)

        # Paper mentions median filtering for salt-and-pepper noise suppression.
        self.declare_parameter("median_filter_enable", True)
        self.declare_parameter("median_filter_kernel", 3)

        # Event budget for realtime.
        self.declare_parameter("max_events", 3000)

        # Local velocity estimate (2D optical-flow-like from space-time continuity).
        self.declare_parameter("flow_search_radius_px", 4.0)
        self.declare_parameter("flow_dt_min_s", 1e-5)
        self.declare_parameter("flow_dt_max_s", 0.03)
        self.declare_parameter("flow_velocity_clip", 5000.0)
        self.declare_parameter("velocity_scale", 1.0)

        # Spatio-temporal normalization for p=(x,y,t).
        self.declare_parameter("position_time_scale", 1000.0)

        # DBSCAN joint metric: wp*||p_i-p_j|| + wv*||v_i-v_j||.
        self.declare_parameter("dbscan_eps", 20.0)
        self.declare_parameter("dbscan_min_samples", 6)
        self.declare_parameter("dbscan_wp", 1.0)
        self.declare_parameter("dbscan_wv", 0.002)
        self.declare_parameter("min_cluster_events", 10)

        # Read parameters.
        self._input_mask_topic = str(self.get_parameter("input_mask_topic").value)
        self._events_comp_topic = str(self.get_parameter("events_comp_topic").value)
        self._clusters_color_topic = str(self.get_parameter("clusters_color_topic").value)

        self._sync_queue_size = int(self.get_parameter("sync_queue_size").value)
        self._sync_slop = float(self.get_parameter("sync_slop").value)
        self._log_stats = bool(self.get_parameter("log_stats").value)

        self._use_input_mask = bool(self.get_parameter("use_input_mask").value)
        self._input_mask_min_value = int(self.get_parameter("input_mask_min_value").value)

        self._median_filter_enable = bool(self.get_parameter("median_filter_enable").value)
        self._median_filter_kernel = int(self.get_parameter("median_filter_kernel").value)

        self._max_events = int(self.get_parameter("max_events").value)

        self._flow_search_radius_px = float(self.get_parameter("flow_search_radius_px").value)
        self._flow_dt_min_s = float(self.get_parameter("flow_dt_min_s").value)
        self._flow_dt_max_s = float(self.get_parameter("flow_dt_max_s").value)
        self._flow_velocity_clip = float(self.get_parameter("flow_velocity_clip").value)
        self._velocity_scale = float(self.get_parameter("velocity_scale").value)

        self._position_time_scale = float(self.get_parameter("position_time_scale").value)

        self._dbscan_eps = float(self.get_parameter("dbscan_eps").value)
        self._dbscan_min_samples = int(self.get_parameter("dbscan_min_samples").value)
        self._dbscan_wp = float(self.get_parameter("dbscan_wp").value)
        self._dbscan_wv = float(self.get_parameter("dbscan_wv").value)
        self._min_cluster_events = int(self.get_parameter("min_cluster_events").value)

        # Safety.
        self._sync_queue_size = max(1, self._sync_queue_size)
        self._sync_slop = max(0.0, self._sync_slop)
        self._input_mask_min_value = max(0, min(255, self._input_mask_min_value))

        self._median_filter_kernel = max(1, self._median_filter_kernel)
        if self._median_filter_kernel % 2 == 0:
            self._median_filter_kernel += 1

        self._max_events = max(100, self._max_events)
        self._flow_search_radius_px = max(0.1, self._flow_search_radius_px)
        self._flow_dt_min_s = max(1e-7, self._flow_dt_min_s)
        self._flow_dt_max_s = max(self._flow_dt_min_s, self._flow_dt_max_s)
        self._flow_velocity_clip = max(1.0, self._flow_velocity_clip)
        self._velocity_scale = max(1e-6, self._velocity_scale)
        self._position_time_scale = max(1e-3, self._position_time_scale)

        self._dbscan_eps = max(1e-3, self._dbscan_eps)
        self._dbscan_min_samples = max(1, self._dbscan_min_samples)
        self._min_cluster_events = max(1, self._min_cluster_events)

        self._bridge = CvBridge()
        self._clusters_color_pub = self.create_publisher(Image, self._clusters_color_topic, 10)

        # Keep sensor QoS for /events_compensated compatibility.
        mask_sub = Subscriber(
            self,
            Image,
            self._input_mask_topic,
            qos_profile=qos_profile_sensor_data,
        )
        events_sub = Subscriber(
            self,
            EventArray,
            self._events_comp_topic,
            qos_profile=qos_profile_sensor_data,
        )

        self._sync = ApproximateTimeSynchronizer(
            [mask_sub, events_sub],
            queue_size=self._sync_queue_size,
            slop=self._sync_slop,
        )
        self._sync.registerCallback(self._on_synced_inputs)

        self.get_logger().info(
            "event_clustering_2_0 ready. "
            f"in_mask={self._input_mask_topic} events={self._events_comp_topic} "
            f"out_clusters_img={self._clusters_color_topic}"
        )

    def _publish_bgr(self, image_bgr: np.ndarray, header, publisher) -> None:
        out_msg = self._bridge.cv2_to_imgmsg(image_bgr, encoding="bgr8")
        out_msg.header = header
        publisher.publish(out_msg)

    def _on_synced_inputs(self, mask_msg: Image, events_msg: EventArray) -> None:
        mask = self._bridge.imgmsg_to_cv2(mask_msg, desired_encoding="mono8")
        if mask is None:
            return

        if self._median_filter_enable:
            mask = cv2.medianBlur(mask, self._median_filter_kernel)

        points_xy, points_t = self._extract_events(events_msg, mask)
        cluster_count = 0
        kept_events = 0
        h, w = mask.shape
        color_img = np.zeros((h, w, 3), dtype=np.uint8)

        if points_xy.shape[0] > 0:
            points_xy, points_t = self._downsample_events(points_xy, points_t)
            flow = self._estimate_local_flow(points_xy, points_t)
            labels = self._dbscan_joint(points_xy, points_t, flow)
            labels = self._drop_small_clusters(labels)

            valid = labels >= 0
            kept_events = int(np.count_nonzero(valid))
            if kept_events > 0:
                cluster_ids = np.unique(labels[valid])
                cluster_count = int(cluster_ids.size)

                for cluster_id in cluster_ids:
                    pts = points_xy[labels == cluster_id]
                    xy = np.rint(pts).astype(np.int32)
                    in_bounds = (
                        (xy[:, 0] >= 0)
                        & (xy[:, 0] < w)
                        & (xy[:, 1] >= 0)
                        & (xy[:, 1] < h)
                    )
                    xy = xy[in_bounds]
                    if xy.shape[0] == 0:
                        continue

                    color_img[xy[:, 1], xy[:, 0]] = (255, 255, 255)

        self._publish_bgr(color_img, mask_msg.header, self._clusters_color_pub)

        if self._log_stats:
            in_px = int(np.count_nonzero(mask >= self._input_mask_min_value))
            self.get_logger().info(
                f"in_px={in_px} selected_events={points_xy.shape[0]} "
                f"clusters={cluster_count} kept_events={kept_events}"
            )

    def _extract_events(self, events_msg: EventArray, mask: np.ndarray):
        """Select compensated events, optionally gated by input dynamic mask."""
        h, w = mask.shape
        rows = []
        use_mask = self._use_input_mask

        for evt in events_msg.events:
            x = int(evt.x)
            y = int(evt.y)
            if x < 0 or x >= w or y < 0 or y >= h:
                continue
            if use_mask and mask[y, x] < self._input_mask_min_value:
                continue
            rows.append((float(x), float(y), to_sec(evt.ts)))

        if not rows:
            return np.empty((0, 2), dtype=np.float32), np.empty((0,), dtype=np.float32)

        arr = np.asarray(rows, dtype=np.float64)
        order = np.argsort(arr[:, 2], kind="mergesort")
        arr = arr[order]
        t0 = float(arr[0, 2])
        t_rel = (arr[:, 2] - t0).astype(np.float32)
        xy = arr[:, :2].astype(np.float32)
        return xy, t_rel

    def _downsample_events(self, points_xy: np.ndarray, points_t: np.ndarray):
        n = points_xy.shape[0]
        if n <= self._max_events:
            return points_xy, points_t
        idx = np.linspace(0, n - 1, self._max_events, dtype=np.int32)
        return points_xy[idx], points_t[idx]

    def _estimate_local_flow(self, points_xy: np.ndarray, points_t: np.ndarray) -> np.ndarray:
        """Estimate local 2D velocities from neighborhood continuity in space-time."""
        n = points_xy.shape[0]
        flow = np.zeros((n, 2), dtype=np.float32)
        if n <= 1:
            return flow

        radius2 = self._flow_search_radius_px * self._flow_search_radius_px
        recent = deque()

        for i in range(n):
            t_i = float(points_t[i])
            while recent and (t_i - float(points_t[recent[0]]) > self._flow_dt_max_s):
                recent.popleft()

            best_j = -1
            best_cost = float("inf")
            x_i = float(points_xy[i, 0])
            y_i = float(points_xy[i, 1])

            for j in reversed(recent):
                dt = t_i - float(points_t[j])
                if dt <= self._flow_dt_min_s:
                    continue
                dx = x_i - float(points_xy[j, 0])
                dy = y_i - float(points_xy[j, 1])
                d2 = dx * dx + dy * dy
                if d2 > radius2:
                    continue
                cost = d2 + 0.01 * dt
                if cost < best_cost:
                    best_cost = cost
                    best_j = j
                    if d2 <= 1.0:
                        break

            if best_j >= 0:
                dt = t_i - float(points_t[best_j])
                if dt > 0.0:
                    flow[i, 0] = (x_i - float(points_xy[best_j, 0])) / dt
                    flow[i, 1] = (y_i - float(points_xy[best_j, 1])) / dt

            recent.append(i)

        np.clip(flow, -self._flow_velocity_clip, self._flow_velocity_clip, out=flow)
        flow *= self._velocity_scale
        return flow

    def _region_query(self, idx: int, pos3: np.ndarray, flow2: np.ndarray) -> np.ndarray:
        dp = np.linalg.norm(pos3 - pos3[idx], axis=1)
        dv = np.linalg.norm(flow2 - flow2[idx], axis=1)
        dist = self._dbscan_wp * dp + self._dbscan_wv * dv
        return np.flatnonzero(dist <= self._dbscan_eps)

    def _dbscan_joint(self, points_xy: np.ndarray, points_t: np.ndarray, flow2: np.ndarray) -> np.ndarray:
        n = points_xy.shape[0]
        labels = -np.ones(n, dtype=np.int32)
        if n == 0:
            return labels

        pos3 = np.column_stack(
            [
                points_xy[:, 0],
                points_xy[:, 1],
                points_t * self._position_time_scale,
            ]
        ).astype(np.float32)

        visited = np.zeros(n, dtype=bool)
        cluster_id = 0

        for i in range(n):
            if visited[i]:
                continue

            visited[i] = True
            neighbors = self._region_query(i, pos3, flow2)
            if neighbors.size < self._dbscan_min_samples:
                continue

            labels[i] = cluster_id
            seeds = deque(int(nb) for nb in neighbors if int(nb) != i)
            in_seeds = np.zeros(n, dtype=bool)
            for nb in list(seeds):
                in_seeds[nb] = True

            while seeds:
                j = seeds.popleft()
                in_seeds[j] = False

                if not visited[j]:
                    visited[j] = True
                    neighbors_j = self._region_query(j, pos3, flow2)
                    if neighbors_j.size >= self._dbscan_min_samples:
                        for nb in neighbors_j:
                            nb = int(nb)
                            if not in_seeds[nb]:
                                seeds.append(nb)
                                in_seeds[nb] = True

                if labels[j] == -1:
                    labels[j] = cluster_id

            cluster_id += 1

        return labels

    def _drop_small_clusters(self, labels: np.ndarray) -> np.ndarray:
        valid = labels >= 0
        if not np.any(valid):
            return labels

        counts = np.bincount(labels[valid])
        keep_ids = np.flatnonzero(counts >= self._min_cluster_events)
        if keep_ids.size == 0:
            return -np.ones_like(labels)

        out = labels.copy()
        out[~np.isin(labels, keep_ids)] = -1
        return out


def main() -> None:
    rclpy.init()
    node = EventClustering20Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
