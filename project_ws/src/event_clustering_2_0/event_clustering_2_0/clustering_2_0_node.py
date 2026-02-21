#!/usr/bin/env python3
"""
Event clustering 2.0 (clustering only), paper-style rendering.

Paper:
- Fig.1(d): objects only, red points on white background.
- Clustering metric Eq.(12): wp*||p_i-p_j|| + wv*||v_i-v_j||.
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

try:
    from scipy.spatial import cKDTree  # type: ignore
    _HAVE_SCIPY = True
except Exception:
    cKDTree = None
    _HAVE_SCIPY = False


def to_sec(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def color_from_id(k: int) -> tuple:
    b = (29 * (k + 1)) % 255
    g = (17 * (k + 1)) % 255
    r = (37 * (k + 1)) % 255
    return (int(b), int(g), int(r))


class EventClustering20Node(Node):
    def __init__(self) -> None:
        super().__init__("event_clustering_2_0")

        # Inputs
        self.declare_parameter("input_mask_topic", "/event_mask")
        self.declare_parameter("events_comp_topic", "/events_compensated")

        # Outputs (paper style)
        self.declare_parameter("objects_vis_topic", "/event_objects_vis")    # bgr8 like Fig.1(d)
        self.declare_parameter("objects_mask_topic", "/event_objects_mask")  # mono8 0/255

        # Sync/log
        self.declare_parameter("sync_queue_size", 10)
        self.declare_parameter("sync_slop", 0.05)
        self.declare_parameter("log_stats", True)

        # Event selection
        self.declare_parameter("use_input_mask", True)
        self.declare_parameter("input_mask_min_value", 1)
        self.declare_parameter("mask_fallback_enable", True)
        self.declare_parameter("mask_fallback_min_selected", 15)
        self.declare_parameter("mask_fallback_on_empty_clusters", True)

        # Median filtering on mask
        self.declare_parameter("median_filter_enable", True)
        self.declare_parameter("median_filter_kernel", 3)

        # --- NEW: Spatio-temporal event filtering (anti isolated noise) ---
        # Keep e_i if exists >= st_filter_min_neighbors events e_j s.t.
        # ||x_i - x_j|| <= d_max AND |t_i - t_j| <= dt_max.
        self.declare_parameter("st_filter_enable", True)
        self.declare_parameter("st_filter_d_max_px", 3.0)
        self.declare_parameter("st_filter_dt_max_s", 0.01)
        self.declare_parameter("st_filter_min_neighbors", 1)
        self.declare_parameter("st_filter_use_kdtree", True)

        # Event budget
        self.declare_parameter("max_events", 3000)

        # Local flow
        self.declare_parameter("flow_search_radius_px", 4.0)
        self.declare_parameter("flow_dt_min_s", 1e-5)
        self.declare_parameter("flow_dt_max_s", 0.03)
        self.declare_parameter("flow_velocity_clip", 5000.0)
        self.declare_parameter("velocity_scale", 1.0)

        # p=(x,y,t_scaled)
        self.declare_parameter("position_time_scale", 1000.0)

        # DBSCAN Eq(12)
        self.declare_parameter("dbscan_eps", 20.0)
        self.declare_parameter("dbscan_min_samples", 6)
        self.declare_parameter("dbscan_wp", 1.0)
        self.declare_parameter("dbscan_wv", 0.002)
        self.declare_parameter("min_cluster_events", 10)

        # Rendering
        self.declare_parameter("paper_style", True)  # True => all objects red on white background
        self.declare_parameter("draw_each_cluster_color", False)  # used only if paper_style=False
        self.declare_parameter("bg_white", True)
        self.declare_parameter("object_bgr", [0, 0, 255])  # red
        self.declare_parameter("background_bgr", [255, 255, 255])  # white

        # Read
        self._input_mask_topic = str(self.get_parameter("input_mask_topic").value)
        self._events_comp_topic = str(self.get_parameter("events_comp_topic").value)
        self._objects_vis_topic = str(self.get_parameter("objects_vis_topic").value)
        self._objects_mask_topic = str(self.get_parameter("objects_mask_topic").value)

        self._sync_queue_size = max(1, int(self.get_parameter("sync_queue_size").value))
        self._sync_slop = max(0.0, float(self.get_parameter("sync_slop").value))
        self._log_stats = bool(self.get_parameter("log_stats").value)

        self._use_input_mask = bool(self.get_parameter("use_input_mask").value)
        self._input_mask_min_value = int(self.get_parameter("input_mask_min_value").value)
        self._mask_fallback_enable = bool(self.get_parameter("mask_fallback_enable").value)
        self._mask_fallback_min_selected = max(1, int(self.get_parameter("mask_fallback_min_selected").value))
        self._mask_fallback_on_empty_clusters = bool(self.get_parameter("mask_fallback_on_empty_clusters").value)

        self._median_filter_enable = bool(self.get_parameter("median_filter_enable").value)
        self._median_filter_kernel = max(1, int(self.get_parameter("median_filter_kernel").value))
        if self._median_filter_kernel % 2 == 0:
            self._median_filter_kernel += 1

        # NEW: spatio-temporal filter params
        self._st_filter_enable = bool(self.get_parameter("st_filter_enable").value)
        self._st_filter_d_max_px = float(self.get_parameter("st_filter_d_max_px").value)
        self._st_filter_dt_max_s = float(self.get_parameter("st_filter_dt_max_s").value)
        self._st_filter_min_neighbors = int(self.get_parameter("st_filter_min_neighbors").value)
        self._st_filter_use_kdtree = bool(self.get_parameter("st_filter_use_kdtree").value)

        self._st_filter_d_max_px = max(0.1, self._st_filter_d_max_px)
        self._st_filter_dt_max_s = max(1e-6, self._st_filter_dt_max_s)
        self._st_filter_min_neighbors = max(1, self._st_filter_min_neighbors)

        self._max_events = max(100, int(self.get_parameter("max_events").value))

        self._flow_search_radius_px = max(0.1, float(self.get_parameter("flow_search_radius_px").value))
        self._flow_dt_min_s = max(1e-7, float(self.get_parameter("flow_dt_min_s").value))
        self._flow_dt_max_s = max(self._flow_dt_min_s, float(self.get_parameter("flow_dt_max_s").value))
        self._flow_velocity_clip = max(1.0, float(self.get_parameter("flow_velocity_clip").value))
        self._velocity_scale = max(1e-6, float(self.get_parameter("velocity_scale").value))

        self._position_time_scale = max(1e-3, float(self.get_parameter("position_time_scale").value))

        self._dbscan_eps = max(1e-3, float(self.get_parameter("dbscan_eps").value))
        self._dbscan_min_samples = max(1, int(self.get_parameter("dbscan_min_samples").value))
        self._dbscan_wp = float(self.get_parameter("dbscan_wp").value)
        self._dbscan_wv = float(self.get_parameter("dbscan_wv").value)
        self._min_cluster_events = max(1, int(self.get_parameter("min_cluster_events").value))

        self._paper_style = bool(self.get_parameter("paper_style").value)
        self._draw_each_cluster_color = bool(self.get_parameter("draw_each_cluster_color").value)
        self._bg_white = bool(self.get_parameter("bg_white").value)
        self._object_bgr = tuple(int(x) for x in self.get_parameter("object_bgr").value)
        self._background_bgr = tuple(int(x) for x in self.get_parameter("background_bgr").value)

        if self._st_filter_use_kdtree and self._st_filter_enable and not _HAVE_SCIPY:
            self.get_logger().warn(
                "SciPy not found -> spatio-temporal KDTree filter will fallback to a sliding-time implementation. "
                "Install: sudo apt install python3-scipy (or pip install scipy)."
            )
        if not _HAVE_SCIPY:
            self.get_logger().warn(
                "SciPy not found -> DBSCAN KDTree acceleration disabled. "
                "Install: sudo apt install python3-scipy (or pip install scipy)."
            )

        self._bridge = CvBridge()
        self._objects_vis_pub = self.create_publisher(Image, self._objects_vis_topic, 10)
        self._objects_mask_pub = self.create_publisher(Image, self._objects_mask_topic, 10)

        mask_sub = Subscriber(self, Image, self._input_mask_topic, qos_profile=qos_profile_sensor_data)
        events_sub = Subscriber(self, EventArray, self._events_comp_topic, qos_profile=qos_profile_sensor_data)

        self._sync = ApproximateTimeSynchronizer(
            [mask_sub, events_sub],
            queue_size=self._sync_queue_size,
            slop=self._sync_slop,
        )
        self._sync.registerCallback(self._on_synced_inputs)

        self.get_logger().info(
            "event_clustering_2_0 ready. "
            f"in_mask={self._input_mask_topic} events={self._events_comp_topic} "
            f"out_vis={self._objects_vis_topic} out_mask={self._objects_mask_topic}"
        )

    def _publish_bgr(self, img_bgr: np.ndarray, header, pub) -> None:
        msg = self._bridge.cv2_to_imgmsg(img_bgr, encoding="bgr8")
        msg.header = header
        pub.publish(msg)

    def _publish_mono(self, img_mono: np.ndarray, header, pub) -> None:
        msg = self._bridge.cv2_to_imgmsg(img_mono, encoding="mono8")
        msg.header = header
        pub.publish(msg)

    # ------------------- NEW: spatio-temporal filtering -------------------

    def _spatiotemporal_filter(self, points_xy: np.ndarray, points_t: np.ndarray):
        """
        Keep point i if it has >=K neighbors j such that:
        ||xy_i - xy_j|| <= d_max AND |t_i - t_j| <= dt_max.
        """
        n = points_xy.shape[0]
        if (not self._st_filter_enable) or n < 2:
            return points_xy, points_t

        d_max = float(self._st_filter_d_max_px)
        dt_max = float(self._st_filter_dt_max_s)
        k_min = int(self._st_filter_min_neighbors)

        keep = np.zeros(n, dtype=bool)

        # Fast path: KDTree over xy gives candidate neighbors in space,
        # then we filter by dt.
        if self._st_filter_use_kdtree and _HAVE_SCIPY and (cKDTree is not None):
            tree = cKDTree(points_xy.astype(np.float32, copy=False))

            # One call gives neighbor lists for all points (within spatial radius)
            neigh_lists = tree.query_ball_tree(tree, r=d_max)

            for i in range(n):
                cand = neigh_lists[i]
                if len(cand) <= 1:
                    continue
                # remove self
                if i in cand:
                    # cheap remove
                    cand = [j for j in cand if j != i]
                    if not cand:
                        continue
                cand = np.asarray(cand, dtype=np.int32)
                dt = np.abs(points_t[cand] - points_t[i])
                cnt = int(np.count_nonzero(dt <= dt_max))
                if cnt >= k_min:
                    keep[i] = True

        else:
            # Fallback (no SciPy): sliding window on time + spatial check.
            # Points are already sorted by time from _extract_events().
            recent = deque()
            counts = np.zeros(n, dtype=np.int32)

            for i in range(n):
                t_i = float(points_t[i])
                while recent and (t_i - float(points_t[recent[0]]) > dt_max):
                    recent.popleft()

                if recent:
                    # vectorized distance to recent points
                    idx = np.asarray(recent, dtype=np.int32)
                    dx = points_xy[i, 0] - points_xy[idx, 0]
                    dy = points_xy[i, 1] - points_xy[idx, 1]
                    d2 = dx * dx + dy * dy
                    ok = d2 <= (d_max * d_max)

                    if np.any(ok):
                        nn = idx[ok]
                        # For each spatial neighbor in the time window, increment both sides
                        counts[i] += nn.size
                        counts[nn] += 1

                recent.append(i)

            keep = counts >= k_min

        if not np.any(keep):
            return np.empty((0, 2), dtype=np.float32), np.empty((0,), dtype=np.float32)

        return points_xy[keep], points_t[keep]

    # ---------------------------------------------------------------------

    def _cluster_selected_events(self, points_xy: np.ndarray, points_t: np.ndarray):
        if points_xy.shape[0] == 0:
            return points_xy, np.empty((0,), dtype=np.int32), 0, 0, 0, 0

        points_xy, points_t = self._downsample_events(points_xy, points_t)

        st_before = int(points_xy.shape[0])
        points_xy, points_t = self._spatiotemporal_filter(points_xy, points_t)
        st_after = int(points_xy.shape[0])

        if points_xy.shape[0] == 0:
            return points_xy, np.empty((0,), dtype=np.int32), 0, 0, st_before, st_after

        flow = self._estimate_local_flow(points_xy, points_t)
        labels = self._dbscan_joint(points_xy, points_t, flow)
        labels = self._drop_small_clusters(labels)

        valid = labels >= 0
        kept_events = int(np.count_nonzero(valid))
        cluster_count = int(np.unique(labels[valid]).size) if kept_events > 0 else 0
        return points_xy, labels, cluster_count, kept_events, st_before, st_after

    def _on_synced_inputs(self, mask_msg: Image, events_msg: EventArray) -> None:
        mask = self._bridge.imgmsg_to_cv2(mask_msg, desired_encoding="mono8")
        if mask is None:
            return
        if self._median_filter_enable:
            mask = cv2.medianBlur(mask, self._median_filter_kernel)

        h, w = mask.shape

        fallback_used = False
        points_xy, points_t = self._extract_events(events_msg, mask, self._use_input_mask)
        masked_selected = int(points_xy.shape[0])

        if self._use_input_mask and self._mask_fallback_enable and masked_selected < self._mask_fallback_min_selected:
            points_xy, points_t = self._extract_events(events_msg, mask, False)
            fallback_used = True

        points_xy, labels, cluster_count, kept_events, st_before, st_after = self._cluster_selected_events(points_xy, points_t)

        if (
            self._use_input_mask and self._mask_fallback_enable and self._mask_fallback_on_empty_clusters
            and not fallback_used and masked_selected >= self._mask_fallback_min_selected and cluster_count == 0
        ):
            points_xy_all, points_t_all = self._extract_events(events_msg, mask, False)
            points_xy_fb, labels_fb, cluster_count_fb, kept_events_fb, st_before_fb, st_after_fb = self._cluster_selected_events(points_xy_all, points_t_all)
            if cluster_count_fb > 0:
                points_xy, labels, cluster_count, kept_events = points_xy_fb, labels_fb, cluster_count_fb, kept_events_fb
                st_before, st_after = st_before_fb, st_after_fb
                fallback_used = True

        # --- Paper-style rendering (Fig.1(d)): white bg, red objects only ---
        bg = np.full((h, w, 3), self._background_bgr, dtype=np.uint8) if self._bg_white else np.zeros((h, w, 3), dtype=np.uint8)
        obj_mask = np.zeros((h, w), dtype=np.uint8)

        if kept_events > 0:
            valid = labels >= 0
            cluster_ids = np.unique(labels[valid])

            for cid in cluster_ids:
                pts = points_xy[labels == cid]
                xy = np.rint(pts).astype(np.int32)

                inb = (xy[:, 0] >= 0) & (xy[:, 0] < w) & (xy[:, 1] >= 0) & (xy[:, 1] < h)
                xy = xy[inb]
                if xy.shape[0] == 0:
                    continue

                if self._paper_style:
                    color = self._object_bgr
                else:
                    color = color_from_id(int(cid)) if self._draw_each_cluster_color else self._object_bgr

                bg[xy[:, 1], xy[:, 0]] = color
                obj_mask[xy[:, 1], xy[:, 0]] = 255

        self._publish_bgr(bg, mask_msg.header, self._objects_vis_pub)
        self._publish_mono(obj_mask, mask_msg.header, self._objects_mask_pub)

        if self._log_stats:
            in_px = int(np.count_nonzero(mask >= self._input_mask_min_value))
            self.get_logger().info(
                f"in_px={in_px} masked_selected={masked_selected} "
                f"st_filter={st_before}->{st_after} "
                f"selected_events={points_xy.shape[0]} clusters={cluster_count} kept_events={kept_events} "
                f"fallback={fallback_used}"
            )

    def _extract_events(self, events_msg: EventArray, mask: np.ndarray, use_mask: bool):
        h, w = mask.shape
        rows = []
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

    def _build_pos3(self, points_xy: np.ndarray, points_t: np.ndarray) -> np.ndarray:
        return np.column_stack([points_xy[:, 0], points_xy[:, 1], points_t * self._position_time_scale]).astype(np.float32)

    def _region_query_bruteforce(self, idx: int, pos3: np.ndarray, flow2: np.ndarray) -> np.ndarray:
        dp = np.linalg.norm(pos3 - pos3[idx], axis=1)
        dv = np.linalg.norm(flow2 - flow2[idx], axis=1)
        dist = self._dbscan_wp * dp + self._dbscan_wv * dv
        return np.flatnonzero(dist <= self._dbscan_eps)

    def _region_query_kdtree(self, idx: int, pos3: np.ndarray, flow2: np.ndarray, kdtree) -> np.ndarray:
        wp = float(self._dbscan_wp)
        if wp <= 0.0:
            wp = 1e-6
        r_pos = float(self._dbscan_eps) / wp

        cand = kdtree.query_ball_point(pos3[idx], r=r_pos)
        if not cand:
            return np.empty((0,), dtype=np.int32)

        cand = np.asarray(cand, dtype=np.int32)
        dp = np.linalg.norm(pos3[cand] - pos3[idx], axis=1)
        dv = np.linalg.norm(flow2[cand] - flow2[idx], axis=1)
        dist = self._dbscan_wp * dp + self._dbscan_wv * dv
        return cand[dist <= self._dbscan_eps]

    def _dbscan_joint(self, points_xy: np.ndarray, points_t: np.ndarray, flow2: np.ndarray) -> np.ndarray:
        n = points_xy.shape[0]
        labels = -np.ones(n, dtype=np.int32)
        if n == 0:
            return labels

        pos3 = self._build_pos3(points_xy, points_t)
        use_kdtree = _HAVE_SCIPY and (cKDTree is not None)
        kdtree = cKDTree(pos3) if use_kdtree else None

        visited = np.zeros(n, dtype=bool)
        cluster_id = 0

        for i in range(n):
            if visited[i]:
                continue
            visited[i] = True

            neighbors = self._region_query_kdtree(i, pos3, flow2, kdtree) if use_kdtree else self._region_query_bruteforce(i, pos3, flow2)
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
                    neighbors_j = self._region_query_kdtree(j, pos3, flow2, kdtree) if use_kdtree else self._region_query_bruteforce(j, pos3, flow2)
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