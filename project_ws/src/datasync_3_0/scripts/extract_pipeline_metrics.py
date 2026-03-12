#!/usr/bin/env python3
"""
Extract pipeline metrics (M1..M6) from live ROS 2 topics while replaying a bag.

Pipeline assumed:
  - datasync_3_0 (motion compensation)
  - event_segmentation
  - event_clustering_2_0

Outputs:
  - one summary CSV row (append mode) with aggregated metrics over a time window.

Metrics covered:
  M1: compensation latency (events -> events_compensated)
  M2: compensation quality on event images
      - event density gain: D_post vs D_pre
      - blur reduction: (blur_pre - blur_post) / blur_pre
  M3: post-compensation isolated-event ratio (iso_post) from /event_noise_metrics
  M4: foreground event ratio from /event_noise_metrics
  M5: segmentation latency (time_image/count_image -> event_mask)
  M6: clustering stability from /event_objects_mask
      - variance of cluster count K_t
      - mean IoU between consecutive object masks
"""

import argparse
import csv
import math
import os
import time
from collections import defaultdict, deque
from datetime import datetime
from typing import Any, Deque, Dict, List, Optional, Tuple

import numpy as np

try:
    import rclpy
    from dv_ros2_msgs.msg import EventArray
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image
    from std_msgs.msg import Float32MultiArray

    _HAVE_ROS_IMPORTS = True
except ModuleNotFoundError:
    rclpy = None
    Node = object  # type: ignore[assignment]
    qos_profile_sensor_data = None  # type: ignore[assignment]
    EventArray = Any  # type: ignore[assignment]
    Image = Any  # type: ignore[assignment]
    Float32MultiArray = Any  # type: ignore[assignment]
    _HAVE_ROS_IMPORTS = False

try:
    import cv2  # type: ignore

    _HAVE_CV2 = True
except Exception:
    cv2 = None
    _HAVE_CV2 = False


def stamp_to_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def finite_only(values: List[float]) -> List[float]:
    return [float(v) for v in values if math.isfinite(v)]


def safe_mean(values: List[float]) -> float:
    vals = finite_only(values)
    if not vals:
        return float("nan")
    return float(np.mean(vals))


def safe_median(values: List[float]) -> float:
    vals = finite_only(values)
    if not vals:
        return float("nan")
    return float(np.median(vals))


def safe_var(values: List[float]) -> float:
    vals = finite_only(values)
    if not vals:
        return float("nan")
    return float(np.var(vals))


def safe_std(values: List[float]) -> float:
    vals = finite_only(values)
    if not vals:
        return float("nan")
    return float(np.std(vals))


def safe_percentile(values: List[float], p: float) -> float:
    vals = finite_only(values)
    if not vals:
        return float("nan")
    return float(np.percentile(vals, p))


def append_summary_csv(path: str, row: Dict[str, object]) -> None:
    directory = os.path.dirname(path)
    if directory:
        os.makedirs(directory, exist_ok=True)
    exists = os.path.exists(path)
    with open(path, "a", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(row.keys()))
        if not exists:
            writer.writeheader()
        writer.writerow(row)


class PipelineMetricsCollector(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("pipeline_metrics_collector")

        self.args = args
        self.sync_tol_ns = int(max(0.0, args.sync_tolerance_ms) * 1e6)
        self.max_cache_age_ns = int(max(0.5, args.cache_age_s) * 1e9)
        self.max_latency_ms = float(args.max_latency_ms)
        self.sensor_width = max(1, int(args.sensor_width))
        self.sensor_height = max(1, int(args.sensor_height))
        self.blur_epsilon = max(1e-12, float(args.blur_epsilon))

        # Caches for timestamp-based pairing.
        self.events_in_batch_by_stamp: Dict[
            int, Deque[Tuple[int, Dict[str, float]]]
        ] = defaultdict(deque)
        self.time_recv_by_stamp: Dict[int, Deque[int]] = defaultdict(deque)
        self.count_recv_by_stamp: Dict[int, Deque[int]] = defaultdict(deque)

        # Metric samples.
        self.comp_lat_ms: List[float] = []       # M1
        self.seg_lat_ms: List[float] = []        # M5
        self.m2_density_pre: List[float] = []       # M2
        self.m2_density_post: List[float] = []      # M2
        self.m2_density_gain: List[float] = []      # M2
        self.m2_blur_pre: List[float] = []          # M2
        self.m2_blur_post: List[float] = []         # M2
        self.m2_blur_reduction: List[float] = []    # M2
        self.warp_valid_ratio: List[float] = []     # diagnostic (legacy)
        self.iso_post: List[float] = []          # M3
        self.fg_ratio: List[float] = []          # M4
        self.hot_post: List[float] = []
        self.small_cc_post: List[float] = []
        self.cluster_count: List[float] = []     # M6 (K_t)
        self.mask_iou: List[float] = []          # M6 continuity proxy
        self.prev_obj_mask: Optional[np.ndarray] = None

        self._warned_no_cv2 = False

        self.create_subscription(
            EventArray, args.events_topic, self._on_events, qos_profile_sensor_data
        )
        self.create_subscription(
            EventArray,
            args.events_comp_topic,
            self._on_events_comp,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Float32MultiArray,
            args.noise_metrics_topic,
            self._on_noise_metrics,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image, args.time_image_topic, self._on_time_image, qos_profile_sensor_data
        )
        self.create_subscription(
            Image, args.count_image_topic, self._on_count_image, qos_profile_sensor_data
        )
        self.create_subscription(
            Image, args.mask_topic, self._on_mask, qos_profile_sensor_data
        )
        self.create_subscription(
            Image, args.objects_mask_topic, self._on_objects_mask, qos_profile_sensor_data
        )

        # Periodic cache pruning.
        self.create_timer(1.0, self._prune_caches)

        self.get_logger().info(
            "Collecting metrics for "
            f"{args.duration:.1f}s on topics: "
            f"events={args.events_topic} "
            f"events_comp={args.events_comp_topic} "
            f"noise={args.noise_metrics_topic} "
            f"time={args.time_image_topic} "
            f"count={args.count_image_topic} "
            f"mask={args.mask_topic} "
            f"objects_mask={args.objects_mask_topic}"
        )

    def _now_ns(self) -> int:
        return int(self.get_clock().now().nanoseconds)

    def _cache_push(self, cache: Dict[int, Deque[int]], stamp_ns: int, recv_ns: int) -> None:
        dq = cache[stamp_ns]
        dq.append(recv_ns)
        # Protect against pathological growth for repeated equal stamps.
        while len(dq) > 8:
            dq.popleft()

    def _cache_push_event_batch(
        self, stamp_ns: int, recv_ns: int, batch_metrics: Dict[str, float]
    ) -> None:
        dq = self.events_in_batch_by_stamp[stamp_ns]
        dq.append((recv_ns, batch_metrics))
        while len(dq) > 8:
            dq.popleft()

    def _cache_pop_event_batch(
        self, stamp_ns: int
    ) -> Optional[Tuple[int, Dict[str, float]]]:
        dq = self.events_in_batch_by_stamp.get(stamp_ns)
        if not dq:
            return None
        item = dq.popleft()
        if not dq:
            del self.events_in_batch_by_stamp[stamp_ns]
        return item

    def _cache_pop_stamp(
        self, cache: Dict[int, Deque[int]], stamp_ns: int, tolerance_ns: int
    ) -> Optional[int]:
        # Exact match first.
        dq = cache.get(stamp_ns)
        if dq:
            recv_ns = dq.popleft()
            if not dq:
                del cache[stamp_ns]
            return recv_ns

        if tolerance_ns <= 0 or not cache:
            return None

        # Nearest stamp within tolerance.
        best_key = None
        best_delta = tolerance_ns + 1
        for key, q in cache.items():
            if not q:
                continue
            delta = abs(key - stamp_ns)
            if delta < best_delta:
                best_delta = delta
                best_key = key

        if best_key is None:
            return None

        qbest = cache[best_key]
        recv_ns = qbest.popleft()
        if not qbest:
            del cache[best_key]
        return recv_ns

    def _prune_cache(self, cache: Dict[int, Deque[int]], now_ns: int) -> None:
        min_recv_ns = now_ns - self.max_cache_age_ns
        to_delete = []
        for stamp_ns, dq in cache.items():
            while dq and dq[0] < min_recv_ns:
                dq.popleft()
            if not dq:
                to_delete.append(stamp_ns)
        for stamp_ns in to_delete:
            del cache[stamp_ns]

    def _prune_event_batch_cache(self, now_ns: int) -> None:
        min_recv_ns = now_ns - self.max_cache_age_ns
        to_delete = []
        for stamp_ns, dq in self.events_in_batch_by_stamp.items():
            while dq and dq[0][0] < min_recv_ns:
                dq.popleft()
            if not dq:
                to_delete.append(stamp_ns)
        for stamp_ns in to_delete:
            del self.events_in_batch_by_stamp[stamp_ns]

    def _resolve_sensor_dims(
        self,
        msg: EventArray,
        fallback_width: Optional[int] = None,
        fallback_height: Optional[int] = None,
    ) -> Tuple[int, int]:
        width = int(getattr(msg, "width", 0))
        height = int(getattr(msg, "height", 0))
        if width <= 0:
            width = (
                int(fallback_width)
                if fallback_width is not None and int(fallback_width) > 0
                else self.sensor_width
            )
        if height <= 0:
            height = (
                int(fallback_height)
                if fallback_height is not None and int(fallback_height) > 0
                else self.sensor_height
            )
        return max(1, width), max(1, height)

    def _sharpness_score(self, img: np.ndarray) -> float:
        if img.size == 0:
            return float("nan")
        if _HAVE_CV2:
            val = float(cv2.Laplacian(img, cv2.CV_32F).var())
            return max(0.0, val) if math.isfinite(val) else float("nan")

        gx = np.diff(img, axis=1)
        gy = np.diff(img, axis=0)
        val = float(np.mean(gx * gx) + np.mean(gy * gy))
        return max(0.0, val) if math.isfinite(val) else float("nan")

    def _event_batch_metrics(
        self,
        msg: EventArray,
        fallback_width: Optional[int] = None,
        fallback_height: Optional[int] = None,
    ) -> Dict[str, float]:
        width, height = self._resolve_sensor_dims(
            msg, fallback_width=fallback_width, fallback_height=fallback_height
        )
        n_total = len(msg.events)
        if n_total <= 0:
            return {
                "width": float(width),
                "height": float(height),
                "n_valid_events": 0.0,
                "active_pixels": 0.0,
                "density": float("nan"),
                "sharpness": float("nan"),
                "blur": float("nan"),
            }

        xs: List[int] = []
        ys: List[int] = []
        for evt in msg.events:
            x = int(evt.x)
            y = int(evt.y)
            if 0 <= x < width and 0 <= y < height:
                xs.append(x)
                ys.append(y)

        n_valid = len(xs)
        if n_valid <= 0:
            return {
                "width": float(width),
                "height": float(height),
                "n_valid_events": 0.0,
                "active_pixels": 0.0,
                "density": float("nan"),
                "sharpness": float("nan"),
                "blur": float("nan"),
            }

        img = np.zeros((height, width), dtype=np.float32)
        xarr = np.asarray(xs, dtype=np.int64)
        yarr = np.asarray(ys, dtype=np.int64)
        np.add.at(img, (yarr, xarr), 1.0)

        active_pixels = int(np.count_nonzero(img))
        density = (
            float(n_valid) / float(active_pixels)
            if active_pixels > 0
            else float("nan")
        )

        max_v = float(np.max(img)) if img.size > 0 else 0.0
        if max_v > 0.0:
            img = img / max_v
        sharpness = self._sharpness_score(img)
        blur = (
            1.0 / (sharpness + self.blur_epsilon)
            if math.isfinite(sharpness)
            else float("nan")
        )

        return {
            "width": float(width),
            "height": float(height),
            "n_valid_events": float(n_valid),
            "active_pixels": float(active_pixels),
            "density": float(density),
            "sharpness": float(sharpness),
            "blur": float(blur),
        }

    def _prune_caches(self) -> None:
        now_ns = self._now_ns()
        self._prune_event_batch_cache(now_ns)
        self._prune_cache(self.time_recv_by_stamp, now_ns)
        self._prune_cache(self.count_recv_by_stamp, now_ns)

    def _on_events(self, msg: EventArray) -> None:
        recv_ns = self._now_ns()
        stamp_ns = stamp_to_ns(msg.header.stamp)
        pre_metrics = self._event_batch_metrics(msg)
        self._cache_push_event_batch(stamp_ns, recv_ns, pre_metrics)

    def _on_events_comp(self, msg: EventArray) -> None:
        recv_ns = self._now_ns()
        stamp_ns = stamp_to_ns(msg.header.stamp)
        batch = self._cache_pop_event_batch(stamp_ns)
        if batch is None:
            return
        in_recv_ns, pre_metrics = batch

        latency_ms = (recv_ns - in_recv_ns) / 1e6
        if 0.0 <= latency_ms <= self.max_latency_ms:
            self.comp_lat_ms.append(float(latency_ms))

        post_metrics = self._event_batch_metrics(
            msg,
            fallback_width=int(pre_metrics["width"]),
            fallback_height=int(pre_metrics["height"]),
        )

        pre_density = float(pre_metrics["density"])
        post_density = float(post_metrics["density"])
        if math.isfinite(pre_density) and math.isfinite(post_density):
            self.m2_density_pre.append(pre_density)
            self.m2_density_post.append(post_density)
            if pre_density > 0.0:
                self.m2_density_gain.append((post_density - pre_density) / pre_density)

        pre_blur = float(pre_metrics["blur"])
        post_blur = float(post_metrics["blur"])
        if math.isfinite(pre_blur) and math.isfinite(post_blur):
            self.m2_blur_pre.append(pre_blur)
            self.m2_blur_post.append(post_blur)
            if pre_blur > 0.0:
                self.m2_blur_reduction.append((pre_blur - post_blur) / pre_blur)

    def _on_noise_metrics(self, msg: Float32MultiArray) -> None:
        # Layout from datasync_3_0/scripts/filter_sweep_metrics.py
        # 0 noise_score (legacy), 2 iso_post, 4 hot_post, 5 small_cc_post,
        # 6 pre_events, 7 post_events, 8 fg_event_ratio
        if len(msg.data) < 16:
            return

        pre_events = float(msg.data[6])
        post_events = float(msg.data[7])
        warp_valid = post_events / pre_events if pre_events > 0.0 else float("nan")

        self.iso_post.append(float(msg.data[2]))
        self.hot_post.append(float(msg.data[4]))
        self.small_cc_post.append(float(msg.data[5]))
        self.warp_valid_ratio.append(float(warp_valid))
        self.fg_ratio.append(float(msg.data[8]))

    def _on_time_image(self, msg: Image) -> None:
        recv_ns = self._now_ns()
        stamp_ns = stamp_to_ns(msg.header.stamp)
        self._cache_push(self.time_recv_by_stamp, stamp_ns, recv_ns)

    def _on_count_image(self, msg: Image) -> None:
        recv_ns = self._now_ns()
        stamp_ns = stamp_to_ns(msg.header.stamp)
        self._cache_push(self.count_recv_by_stamp, stamp_ns, recv_ns)

    def _on_mask(self, msg: Image) -> None:
        recv_ns = self._now_ns()
        stamp_ns = stamp_to_ns(msg.header.stamp)

        time_recv_ns = self._cache_pop_stamp(
            self.time_recv_by_stamp, stamp_ns, self.sync_tol_ns
        )
        count_recv_ns = self._cache_pop_stamp(
            self.count_recv_by_stamp, stamp_ns, self.sync_tol_ns
        )
        if time_recv_ns is None or count_recv_ns is None:
            return

        start_ns = max(time_recv_ns, count_recv_ns)
        latency_ms = (recv_ns - start_ns) / 1e6
        if 0.0 <= latency_ms <= self.max_latency_ms:
            self.seg_lat_ms.append(float(latency_ms))

    def _decode_mono8(self, msg: Image) -> Optional[np.ndarray]:
        # Expected for object mask: mono8.
        if msg.height <= 0 or msg.width <= 0:
            return None
        if msg.step <= 0 or len(msg.data) < msg.height * msg.step:
            return None
        arr = np.frombuffer(msg.data, dtype=np.uint8)
        try:
            arr = arr.reshape((msg.height, msg.step))
        except ValueError:
            return None
        mono = arr[:, : msg.width]
        return mono

    def _count_components(self, binary_mask: np.ndarray) -> float:
        if not np.any(binary_mask):
            return 0.0
        if not _HAVE_CV2:
            if not self._warned_no_cv2:
                self._warned_no_cv2 = True
                self.get_logger().warn(
                    "OpenCV not found; cluster_count metric (M6) disabled."
                )
            return float("nan")

        nlabels, _ = cv2.connectedComponents(binary_mask.astype(np.uint8), connectivity=8)
        return float(max(0, int(nlabels) - 1))

    def _on_objects_mask(self, msg: Image) -> None:
        mono = self._decode_mono8(msg)
        if mono is None:
            return
        binary = mono > 0

        k_t = self._count_components(binary)
        self.cluster_count.append(k_t)

        if self.prev_obj_mask is not None:
            inter = int(np.logical_and(self.prev_obj_mask, binary).sum())
            union = int(np.logical_or(self.prev_obj_mask, binary).sum())
            iou = 1.0 if union == 0 else float(inter) / float(union)
            self.mask_iou.append(iou)
        self.prev_obj_mask = binary

    def build_summary(self, run_name: str, duration_s: float) -> Dict[str, object]:
        m2_warp_mean = safe_mean(self.warp_valid_ratio)
        m2_density_gain_mean = safe_mean(self.m2_density_gain)
        m2_blur_reduction_mean = safe_mean(self.m2_blur_reduction)
        return {
            "run_name": run_name,
            "timestamp_utc": datetime.utcnow().isoformat(timespec="seconds"),
            "duration_s": f"{duration_s:.3f}",
            "samples_m1": len(finite_only(self.comp_lat_ms)),
            "samples_m2_m4": len(finite_only(self.iso_post)),
            "samples_m2_density": len(finite_only(self.m2_density_gain)),
            "samples_m2_blur": len(finite_only(self.m2_blur_reduction)),
            "samples_m5": len(finite_only(self.seg_lat_ms)),
            "samples_m6_k": len(finite_only(self.cluster_count)),
            "samples_m6_iou": len(finite_only(self.mask_iou)),
            # M1
            "m1_comp_lat_mean_ms": f"{safe_mean(self.comp_lat_ms):.6f}",
            "m1_comp_lat_median_ms": f"{safe_median(self.comp_lat_ms):.6f}",
            "m1_comp_lat_p95_ms": f"{safe_percentile(self.comp_lat_ms, 95.0):.6f}",
            # M2: densite + reduction de flou (etape compensation)
            "m2_density_pre_mean": f"{safe_mean(self.m2_density_pre):.6f}",
            "m2_density_post_mean": f"{safe_mean(self.m2_density_post):.6f}",
            "m2_density_gain_mean": f"{m2_density_gain_mean:.6f}",
            "m2_blur_pre_mean": f"{safe_mean(self.m2_blur_pre):.6f}",
            "m2_blur_post_mean": f"{safe_mean(self.m2_blur_post):.6f}",
            "m2_blur_reduction_mean": f"{m2_blur_reduction_mean:.6f}",
            "m2_blur_reduction_percent_mean": (
                f"{(100.0 * m2_blur_reduction_mean):.6f}"
                if math.isfinite(m2_blur_reduction_mean)
                else "nan"
            ),
            # M2 legacy diagnostic (geometrie de projection)
            "m2_keep_ratio_mean": f"{m2_warp_mean:.6f}",  # legacy alias
            "m2_warp_valid_ratio_mean": f"{m2_warp_mean:.6f}",
            "m2_projection_loss_mean": (
                f"{(1.0 - m2_warp_mean):.6f}"
                if math.isfinite(m2_warp_mean)
                else "nan"
            ),
            # M3
            "m3_iso_post_mean": f"{safe_mean(self.iso_post):.6f}",
            "m3_hot_post_mean": f"{safe_mean(self.hot_post):.6f}",
            "m3_small_cc_post_mean": f"{safe_mean(self.small_cc_post):.6f}",
            # M4
            "m4_fg_ratio_mean": f"{safe_mean(self.fg_ratio):.6f}",
            # M5
            "m5_seg_lat_mean_ms": f"{safe_mean(self.seg_lat_ms):.6f}",
            "m5_seg_lat_median_ms": f"{safe_median(self.seg_lat_ms):.6f}",
            "m5_seg_lat_p95_ms": f"{safe_percentile(self.seg_lat_ms, 95.0):.6f}",
            # M6
            "m6_cluster_count_mean": f"{safe_mean(self.cluster_count):.6f}",
            "m6_cluster_count_var": f"{safe_var(self.cluster_count):.6f}",
            "m6_cluster_count_std": f"{safe_std(self.cluster_count):.6f}",
            "m6_mask_iou_mean": f"{safe_mean(self.mask_iou):.6f}",
            "m6_mask_iou_std": f"{safe_std(self.mask_iou):.6f}",
        }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Extract M1..M6 metrics from running ROS2 event pipeline."
    )
    parser.add_argument("--duration", type=float, default=20.0, help="Acquisition duration (s).")
    parser.add_argument("--run-name", type=str, default="", help="Run identifier saved in CSV.")
    parser.add_argument(
        "--summary-csv",
        type=str,
        default="pipeline_metrics_summary.csv",
        help="Path to summary CSV (append mode).",
    )

    parser.add_argument("--events-topic", type=str, default="/events")
    parser.add_argument("--events-comp-topic", type=str, default="/events_compensated")
    parser.add_argument("--noise-metrics-topic", type=str, default="/event_noise_metrics")
    parser.add_argument("--time-image-topic", type=str, default="/time_image")
    parser.add_argument("--count-image-topic", type=str, default="/count_image")
    parser.add_argument("--mask-topic", type=str, default="/event_mask")
    parser.add_argument("--objects-mask-topic", type=str, default="/event_objects_mask")

    parser.add_argument(
        "--sync-tolerance-ms",
        type=float,
        default=20.0,
        help="Header timestamp tolerance when pairing /time_image and /count_image.",
    )
    parser.add_argument(
        "--cache-age-s",
        type=float,
        default=5.0,
        help="Drop unmatched cached timestamps older than this age (s).",
    )
    parser.add_argument(
        "--max-latency-ms",
        type=float,
        default=1000.0,
        help="Discard latency samples above this threshold (ms).",
    )
    parser.add_argument(
        "--sensor-width",
        type=int,
        default=346,
        help="Fallback sensor width when EventArray width is unset.",
    )
    parser.add_argument(
        "--sensor-height",
        type=int,
        default=260,
        help="Fallback sensor height when EventArray height is unset.",
    )
    parser.add_argument(
        "--blur-epsilon",
        type=float,
        default=1e-6,
        help="Small positive epsilon used in blur proxy: blur=1/(sharpness+eps).",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.duration <= 0.0:
        raise ValueError("--duration must be > 0.")

    if not _HAVE_ROS_IMPORTS:
        print(
            "Missing ROS 2 Python dependencies (e.g. dv_ros2_msgs). "
            "Source your workspace first, for example:\n"
            "  source /opt/ros/humble/setup.bash\n"
            "  source /home/nochi/NOCHI/M2_PAR/Projet_de_synthese/project_ws/install/setup.bash"
        )
        return 2

    run_name = args.run_name.strip()
    if not run_name:
        run_name = datetime.utcnow().strftime("run_%Y%m%d_%H%M%S")

    rclpy.init()
    node = PipelineMetricsCollector(args)

    start = time.monotonic()
    end = start + args.duration
    try:
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        duration_measured = time.monotonic() - start
        summary = node.build_summary(run_name=run_name, duration_s=duration_measured)
        append_summary_csv(args.summary_csv, summary)

        node.get_logger().info(f"Metrics summary appended to: {args.summary_csv}")
        for k, v in summary.items():
            node.get_logger().info(f"{k}: {v}")

        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
