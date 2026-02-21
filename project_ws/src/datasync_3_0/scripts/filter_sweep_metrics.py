#!/usr/bin/env python3
import argparse
import math
import os
import signal
import statistics
import subprocess
import sys
import time
from dataclasses import dataclass
from typing import Dict, List, Tuple

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


METRIC_FIELDS = [
    "noise_score",              # 0
    "iso_pre",                  # 1
    "iso_post",                 # 2
    "hot_pre",                  # 3
    "hot_post",                 # 4
    "small_cc_post",            # 5
    "pre_events",               # 6
    "post_events",              # 7
    "fg_event_ratio",           # 8
    "fg_pixel_ratio",           # 9
    "fg_events",                # 10
    "active_events",            # 11
    "lambda",                   # 12
    "omega_norm",               # 13
    "motion_active",            # 14
    "max_count"                 # 15
]


class MetricsCollector(Node):
    def __init__(self, topic: str) -> None:
        super().__init__("filter_sweep_metrics_collector")
        self.samples: List[Dict[str, float]] = []
        self.create_subscription(Float32MultiArray, topic, self._on_msg, 100)

    def _on_msg(self, msg: Float32MultiArray) -> None:
        if len(msg.data) < len(METRIC_FIELDS):
            return
        sample = {k: float(msg.data[i]) for i, k in enumerate(METRIC_FIELDS)}
        self.samples.append(sample)


@dataclass
class ComboResult:
    name: str
    params: Dict[str, str]
    n_total: int
    n_motion: int
    avg_noise: float
    avg_iso_post: float
    avg_hot_post: float
    avg_small_cc: float
    avg_fg_ratio: float
    avg_fg_events: float
    avg_pre_events: float
    avg_post_events: float
    avg_lambda: float
    avg_omega: float
    fg_preserve: float = 0.0
    evt_preserve: float = 0.0
    viable: bool = False


def mean(values: List[float]) -> float:
    if not values:
        return 0.0
    return float(statistics.fmean(values))


def summarize(name: str, params: Dict[str, str], samples: List[Dict[str, float]]) -> ComboResult:
    motion_samples = [s for s in samples if s["motion_active"] > 0.5]
    used = motion_samples if motion_samples else samples
    return ComboResult(
        name=name,
        params=params,
        n_total=len(samples),
        n_motion=len(motion_samples),
        avg_noise=mean([s["noise_score"] for s in used]),
        avg_iso_post=mean([s["iso_post"] for s in used]),
        avg_hot_post=mean([s["hot_post"] for s in used]),
        avg_small_cc=mean([s["small_cc_post"] for s in used]),
        avg_fg_ratio=mean([s["fg_event_ratio"] for s in used]),
        avg_fg_events=mean([s["fg_events"] for s in used]),
        avg_pre_events=mean([s["pre_events"] for s in used]),
        avg_post_events=mean([s["post_events"] for s in used]),
        avg_lambda=mean([s["lambda"] for s in used]),
        avg_omega=mean([s["omega_norm"] for s in used]),
    )


def build_launch_args(params: Dict[str, str]) -> List[str]:
    return [f"{k}:={v}" for k, v in params.items()]


def stop_process_tree(proc: subprocess.Popen) -> None:
    if proc.poll() is not None:
        return
    try:
        os.killpg(proc.pid, signal.SIGINT)
        proc.wait(timeout=5.0)
    except Exception:
        try:
            os.killpg(proc.pid, signal.SIGTERM)
            proc.wait(timeout=3.0)
        except Exception:
            try:
                os.killpg(proc.pid, signal.SIGKILL)
            except Exception:
                pass


def run_combo(
    collector: MetricsCollector,
    combo_name: str,
    combo_params: Dict[str, str],
    duration_s: float,
    warmup_s: float,
) -> ComboResult:
    collector.samples.clear()
    launch_args = build_launch_args(combo_params)
    cmd = ["ros2", "launch", "datasync_3_0", "motion_compensation.launch.py", *launch_args]

    proc = subprocess.Popen(
        cmd,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.STDOUT,
        preexec_fn=os.setsid,
    )

    try:
        warmup_end = time.monotonic() + warmup_s
        while time.monotonic() < warmup_end:
            rclpy.spin_once(collector, timeout_sec=0.1)

        collector.samples.clear()
        end = time.monotonic() + duration_s
        while time.monotonic() < end:
            rclpy.spin_once(collector, timeout_sec=0.1)

    finally:
        stop_process_tree(proc)

    return summarize(combo_name, combo_params, list(collector.samples))


def combos() -> List[Tuple[str, Dict[str, str]]]:
    base = {
        "metrics_enable": "true",
        "metrics_log_stats": "false",
        "noise_metrics_topic": "/event_noise_metrics",
        "prefilter_enable": "false",
        "prefilter_refractory_enable": "false",
        "prefilter_refractory_us": "800",
        "prefilter_ba_enable": "false",
        "prefilter_ba_radius_px": "1",
        "prefilter_ba_window_us": "2500",
        "prefilter_ba_min_neighbors": "1",
        "prefilter_ba_same_polarity_only": "false",
        "prefilter_ba_support_from_kept_only": "false",
        "prefilter_hot_pixel_enable": "false",
        "prefilter_hot_pixel_max_events_per_batch": "10",
        "postfilter_enable": "false",
        "post_refractory_enable": "false",
        "post_refractory_us": "300",
        "post_hot_pixel_enable": "false",
        "post_hot_pixel_max_events_per_pixel": "11",
        "postfilter_min_count_per_pixel": "1",
        "postfilter_neighbor_radius_px": "1",
        "postfilter_min_count_in_neighborhood": "3",
        "postfilter_min_component_pixels": "3",
        "postfilter_adaptive_enable": "true",
        "postfilter_apply_to_compensated_events": "true",
    }

    def cfg(name: str, **overrides: str) -> Tuple[str, Dict[str, str]]:
        p = dict(base)
        p.update({k: str(v).lower() if isinstance(v, bool) else str(v) for k, v in overrides.items()})
        return name, p

    return [
        cfg("all_off"),
        cfg("pre_ref_800us", prefilter_enable=True, prefilter_refractory_enable=True, prefilter_refractory_us=800),
        cfg("pre_ref_1500us", prefilter_enable=True, prefilter_refractory_enable=True, prefilter_refractory_us=1500),
        cfg("pre_ref_2500us", prefilter_enable=True, prefilter_refractory_enable=True, prefilter_refractory_us=2500),
        cfg("pre_hot_only", prefilter_enable=True, prefilter_hot_pixel_enable=True, prefilter_hot_pixel_max_events_per_batch=9),
        cfg("pre_ba_only", prefilter_enable=True, prefilter_ba_enable=True, prefilter_ba_window_us=2500, prefilter_ba_min_neighbors=2),
        cfg("pre_ba_hot", prefilter_enable=True, prefilter_ba_enable=True, prefilter_ba_window_us=2500,
            prefilter_ba_min_neighbors=2, prefilter_hot_pixel_enable=True, prefilter_hot_pixel_max_events_per_batch=10),
        cfg("pre_ref_hot", prefilter_enable=True, prefilter_refractory_enable=True, prefilter_refractory_us=1200,
            prefilter_hot_pixel_enable=True, prefilter_hot_pixel_max_events_per_batch=10),
        cfg("post_ref_300us", post_refractory_enable=True, post_refractory_us=300),
        cfg("post_hot_only", post_hot_pixel_enable=True, post_hot_pixel_max_events_per_pixel=10),
        cfg("post_density_mild", postfilter_enable=True, postfilter_min_count_per_pixel=1,
            postfilter_min_count_in_neighborhood=3, postfilter_min_component_pixels=3, postfilter_adaptive_enable=True),
        cfg("post_density_strong", postfilter_enable=True, postfilter_min_count_per_pixel=2,
            postfilter_min_count_in_neighborhood=8, postfilter_min_component_pixels=4, postfilter_adaptive_enable=False),
        cfg("pre_ref_post_ref", prefilter_enable=True, prefilter_refractory_enable=True, prefilter_refractory_us=1200,
            post_refractory_enable=True, post_refractory_us=300),
        cfg("balanced_1", prefilter_enable=True, prefilter_refractory_enable=True, prefilter_refractory_us=1000,
            prefilter_hot_pixel_enable=True, prefilter_hot_pixel_max_events_per_batch=10, postfilter_enable=True,
            postfilter_min_count_per_pixel=1, postfilter_min_count_in_neighborhood=4,
            postfilter_min_component_pixels=3, postfilter_adaptive_enable=True),
        cfg("balanced_2", prefilter_enable=True, prefilter_ba_enable=True, prefilter_ba_window_us=2200,
            prefilter_ba_min_neighbors=1, prefilter_hot_pixel_enable=True, prefilter_hot_pixel_max_events_per_batch=10,
            postfilter_enable=True, postfilter_min_count_per_pixel=1, postfilter_min_count_in_neighborhood=3,
            postfilter_min_component_pixels=3, postfilter_adaptive_enable=True),
    ]


def print_table(results: List[ComboResult]) -> None:
    print("\n=== Sweep results (motion-active batches prioritized) ===")
    print(
        "name, viable, noise, fg_ratio, fg_events, keep_events, fg_preserve, evt_preserve, "
        "iso_post, hot_post, small_cc, motion_samples"
    )
    for r in results:
        keep_ratio = (r.avg_post_events / r.avg_pre_events) if r.avg_pre_events > 0.0 else 0.0
        print(
            f"{r.name}, {r.viable}, {r.avg_noise:.4f}, {r.avg_fg_ratio:.3f}, "
            f"{r.avg_fg_events:.1f}, {keep_ratio:.3f}, {r.fg_preserve:.3f}, {r.evt_preserve:.3f}, "
            f"{r.avg_iso_post:.3f}, {r.avg_hot_post:.3f}, {r.avg_small_cc:.3f}, {r.n_motion}"
        )


def main() -> int:
    parser = argparse.ArgumentParser(description="Sweep datasync_3_0 filter combos using /event_noise_metrics.")
    parser.add_argument("--duration", type=float, default=10.0, help="Measurement duration per combo (s).")
    parser.add_argument("--warmup", type=float, default=2.0, help="Warmup per combo before measurement (s).")
    parser.add_argument("--min-motion-samples", type=int, default=40, help="Minimum motion-active samples to trust a combo.")
    parser.add_argument("--min-fg-preserve", type=float, default=0.35,
                        help="Minimum preserved segmentation-passing events vs all_off baseline.")
    parser.add_argument("--min-evt-preserve", type=float, default=0.20,
                        help="Minimum preserved compensated events vs all_off baseline.")
    args = parser.parse_args()

    if args.duration <= 0.0 or args.warmup < 0.0:
        print("Invalid duration/warmup.", file=sys.stderr)
        return 2

    rclpy.init()
    collector = MetricsCollector("/event_noise_metrics")

    all_results: List[ComboResult] = []
    try:
        for idx, (name, params) in enumerate(combos(), start=1):
            print(f"[{idx}/{len(combos())}] running {name} ...", flush=True)
            result = run_combo(collector, name, params, args.duration, args.warmup)
            print(
                f"  samples={result.n_total} motion={result.n_motion} "
                f"noise={result.avg_noise:.4f} fg_ratio={result.avg_fg_ratio:.3f}",
                flush=True,
            )
            all_results.append(result)
    finally:
        collector.destroy_node()
        rclpy.shutdown()

    baseline = next((r for r in all_results if r.name == "all_off"), None)
    base_fg = baseline.avg_fg_events if baseline and baseline.avg_fg_events > 0.0 else 1.0
    base_evt = baseline.avg_post_events if baseline and baseline.avg_post_events > 0.0 else 1.0

    for r in all_results:
        r.fg_preserve = r.avg_fg_events / base_fg if base_fg > 0.0 else 0.0
        r.evt_preserve = r.avg_post_events / base_evt if base_evt > 0.0 else 0.0
        r.viable = (
            r.n_motion >= args.min_motion_samples
            and r.fg_preserve >= args.min_fg_preserve
            and r.evt_preserve >= args.min_evt_preserve
        )

    def rank_key(r: ComboResult) -> Tuple[float, float, float]:
        penalty = 0.0 if r.viable else 1.0
        return (penalty + r.avg_noise, -r.fg_preserve, -r.evt_preserve)

    ranked = sorted(all_results, key=rank_key)
    print_table(ranked)

    best = next((r for r in ranked if r.viable), ranked[0] if ranked else None)
    if best is None:
        print("No result available.")
        return 1

    print("\n=== Recommended combo ===")
    print(f"name: {best.name}")
    print(
        f"noise={best.avg_noise:.4f}, fg_preserve={best.fg_preserve:.3f}, "
        f"evt_preserve={best.evt_preserve:.3f}, motion_samples={best.n_motion}"
    )
    print("launch overrides:")
    for k, v in best.params.items():
        if k in ("metrics_enable", "metrics_log_stats", "noise_metrics_topic"):
            continue
        print(f"  {k}:={v}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
