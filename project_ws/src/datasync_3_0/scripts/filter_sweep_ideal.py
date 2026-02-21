#!/usr/bin/env python3
import argparse
import csv
import os
import random
import signal
import statistics
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from scipy import ndimage
from sensor_msgs.msg import Image
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


@dataclass
class Candidate:
    name: str
    ds_params: Dict[str, str]
    seg_params: Dict[str, str]


@dataclass
class EvalResult:
    candidate: Candidate
    n_total: int
    n_motion: int
    noise_mean: float
    iso_mean: float
    hot_mean: float
    small_cc_mean: float
    fg_event_ratio_mean: float
    fg_events_mean: float
    pre_events_mean: float
    post_events_mean: float
    event_keep_ratio: float
    lambda_mean: float
    omega_mean: float
    mask_fg_mean: float
    mask_lcc_ratio_mean: float
    mask_components_mean: float
    mask_presence_ratio: float
    score: float = 999.0
    viable: bool = False


def fmean(values: Iterable[float]) -> float:
    vals = list(values)
    if not vals:
        return 0.0
    return float(statistics.fmean(vals))


class CombinedCollector(Node):
    def __init__(self, metrics_topic: str, mask_topic: str) -> None:
        super().__init__("filter_sweep_ideal_collector")
        self.samples: List[Dict[str, float]] = []
        self._latest_mask_fg = 0.0
        self._latest_mask_lcc_ratio = 0.0
        self._latest_mask_components = 0.0
        self._latest_mask_mono_time = 0.0

        self.create_subscription(Float32MultiArray, metrics_topic, self._on_metrics, 200)
        self.create_subscription(Image, mask_topic, self._on_mask, 50)

    def reset(self) -> None:
        self.samples.clear()

    def _on_mask(self, msg: Image) -> None:
        if msg.encoding not in ("mono8", "8UC1"):
            return
        if msg.width <= 0 or msg.height <= 0 or msg.step <= 0:
            return
        arr = np.frombuffer(msg.data, dtype=np.uint8)
        if arr.size < msg.step * msg.height:
            return
        arr = arr.reshape((msg.height, msg.step))[:, : msg.width]
        fg = arr > 0
        fg_pixels = int(np.count_nonzero(fg))

        lcc_ratio = 0.0
        components = 0
        if fg_pixels > 0:
            labels, components = ndimage.label(fg)
            if components > 0:
                counts = np.bincount(labels.ravel())
                if counts.size > 1:
                    lcc = int(np.max(counts[1:]))
                    lcc_ratio = float(lcc) / float(fg_pixels)

        self._latest_mask_fg = float(fg_pixels)
        self._latest_mask_lcc_ratio = float(lcc_ratio)
        self._latest_mask_components = float(components)
        self._latest_mask_mono_time = time.monotonic()

    def _on_metrics(self, msg: Float32MultiArray) -> None:
        if len(msg.data) < len(METRIC_FIELDS):
            return

        sample = {k: float(msg.data[i]) for i, k in enumerate(METRIC_FIELDS)}
        mask_age = time.monotonic() - self._latest_mask_mono_time
        if mask_age <= 0.25:
            sample["mask_fg_pixels"] = self._latest_mask_fg
            sample["mask_lcc_ratio"] = self._latest_mask_lcc_ratio
            sample["mask_components"] = self._latest_mask_components
        else:
            sample["mask_fg_pixels"] = 0.0
            sample["mask_lcc_ratio"] = 0.0
            sample["mask_components"] = 0.0

        self.samples.append(sample)


def stop_process_tree(proc: subprocess.Popen) -> None:
    if proc.poll() is not None:
        return
    try:
        os.killpg(proc.pid, signal.SIGINT)
        proc.wait(timeout=4.0)
        return
    except Exception:
        pass
    try:
        os.killpg(proc.pid, signal.SIGTERM)
        proc.wait(timeout=2.0)
        return
    except Exception:
        pass
    try:
        os.killpg(proc.pid, signal.SIGKILL)
    except Exception:
        pass


def _param_args(params: Dict[str, str]) -> List[str]:
    return [f"{k}:={v}" for k, v in params.items()]


def run_candidate(
    collector: CombinedCollector,
    cand: Candidate,
    params_file: str,
    warmup_s: float,
    duration_s: float,
) -> EvalResult:
    ds_params = dict(cand.ds_params)
    ds_params["params_file"] = params_file
    ds_params["metrics_enable"] = "true"
    ds_params["metrics_log_stats"] = "false"
    ds_params["noise_metrics_topic"] = "/event_noise_metrics"

    ds_cmd = [
        "ros2",
        "launch",
        "datasync_3_0",
        "motion_compensation.launch.py",
        *_param_args(ds_params),
    ]
    seg_cmd = [
        "ros2",
        "run",
        "event_segmentation",
        "event_segmentation_node",
        "--ros-args",
        "-p",
        f"log_stats:={cand.seg_params.get('log_stats', 'false')}",
        "-p",
        f"min_count:={cand.seg_params.get('min_count', '1')}",
        "-p",
        f"morph_open:={cand.seg_params.get('morph_open', 'false')}",
        "-p",
        f"morph_close:={cand.seg_params.get('morph_close', 'false')}",
        "-p",
        f"morph_kernel:={cand.seg_params.get('morph_kernel', '3')}",
        "-p",
        f"cc_filter_enable:={cand.seg_params.get('cc_filter_enable', 'true')}",
        "-p",
        f"cc_min_area:={cand.seg_params.get('cc_min_area', '18')}",
        "-p",
        f"cc_max_area:={cand.seg_params.get('cc_max_area', '6000')}",
        "-p",
        f"cc_top_k:={cand.seg_params.get('cc_top_k', '1')}",
        "-p",
        f"cc_min_mean_abs_time:={cand.seg_params.get('cc_min_mean_abs_time', '0.025')}",
        "-p",
        f"cc_min_mean_count:={cand.seg_params.get('cc_min_mean_count', '1.5')}",
        "-p",
        f"temporal_filter_enable:={cand.seg_params.get('temporal_filter_enable', 'true')}",
        "-p",
        f"temporal_alpha:={cand.seg_params.get('temporal_alpha', '0.85')}",
        "-p",
        f"temporal_threshold:={cand.seg_params.get('temporal_threshold', '0.45')}",
        "-p",
        f"temporal_boost_current:={cand.seg_params.get('temporal_boost_current', 'true')}",
    ]

    ds_proc = subprocess.Popen(
        ds_cmd,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.STDOUT,
        preexec_fn=os.setsid,
    )
    seg_proc = subprocess.Popen(
        seg_cmd,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.STDOUT,
        preexec_fn=os.setsid,
    )

    try:
        t_end = time.monotonic() + warmup_s
        while time.monotonic() < t_end:
            rclpy.spin_once(collector, timeout_sec=0.1)

        collector.reset()
        t_end = time.monotonic() + duration_s
        while time.monotonic() < t_end:
            rclpy.spin_once(collector, timeout_sec=0.1)
    finally:
        stop_process_tree(seg_proc)
        stop_process_tree(ds_proc)

    samples = list(collector.samples)
    motion_samples = [s for s in samples if s.get("motion_active", 0.0) > 0.5]
    used = motion_samples if motion_samples else samples

    pre_mean = fmean(s.get("pre_events", 0.0) for s in used)
    post_mean = fmean(s.get("post_events", 0.0) for s in used)
    keep_ratio = post_mean / pre_mean if pre_mean > 1e-6 else 0.0

    mask_presence = fmean(1.0 if s.get("mask_fg_pixels", 0.0) >= 30.0 else 0.0 for s in used)

    return EvalResult(
        candidate=cand,
        n_total=len(samples),
        n_motion=len(motion_samples),
        noise_mean=fmean(s.get("noise_score", 0.0) for s in used),
        iso_mean=fmean(s.get("iso_post", 0.0) for s in used),
        hot_mean=fmean(s.get("hot_post", 0.0) for s in used),
        small_cc_mean=fmean(s.get("small_cc_post", 0.0) for s in used),
        fg_event_ratio_mean=fmean(s.get("fg_event_ratio", 0.0) for s in used),
        fg_events_mean=fmean(s.get("fg_events", 0.0) for s in used),
        pre_events_mean=pre_mean,
        post_events_mean=post_mean,
        event_keep_ratio=keep_ratio,
        lambda_mean=fmean(s.get("lambda", 0.0) for s in used),
        omega_mean=fmean(s.get("omega_norm", 0.0) for s in used),
        mask_fg_mean=fmean(s.get("mask_fg_pixels", 0.0) for s in used),
        mask_lcc_ratio_mean=fmean(s.get("mask_lcc_ratio", 0.0) for s in used),
        mask_components_mean=fmean(s.get("mask_components", 0.0) for s in used),
        mask_presence_ratio=mask_presence,
    )


def compute_score(res: EvalResult, base: EvalResult, min_motion_samples: int) -> Tuple[float, bool]:
    target_fg_ratio = max(0.07, 0.65 * base.fg_event_ratio_mean)
    target_fg_events = max(50.0, 0.55 * base.fg_events_mean)
    target_keep = max(0.22, 0.50 * base.event_keep_ratio)
    target_lcc = max(0.18, 1.10 * base.mask_lcc_ratio_mean)
    target_presence = max(0.35, 0.90 * base.mask_presence_ratio)

    penalty = 0.0
    if res.n_motion < min_motion_samples:
        penalty += 1.5

    if res.fg_event_ratio_mean < target_fg_ratio:
        penalty += (target_fg_ratio - res.fg_event_ratio_mean) * 4.0
    if res.fg_events_mean < target_fg_events:
        penalty += (target_fg_events - res.fg_events_mean) / max(target_fg_events, 1.0)
    if res.event_keep_ratio < target_keep:
        penalty += (target_keep - res.event_keep_ratio) * 2.0
    if res.mask_lcc_ratio_mean < target_lcc:
        penalty += (target_lcc - res.mask_lcc_ratio_mean) * 2.2
    if res.mask_presence_ratio < target_presence:
        penalty += (target_presence - res.mask_presence_ratio) * 2.5

    # Penalize too large mask area (flooded by noise) relative to baseline
    max_mask_fg = max(1200.0, 1.7 * base.mask_fg_mean)
    if res.mask_fg_mean > max_mask_fg:
        penalty += (res.mask_fg_mean - max_mask_fg) / max(max_mask_fg, 1.0)

    score = res.noise_mean + penalty

    viable = (
        res.n_motion >= min_motion_samples
        and res.fg_event_ratio_mean >= target_fg_ratio
        and res.fg_events_mean >= target_fg_events
        and res.event_keep_ratio >= target_keep
        and res.mask_lcc_ratio_mean >= target_lcc
        and res.mask_presence_ratio >= target_presence
    )
    return score, viable


def base_ds_params() -> Dict[str, str]:
    return {
        "events_topic": "events",
        "imu_topic": "imu",
        "count_image_topic": "count_image",
        "metrics_enable": "true",
        "metrics_log_stats": "false",
        "noise_metrics_topic": "/event_noise_metrics",
        "metrics_motion_omega_min": "0.10",
        "prefilter_enable": "false",
        "prefilter_refractory_enable": "false",
        "prefilter_refractory_us": "800",
        "prefilter_ba_enable": "false",
        "prefilter_ba_radius_px": "1",
        "prefilter_ba_window_us": "2200",
        "prefilter_ba_min_neighbors": "1",
        "prefilter_ba_same_polarity_only": "false",
        "prefilter_ba_support_from_kept_only": "false",
        "prefilter_hot_pixel_enable": "false",
        "prefilter_hot_pixel_max_events_per_batch": "10",
        "postfilter_enable": "false",
        "post_refractory_enable": "false",
        "post_refractory_us": "250",
        "post_hot_pixel_enable": "false",
        "post_hot_pixel_max_events_per_pixel": "10",
        "postfilter_min_count_per_pixel": "1",
        "postfilter_neighbor_radius_px": "1",
        "postfilter_min_count_in_neighborhood": "3",
        "postfilter_min_component_pixels": "3",
        "postfilter_adaptive_enable": "true",
        "postfilter_adaptive_min_keep_pixels": "8",
        "postfilter_adaptive_min_keep_ratio": "0.03",
        "postfilter_apply_to_compensated_events": "true",
        "lambda_a": "0.70",
        "lambda_b": "0.10",
    }


def base_seg_params() -> Dict[str, str]:
    return {
        "log_stats": "false",
        "min_count": "3",
        "morph_open": "false",
        "morph_close": "true",
        "morph_kernel": "5",
        "cc_filter_enable": "true",
        "cc_min_area": "18",
        "cc_max_area": "6000",
        "cc_top_k": "1",
        "cc_min_mean_abs_time": "0.025",
        "cc_min_mean_count": "1.5",
        "temporal_filter_enable": "false",
        "temporal_alpha": "0.85",
        "temporal_threshold": "0.45",
        "temporal_boost_current": "true",
    }


def make_candidate(name: str, ds_updates: Dict[str, str], seg_updates: Optional[Dict[str, str]] = None) -> Candidate:
    ds = base_ds_params()
    ds.update({k: str(v).lower() if isinstance(v, bool) else str(v) for k, v in ds_updates.items()})
    seg = base_seg_params()
    if seg_updates:
        seg.update({k: str(v).lower() if isinstance(v, bool) else str(v) for k, v in seg_updates.items()})
    return Candidate(name=name, ds_params=ds, seg_params=seg)


def curated_candidates() -> List[Candidate]:
    return [
        make_candidate("all_off", {}),
        make_candidate("current_pref_ref_hot", {
            "prefilter_enable": True,
            "prefilter_refractory_enable": True,
            "prefilter_refractory_us": 1200,
            "prefilter_hot_pixel_enable": True,
            "prefilter_hot_pixel_max_events_per_batch": 10,
        }),
        make_candidate("pref_ref_900", {
            "prefilter_enable": True,
            "prefilter_refractory_enable": True,
            "prefilter_refractory_us": 900,
        }),
        make_candidate("pref_ref_1500", {
            "prefilter_enable": True,
            "prefilter_refractory_enable": True,
            "prefilter_refractory_us": 1500,
        }),
        make_candidate("pref_hot_9", {
            "prefilter_enable": True,
            "prefilter_hot_pixel_enable": True,
            "prefilter_hot_pixel_max_events_per_batch": 9,
        }),
        make_candidate("pref_ba_mild", {
            "prefilter_enable": True,
            "prefilter_ba_enable": True,
            "prefilter_ba_window_us": 2200,
            "prefilter_ba_min_neighbors": 1,
        }),
        make_candidate("pref_ba_strict", {
            "prefilter_enable": True,
            "prefilter_ba_enable": True,
            "prefilter_ba_window_us": 3200,
            "prefilter_ba_min_neighbors": 2,
            "prefilter_ba_support_from_kept_only": True,
        }),
        make_candidate("post_density_mild", {
            "postfilter_enable": True,
            "postfilter_min_count_per_pixel": 1,
            "postfilter_min_count_in_neighborhood": 4,
            "postfilter_min_component_pixels": 3,
            "postfilter_adaptive_enable": True,
        }),
        make_candidate("post_density_strong", {
            "postfilter_enable": True,
            "postfilter_min_count_per_pixel": 2,
            "postfilter_min_count_in_neighborhood": 8,
            "postfilter_min_component_pixels": 4,
            "postfilter_adaptive_enable": False,
        }),
        make_candidate("post_ref_250", {
            "post_refractory_enable": True,
            "post_refractory_us": 250,
        }),
        make_candidate("post_hot_10", {
            "post_hot_pixel_enable": True,
            "post_hot_pixel_max_events_per_pixel": 10,
        }),
        make_candidate("hybrid_1", {
            "prefilter_enable": True,
            "prefilter_refractory_enable": True,
            "prefilter_refractory_us": 900,
            "prefilter_hot_pixel_enable": True,
            "prefilter_hot_pixel_max_events_per_batch": 10,
            "postfilter_enable": True,
            "postfilter_min_count_per_pixel": 1,
            "postfilter_min_count_in_neighborhood": 4,
            "postfilter_min_component_pixels": 3,
            "postfilter_adaptive_enable": True,
            "lambda_a": 0.65,
            "lambda_b": 0.08,
        }),
        make_candidate("hybrid_2", {
            "prefilter_enable": True,
            "prefilter_ba_enable": True,
            "prefilter_ba_window_us": 2500,
            "prefilter_ba_min_neighbors": 1,
            "prefilter_hot_pixel_enable": True,
            "prefilter_hot_pixel_max_events_per_batch": 10,
            "post_refractory_enable": True,
            "post_refractory_us": 200,
            "lambda_a": 0.85,
            "lambda_b": 0.15,
        }, {"min_count": 2}),
    ]


def random_candidate(idx: int, rng: random.Random) -> Candidate:
    ds = base_ds_params()

    ds["lambda_a"] = f"{rng.uniform(0.45, 1.25):.3f}"
    ds["lambda_b"] = f"{rng.uniform(0.03, 0.35):.3f}"

    pre_enable = rng.random() < 0.80
    post_density_enable = rng.random() < 0.40
    post_ref_enable = rng.random() < 0.35
    post_hot_enable = rng.random() < 0.25

    ds["prefilter_enable"] = str(pre_enable).lower()
    if pre_enable:
        pre_ref = rng.random() < 0.70
        pre_ba = rng.random() < 0.45
        pre_hot = rng.random() < 0.55

        ds["prefilter_refractory_enable"] = str(pre_ref).lower()
        ds["prefilter_refractory_us"] = str(rng.choice([300, 600, 900, 1200, 1500, 2000, 2600]))
        ds["prefilter_ba_enable"] = str(pre_ba).lower()
        ds["prefilter_ba_radius_px"] = str(rng.choice([1, 1, 2]))
        ds["prefilter_ba_window_us"] = str(rng.choice([1500, 2200, 3000, 4000]))
        ds["prefilter_ba_min_neighbors"] = str(rng.choice([1, 1, 2, 3]))
        ds["prefilter_ba_same_polarity_only"] = str(rng.random() < 0.35).lower()
        ds["prefilter_ba_support_from_kept_only"] = str(rng.random() < 0.25).lower()
        ds["prefilter_hot_pixel_enable"] = str(pre_hot).lower()
        ds["prefilter_hot_pixel_max_events_per_batch"] = str(rng.choice([8, 9, 10, 11, 12, 14]))
    else:
        ds["prefilter_refractory_enable"] = "false"
        ds["prefilter_ba_enable"] = "false"
        ds["prefilter_hot_pixel_enable"] = "false"

    ds["postfilter_enable"] = str(post_density_enable).lower()
    ds["post_refractory_enable"] = str(post_ref_enable).lower()
    ds["post_refractory_us"] = str(rng.choice([120, 180, 250, 300, 450, 700]))
    ds["post_hot_pixel_enable"] = str(post_hot_enable).lower()
    ds["post_hot_pixel_max_events_per_pixel"] = str(rng.choice([8, 9, 10, 11, 12]))
    ds["postfilter_min_count_per_pixel"] = str(rng.choice([1, 1, 2, 3]))
    ds["postfilter_min_count_in_neighborhood"] = str(rng.choice([3, 4, 5, 6, 8, 10]))
    ds["postfilter_min_component_pixels"] = str(rng.choice([2, 3, 4, 5]))
    ds["postfilter_adaptive_enable"] = str(rng.random() < 0.65).lower()
    ds["postfilter_adaptive_min_keep_pixels"] = str(rng.choice([6, 8, 10, 12]))
    ds["postfilter_adaptive_min_keep_ratio"] = f"{rng.choice([0.02, 0.03, 0.04, 0.05]):.2f}"

    if not pre_enable and not post_density_enable and not post_ref_enable and not post_hot_enable:
        ds["prefilter_enable"] = "true"
        ds["prefilter_hot_pixel_enable"] = "true"
        ds["prefilter_hot_pixel_max_events_per_batch"] = "10"

    seg = base_seg_params()
    seg["min_count"] = str(rng.choice([1, 1, 1, 2, 3]))
    seg["morph_open"] = str(rng.random() < 0.20).lower()
    seg["morph_close"] = str(rng.random() < 0.20).lower()
    seg["morph_kernel"] = str(rng.choice([3, 3, 5]))

    return Candidate(name=f"rand_{idx:03d}", ds_params=ds, seg_params=seg)


def mutate_around(best: Candidate, rng: random.Random, count: int) -> List[Candidate]:
    out: List[Candidate] = []
    for i in range(count):
        ds = dict(best.ds_params)
        seg = dict(best.seg_params)

        if rng.random() < 0.6:
            ds["prefilter_refractory_us"] = str(max(200, min(3000, int(float(ds["prefilter_refractory_us"])) + rng.choice([-400, -250, -120, 120, 250, 400]))))
        if rng.random() < 0.5:
            ds["prefilter_hot_pixel_max_events_per_batch"] = str(max(6, min(15, int(float(ds["prefilter_hot_pixel_max_events_per_batch"])) + rng.choice([-2, -1, 1, 2]))))
        if rng.random() < 0.4:
            ds["prefilter_ba_window_us"] = str(max(1000, min(5000, int(float(ds["prefilter_ba_window_us"])) + rng.choice([-600, -300, 300, 600]))))
        if rng.random() < 0.4:
            ds["post_refractory_us"] = str(max(80, min(1000, int(float(ds["post_refractory_us"])) + rng.choice([-120, -80, 80, 120]))))
        if rng.random() < 0.5:
            ds["postfilter_min_count_in_neighborhood"] = str(max(2, min(12, int(float(ds["postfilter_min_count_in_neighborhood"])) + rng.choice([-2, -1, 1, 2]))))
        if rng.random() < 0.5:
            ds["lambda_a"] = f"{max(0.35, min(1.40, float(ds['lambda_a']) + rng.choice([-0.12, -0.06, 0.06, 0.12]))):.3f}"
        if rng.random() < 0.5:
            ds["lambda_b"] = f"{max(0.01, min(0.45, float(ds['lambda_b']) + rng.choice([-0.05, -0.02, 0.02, 0.05]))):.3f}"

        for toggle in [
            "prefilter_enable",
            "prefilter_refractory_enable",
            "prefilter_ba_enable",
            "prefilter_hot_pixel_enable",
            "postfilter_enable",
            "post_refractory_enable",
            "post_hot_pixel_enable",
            "postfilter_adaptive_enable",
        ]:
            if rng.random() < 0.12:
                ds[toggle] = "false" if ds[toggle] == "true" else "true"

        if rng.random() < 0.35:
            seg["min_count"] = str(max(1, min(4, int(seg["min_count"]) + rng.choice([-1, 1]))))

        out.append(Candidate(name=f"mut_{i:03d}_{best.name}", ds_params=ds, seg_params=seg))
    return out


def dedupe_candidates(cands: List[Candidate]) -> List[Candidate]:
    seen = set()
    unique: List[Candidate] = []
    for c in cands:
        key = tuple(sorted(c.ds_params.items())) + tuple(sorted(c.seg_params.items()))
        if key in seen:
            continue
        seen.add(key)
        unique.append(c)
    return unique


def write_csv(path: Path, results: List[EvalResult]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow([
            "name",
            "viable",
            "score",
            "n_total",
            "n_motion",
            "noise_mean",
            "fg_event_ratio_mean",
            "fg_events_mean",
            "event_keep_ratio",
            "mask_fg_mean",
            "mask_lcc_ratio_mean",
            "mask_presence_ratio",
            "lambda_mean",
            "omega_mean",
            "ds_params",
            "seg_params",
        ])
        for r in results:
            writer.writerow([
                r.candidate.name,
                r.viable,
                f"{r.score:.6f}",
                r.n_total,
                r.n_motion,
                f"{r.noise_mean:.6f}",
                f"{r.fg_event_ratio_mean:.6f}",
                f"{r.fg_events_mean:.6f}",
                f"{r.event_keep_ratio:.6f}",
                f"{r.mask_fg_mean:.6f}",
                f"{r.mask_lcc_ratio_mean:.6f}",
                f"{r.mask_presence_ratio:.6f}",
                f"{r.lambda_mean:.6f}",
                f"{r.omega_mean:.6f}",
                " ".join(f"{k}={v}" for k, v in sorted(r.candidate.ds_params.items())),
                " ".join(f"{k}={v}" for k, v in sorted(r.candidate.seg_params.items())),
            ])


def rank_results(results: List[EvalResult], base: EvalResult, min_motion_samples: int) -> List[EvalResult]:
    for r in results:
        r.score, r.viable = compute_score(r, base, min_motion_samples)
    return sorted(results, key=lambda x: (0 if x.viable else 1, x.score))


def main() -> int:
    parser = argparse.ArgumentParser(description="Long multi-phase filter optimization with segmentation quality.")
    parser.add_argument("--params-file", default="", help="datasync params yaml path (default: share config/motion_compensation.yaml)")
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--random-trials", type=int, default=36)
    parser.add_argument("--broad-warmup", type=float, default=2.5)
    parser.add_argument("--broad-duration", type=float, default=8.0)
    parser.add_argument("--refine-top", type=int, default=10)
    parser.add_argument("--refine-warmup", type=float, default=3.0)
    parser.add_argument("--refine-duration", type=float, default=22.0)
    parser.add_argument("--final-top", type=int, default=3)
    parser.add_argument("--final-warmup", type=float, default=4.0)
    parser.add_argument("--final-duration", type=float, default=55.0)
    parser.add_argument("--baseline-warmup", type=float, default=3.0)
    parser.add_argument("--baseline-duration", type=float, default=28.0)
    parser.add_argument("--min-motion-samples", type=int, default=90)
    parser.add_argument("--out-csv", default="/tmp/filter_sweep_ideal_results.csv")
    args = parser.parse_args()

    params_file = args.params_file.strip()
    if not params_file:
        params_file = str(Path.home() / "NOCHI/M2_PAR/Projet_de_synthese/project_ws/src/datasync_3_0/config/motion_compensation.yaml")

    if not Path(params_file).exists():
        print(f"Params file not found: {params_file}", file=sys.stderr)
        return 2

    rng = random.Random(args.seed)
    rclpy.init()
    collector = CombinedCollector(metrics_topic="/event_noise_metrics", mask_topic="/event_mask")

    try:
        base_cand = make_candidate("all_off_baseline", {})
        print("[baseline] running all_off baseline ...", flush=True)
        baseline = run_candidate(
            collector,
            base_cand,
            params_file=params_file,
            warmup_s=args.baseline_warmup,
            duration_s=args.baseline_duration,
        )
        baseline.score = baseline.noise_mean
        baseline.viable = True
        print(
            f"[baseline] motion={baseline.n_motion} noise={baseline.noise_mean:.4f} "
            f"fg_ratio={baseline.fg_event_ratio_mean:.3f} fg_events={baseline.fg_events_mean:.1f} "
            f"mask_lcc={baseline.mask_lcc_ratio_mean:.3f}",
            flush=True,
        )

        broad: List[Candidate] = curated_candidates()
        broad.extend(random_candidate(i + 1, rng) for i in range(args.random_trials))
        broad = dedupe_candidates(broad)

        broad_results: List[EvalResult] = []
        for i, cand in enumerate(broad, start=1):
            print(f"[broad {i}/{len(broad)}] {cand.name}", flush=True)
            res = run_candidate(
                collector,
                cand,
                params_file=params_file,
                warmup_s=args.broad_warmup,
                duration_s=args.broad_duration,
            )
            broad_results.append(res)
            print(
                f"  motion={res.n_motion} noise={res.noise_mean:.4f} fg_ratio={res.fg_event_ratio_mean:.3f} "
                f"fg_evt={res.fg_events_mean:.1f} lcc={res.mask_lcc_ratio_mean:.3f} mask_presence={res.mask_presence_ratio:.3f}",
                flush=True,
            )

        ranked_broad = rank_results(broad_results, baseline, args.min_motion_samples)
        refine_seed = ranked_broad[: max(1, args.refine_top)]

        refined_candidates = [r.candidate for r in refine_seed]
        if refine_seed:
            refined_candidates.extend(mutate_around(refine_seed[0].candidate, rng, max(8, args.refine_top)))
        refined_candidates = dedupe_candidates(refined_candidates)

        refine_results: List[EvalResult] = []
        for i, cand in enumerate(refined_candidates, start=1):
            print(f"[refine {i}/{len(refined_candidates)}] {cand.name}", flush=True)
            res = run_candidate(
                collector,
                cand,
                params_file=params_file,
                warmup_s=args.refine_warmup,
                duration_s=args.refine_duration,
            )
            refine_results.append(res)
            print(
                f"  motion={res.n_motion} noise={res.noise_mean:.4f} fg_ratio={res.fg_event_ratio_mean:.3f} "
                f"fg_evt={res.fg_events_mean:.1f} lcc={res.mask_lcc_ratio_mean:.3f} mask_presence={res.mask_presence_ratio:.3f}",
                flush=True,
            )

        ranked_refine = rank_results(refine_results, baseline, args.min_motion_samples)
        finalists = [r.candidate for r in ranked_refine[: max(1, args.final_top)]]

        final_results: List[EvalResult] = []
        for i, cand in enumerate(finalists, start=1):
            print(f"[final {i}/{len(finalists)}] {cand.name}", flush=True)
            res = run_candidate(
                collector,
                cand,
                params_file=params_file,
                warmup_s=args.final_warmup,
                duration_s=args.final_duration,
            )
            final_results.append(res)
            print(
                f"  motion={res.n_motion} noise={res.noise_mean:.4f} fg_ratio={res.fg_event_ratio_mean:.3f} "
                f"fg_evt={res.fg_events_mean:.1f} lcc={res.mask_lcc_ratio_mean:.3f} mask_presence={res.mask_presence_ratio:.3f}",
                flush=True,
            )

        ranked_final = rank_results(final_results, baseline, args.min_motion_samples)
        best = ranked_final[0]

        merged_all = [baseline] + rank_results(broad_results + refine_results + final_results, baseline, args.min_motion_samples)
        write_csv(Path(args.out_csv), merged_all)

        print("\n=== FINAL RANKING (top 8) ===")
        for r in ranked_final[:8]:
            keep = r.event_keep_ratio
            print(
                f"{r.candidate.name}: viable={r.viable} score={r.score:.4f} noise={r.noise_mean:.4f} "
                f"fg_ratio={r.fg_event_ratio_mean:.3f} fg_evt={r.fg_events_mean:.1f} "
                f"keep={keep:.3f} lcc={r.mask_lcc_ratio_mean:.3f} mask_presence={r.mask_presence_ratio:.3f} motion={r.n_motion}",
                flush=True,
            )

        print("\n=== BEST CONFIG ===")
        print(f"name: {best.candidate.name}")
        print(
            f"score={best.score:.4f} noise={best.noise_mean:.4f} fg_ratio={best.fg_event_ratio_mean:.3f} "
            f"fg_evt={best.fg_events_mean:.1f} lcc={best.mask_lcc_ratio_mean:.3f} mask_presence={best.mask_presence_ratio:.3f}"
        )
        print("datasync launch overrides:")
        for k, v in sorted(best.candidate.ds_params.items()):
            if k in {"metrics_enable", "metrics_log_stats", "noise_metrics_topic"}:
                continue
            print(f"  {k}:={v}")
        print("segmentation overrides:")
        for k, v in sorted(best.candidate.seg_params.items()):
            print(f"  {k}:={v}")
        print(f"results_csv: {args.out_csv}")

    finally:
        collector.destroy_node()
        rclpy.shutdown()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
