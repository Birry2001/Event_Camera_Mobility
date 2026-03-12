#!/usr/bin/env python3
"""
Automate ROS 2 bag campaign testing for datasync_3_0 with parameter sweep on (a, b).

Main workflow for each (bag, a, b):
  1) Stage bag inside workspace src (copy or move).
  2) Launch pipeline:
       - datasync_3_0 motion_compensation
       - event_segmentation (optional)
       - event_clustering_2_0 (optional)
  3) Play bag once.
  4) Run metric extractor (M1..M6) during playback.
  5) Store logs + metrics under a structured run directory.
  6) Restore or cleanup staged bag.

Outputs:
  - One directory per run:
      <output_root>/<campaign_path>/<bag_name>/a_<...>_b_<...>/
  - A global manifest CSV collecting all runs:
      <output_root>/manifest.csv
"""

from __future__ import annotations

import argparse
import csv
import json
import os
import re
import shutil
import signal
import subprocess
import sys
import time
import uuid
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

try:
    import yaml  # type: ignore
except Exception:
    yaml = None


SUMMARY_KEYS = [
    "run_name",
    "timestamp_utc",
    "duration_s",
    "samples_m1",
    "samples_m2_m4",
    "samples_m2_density",
    "samples_m2_blur",
    "samples_m5",
    "samples_m6_k",
    "samples_m6_iou",
    "m1_comp_lat_mean_ms",
    "m1_comp_lat_median_ms",
    "m1_comp_lat_p95_ms",
    "m2_density_pre_mean",
    "m2_density_post_mean",
    "m2_density_gain_mean",
    "m2_blur_pre_mean",
    "m2_blur_post_mean",
    "m2_blur_reduction_mean",
    "m2_blur_reduction_percent_mean",
    "m2_keep_ratio_mean",
    "m2_warp_valid_ratio_mean",
    "m2_projection_loss_mean",
    "m3_iso_post_mean",
    "m3_hot_post_mean",
    "m3_small_cc_post_mean",
    "m4_fg_ratio_mean",
    "m5_seg_lat_mean_ms",
    "m5_seg_lat_median_ms",
    "m5_seg_lat_p95_ms",
    "m6_cluster_count_mean",
    "m6_cluster_count_var",
    "m6_cluster_count_std",
    "m6_mask_iou_mean",
    "m6_mask_iou_std",
]


def now_utc() -> str:
    return datetime.utcnow().isoformat(timespec="seconds")


def sanitize_token(token: str) -> str:
    token = token.strip()
    token = re.sub(r"[^A-Za-z0-9._-]+", "_", token)
    token = token.strip("._-")
    return token or "x"


def filesystem_safe_component(name: str) -> str:
    """Keep original naming as much as possible; only strip forbidden path separators."""
    out = name.replace("/", "_").replace("\0", "")
    out = out.strip()
    return out or "x"


def rel_or_name(path: Path, root: Path) -> Path:
    try:
        return path.resolve().relative_to(root.resolve())
    except Exception:
        return Path(path.name)


def parse_float_list(text: str) -> List[float]:
    vals: List[float] = []
    for chunk in text.split(","):
        c = chunk.strip()
        if not c:
            continue
        vals.append(float(c))
    return vals


def fmt_ab_dir(a: float, b: float) -> str:
    sa = f"{a:.3f}".replace("-", "m").replace(".", "p")
    sb = f"{b:.3f}".replace("-", "m").replace(".", "p")
    return f"a_{sa}_b_{sb}"


def unique_pairs(pairs: Iterable[Tuple[float, float]], ndigits: int = 6) -> List[Tuple[float, float]]:
    out: List[Tuple[float, float]] = []
    seen = set()
    for a, b in pairs:
        key = (round(float(a), ndigits), round(float(b), ndigits))
        if key in seen:
            continue
        seen.add(key)
        out.append((float(a), float(b)))
    return out


def build_ab_sweep(
    anchor_a: float,
    anchor_b: float,
    a_sweep: List[float],
    b_sweep: List[float],
    grid_a: List[float],
    grid_b: List[float],
) -> List[Tuple[float, float]]:
    pairs: List[Tuple[float, float]] = []
    pairs.append((anchor_a, anchor_b))
    for a in a_sweep:
        pairs.append((a, anchor_b))
    for b in b_sweep:
        pairs.append((anchor_a, b))
    for a in grid_a:
        for b in grid_b:
            pairs.append((a, b))
    return unique_pairs(pairs)


def find_bag_dirs(bags_root: Path, ignore_parts: Sequence[str]) -> List[Path]:
    bags: List[Path] = []
    for meta in bags_root.rglob("metadata.yaml"):
        bag_dir = meta.parent
        parts = set(bag_dir.parts)
        if any(part in parts for part in ignore_parts):
            continue
        bags.append(bag_dir)
    bags.sort(key=lambda p: str(p))
    return bags


def read_bag_duration_sec_from_metadata(bag_dir: Path) -> Optional[float]:
    meta = bag_dir / "metadata.yaml"
    if not meta.exists():
        return None

    if yaml is None:
        return None

    try:
        with meta.open("r", encoding="utf-8") as f:
            doc = yaml.safe_load(f)
    except Exception:
        return None

    if not isinstance(doc, dict):
        return None
    info = doc.get("rosbag2_bagfile_information", {})
    if not isinstance(info, dict):
        return None

    duration = info.get("duration")
    if isinstance(duration, dict):
        if "nanoseconds" in duration:
            try:
                return float(duration["nanoseconds"]) * 1e-9
            except Exception:
                pass
        sec = duration.get("sec")
        nsec = duration.get("nsec")
        if sec is not None and nsec is not None:
            try:
                return float(sec) + float(nsec) * 1e-9
            except Exception:
                pass
    elif isinstance(duration, (int, float)):
        # Usually nanoseconds in rosbag2 metadata.
        d = float(duration)
        if d > 1e6:
            return d * 1e-9
        return d

    return None


def read_bag_duration_sec_via_info(bag_dir: Path) -> Optional[float]:
    try:
        proc = subprocess.run(
            ["ros2", "bag", "info", str(bag_dir)],
            check=False,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
        )
    except Exception:
        return None

    txt = proc.stdout or ""
    m = re.search(r"Duration:\s*([0-9]+(?:\.[0-9]+)?)s", txt)
    if not m:
        return None
    try:
        return float(m.group(1))
    except Exception:
        return None


def read_bag_duration_sec(bag_dir: Path, fallback_sec: float) -> float:
    d = read_bag_duration_sec_from_metadata(bag_dir)
    if d is None:
        d = read_bag_duration_sec_via_info(bag_dir)
    if d is None or d <= 0.0:
        d = fallback_sec
    return max(0.1, float(d))


def stop_process_tree(proc: Optional[subprocess.Popen]) -> None:
    if proc is None:
        return
    if proc.poll() is not None:
        return
    try:
        os.killpg(proc.pid, signal.SIGINT)
        proc.wait(timeout=5.0)
        return
    except Exception:
        pass
    try:
        os.killpg(proc.pid, signal.SIGTERM)
        proc.wait(timeout=3.0)
        return
    except Exception:
        pass
    try:
        os.killpg(proc.pid, signal.SIGKILL)
    except Exception:
        pass


def wait_with_timeout(proc: subprocess.Popen, timeout_s: float) -> Optional[int]:
    try:
        return proc.wait(timeout=max(0.1, timeout_s))
    except subprocess.TimeoutExpired:
        return None


def start_logged_process(cmd: List[str], log_path: Path, cwd: Optional[Path] = None) -> subprocess.Popen:
    log_path.parent.mkdir(parents=True, exist_ok=True)
    f = log_path.open("w", encoding="utf-8")
    f.write(f"# start_utc: {now_utc()}\n")
    f.write("# cmd: " + " ".join(cmd) + "\n\n")
    f.flush()
    proc = subprocess.Popen(
        cmd,
        stdout=f,
        stderr=subprocess.STDOUT,
        preexec_fn=os.setsid,
        cwd=str(cwd) if cwd else None,
    )
    # Keep file descriptor attached to process object for later close.
    setattr(proc, "_log_file", f)
    return proc


def close_proc_log(proc: Optional[subprocess.Popen]) -> None:
    if proc is None:
        return
    f = getattr(proc, "_log_file", None)
    if f is not None:
        try:
            f.flush()
            f.close()
        except Exception:
            pass


def append_csv_row(path: Path, row: Dict[str, object]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    exists = path.exists()
    with path.open("a", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(row.keys()))
        if not exists:
            writer.writeheader()
        writer.writerow(row)


def load_single_row_csv(path: Path) -> Dict[str, str]:
    if not path.exists():
        return {}
    with path.open("r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        rows = list(reader)
    if not rows:
        return {}
    return rows[-1]


@dataclass
class StageHandle:
    original_bag: Path
    staged_bag: Path
    mode: str  # copy|move
    keep_copy: bool


def stage_bag(
    bag_dir: Path,
    stage_root: Path,
    mode: str,
    keep_copy: bool,
) -> StageHandle:
    stage_root.mkdir(parents=True, exist_ok=True)
    uid = datetime.utcnow().strftime("%Y%m%d_%H%M%S") + "_" + uuid.uuid4().hex[:8]
    staged_bag = stage_root / f"{sanitize_token(bag_dir.name)}__{uid}"

    if mode == "copy":
        shutil.copytree(bag_dir, staged_bag)
        return StageHandle(original_bag=bag_dir, staged_bag=staged_bag, mode=mode, keep_copy=keep_copy)

    if mode == "move":
        shutil.move(str(bag_dir), str(staged_bag))
        return StageHandle(original_bag=bag_dir, staged_bag=staged_bag, mode=mode, keep_copy=keep_copy)

    raise ValueError(f"Unsupported stage mode: {mode}")


def unstage_bag(stage: StageHandle) -> None:
    if stage.mode == "copy":
        if stage.keep_copy:
            return
        shutil.rmtree(stage.staged_bag, ignore_errors=True)
        return

    if stage.mode == "move":
        if stage.original_bag.exists():
            # Best-effort fallback if original path re-created unexpectedly.
            backup = stage.original_bag.parent / f"{stage.original_bag.name}__restored_{uuid.uuid4().hex[:6]}"
            shutil.move(str(stage.staged_bag), str(backup))
        else:
            shutil.move(str(stage.staged_bag), str(stage.original_bag))
        return


def require_ros_environment() -> None:
    if shutil.which("ros2") is None:
        raise RuntimeError(
            "ros2 command not found. Source your environment first:\n"
            "  source /opt/ros/humble/setup.bash\n"
            "  source <workspace>/install/setup.bash"
        )


def parse_args() -> argparse.Namespace:
    script_path = Path(__file__).resolve()
    workspace_src_default = script_path.parents[2]
    metrics_script_default = script_path.parent / "extract_pipeline_metrics.py"
    output_default = workspace_src_default.parent / "automation_results" / (
        "ab_sweep_" + datetime.utcnow().strftime("%Y%m%d_%H%M%S")
    )

    parser = argparse.ArgumentParser(
        description="Automate campaign bag tests for datasync_3_0 with (a,b) sweep."
    )
    parser.add_argument("--bags-root", required=True, help="Root directory containing bag folders.")
    parser.add_argument(
        "--workspace-src",
        default=str(workspace_src_default),
        help="Workspace src path where bags are staged before playback.",
    )
    parser.add_argument(
        "--output-root",
        default=str(output_default),
        help="Output root directory for run artifacts and manifest.",
    )
    parser.add_argument(
        "--stage-mode",
        choices=["copy", "move"],
        default="copy",
        help="How to stage bags in workspace src before playing.",
    )
    parser.add_argument(
        "--keep-staged-copy",
        action="store_true",
        help="If stage-mode=copy, keep staged bag copy after run.",
    )

    parser.add_argument("--anchor-a", type=float, default=0.5)
    parser.add_argument("--anchor-b", type=float, default=0.2)
    parser.add_argument(
        "--a-sweep",
        default="0.3,0.4,0.5,0.6,0.7",
        help="Values of a with b fixed at anchor_b.",
    )
    parser.add_argument(
        "--b-sweep",
        default="0.0,0.1,0.2,0.3,0.4",
        help="Values of b with a fixed at anchor_a.",
    )
    parser.add_argument(
        "--grid-a",
        default="0.4,0.5,0.6",
        help="Grid values for a (joint sweep with grid-b).",
    )
    parser.add_argument(
        "--grid-b",
        default="0.1,0.2,0.3",
        help="Grid values for b (joint sweep with grid-a).",
    )
    parser.add_argument(
        "--max-combos",
        type=int,
        default=0,
        help="Limit number of (a,b) combos (0 = no limit).",
    )
    parser.add_argument(
        "--bag-filter",
        default="",
        help="Regex on full bag path; only matching bags are tested.",
    )
    parser.add_argument(
        "--sanitize-output-names",
        action="store_true",
        help=(
            "Sanitize campaign/bag folder names in output directories. "
            "By default, original names are preserved."
        ),
    )

    parser.add_argument("--play-rate", type=float, default=1.0)
    parser.add_argument(
        "--fallback-bag-duration-s",
        type=float,
        default=30.0,
        help="Used only if bag duration cannot be parsed.",
    )
    parser.add_argument("--startup-s", type=float, default=3.0, help="Delay after launches before metrics/bag.")
    parser.add_argument("--tail-s", type=float, default=2.0, help="Extra acquisition time after expected bag duration.")
    parser.add_argument(
        "--bag-timeout-pad-s",
        type=float,
        default=20.0,
        help="Extra timeout margin for bag play process.",
    )
    parser.add_argument(
        "--metrics-timeout-pad-s",
        type=float,
        default=15.0,
        help="Extra timeout margin for metrics extractor.",
    )

    parser.add_argument(
        "--metrics-script",
        default=str(metrics_script_default),
        help="Path to extract_pipeline_metrics.py",
    )
    parser.add_argument("--metrics-sync-tolerance-ms", type=float, default=20.0)
    parser.add_argument("--metrics-cache-age-s", type=float, default=5.0)
    parser.add_argument("--metrics-max-latency-ms", type=float, default=1000.0)

    parser.add_argument(
        "--motion-launch-package",
        default="datasync_3_0",
        help="Package for motion compensation launch.",
    )
    parser.add_argument(
        "--motion-launch-file",
        default="motion_compensation.launch.py",
        help="Launch file for motion compensation.",
    )
    parser.add_argument(
        "--motion-params-file",
        default="",
        help="Optional params_file passed to motion launch.",
    )
    parser.add_argument(
        "--motion-extra-arg",
        action="append",
        default=[],
        help="Extra launch args for motion launch (key:=value). Repeatable.",
    )

    parser.add_argument("--enable-segmentation", action="store_true", default=True)
    parser.add_argument("--disable-segmentation", action="store_true")
    parser.add_argument("--seg-launch-package", default="event_segmentation")
    parser.add_argument("--seg-launch-file", default="segmentation.launch.py")
    parser.add_argument("--seg-params-file", default="")
    parser.add_argument(
        "--seg-extra-arg",
        action="append",
        default=[],
        help="Extra launch args for segmentation launch (key:=value). Repeatable.",
    )

    parser.add_argument("--enable-clustering", action="store_true", default=True)
    parser.add_argument("--disable-clustering", action="store_true")
    parser.add_argument("--cluster-launch-package", default="event_clustering_2_0")
    parser.add_argument("--cluster-launch-file", default="clustering_2_0.launch.py")
    parser.add_argument("--cluster-params-file", default="")
    parser.add_argument(
        "--cluster-extra-arg",
        action="append",
        default=[],
        help="Extra launch args for clustering launch (key:=value). Repeatable.",
    )

    parser.add_argument("--dry-run", action="store_true", help="Print plan without executing.")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    require_ros_environment()

    bags_root = Path(args.bags_root).expanduser().resolve()
    workspace_src = Path(args.workspace_src).expanduser().resolve()
    output_root = Path(args.output_root).expanduser().resolve()
    stage_root = workspace_src / ".bag_stage"
    metrics_script = Path(args.metrics_script).expanduser().resolve()

    if not bags_root.exists():
        raise FileNotFoundError(f"bags-root not found: {bags_root}")
    if not workspace_src.exists():
        raise FileNotFoundError(f"workspace-src not found: {workspace_src}")
    if not metrics_script.exists():
        raise FileNotFoundError(f"metrics script not found: {metrics_script}")

    enable_segmentation = bool(args.enable_segmentation and not args.disable_segmentation)
    enable_clustering = bool(args.enable_clustering and not args.disable_clustering)

    ignore_parts = {".bag_stage", "__pycache__", ".git"}
    bag_dirs = find_bag_dirs(bags_root, ignore_parts=tuple(ignore_parts))

    if args.bag_filter.strip():
        rx = re.compile(args.bag_filter.strip())
        bag_dirs = [b for b in bag_dirs if rx.search(str(b))]

    if not bag_dirs:
        print(f"No bag found under: {bags_root}")
        return 1

    a_vals = parse_float_list(args.a_sweep)
    b_vals = parse_float_list(args.b_sweep)
    g_a = parse_float_list(args.grid_a)
    g_b = parse_float_list(args.grid_b)
    ab_pairs = build_ab_sweep(args.anchor_a, args.anchor_b, a_vals, b_vals, g_a, g_b)
    if args.max_combos > 0:
        ab_pairs = ab_pairs[: args.max_combos]

    print(f"Found {len(bag_dirs)} bag(s).")
    print(f"Generated {len(ab_pairs)} (a,b) combo(s).")
    print(f"Output root: {output_root}")
    print(f"Stage mode: {args.stage_mode} (stage root: {stage_root})")
    print(f"Segmentation enabled: {enable_segmentation}")
    print(f"Clustering enabled: {enable_clustering}")

    if args.dry_run:
        for bag in bag_dirs:
            print(f"- bag: {bag}")
        for a, b in ab_pairs:
            print(f"- combo: a={a:.3f}, b={b:.3f}")
        return 0

    output_root.mkdir(parents=True, exist_ok=True)
    manifest_csv = output_root / "manifest.csv"

    total_runs = len(bag_dirs) * len(ab_pairs)
    run_idx = 0

    for bag_dir in bag_dirs:
        rel_bag_parent = rel_or_name(bag_dir.parent, bags_root)
        component_fn = sanitize_token if args.sanitize_output_names else filesystem_safe_component
        rel_parts = [component_fn(p) for p in rel_bag_parent.parts if p not in (".", "")]
        if not rel_parts:
            rel_parts = ["root"]
        campaign_dir = Path(*rel_parts)
        bag_name = component_fn(bag_dir.name)

        for a, b in ab_pairs:
            run_idx += 1
            ab_dir = fmt_ab_dir(a, b)
            run_name = f"{bag_name}__{ab_dir}"
            run_dir = output_root / campaign_dir / bag_name / ab_dir
            run_dir.mkdir(parents=True, exist_ok=True)

            print(
                f"[{run_idx}/{total_runs}] bag={bag_dir.name} "
                f"a={a:.3f} b={b:.3f}"
            )

            stage: Optional[StageHandle] = None
            ds_proc: Optional[subprocess.Popen] = None
            seg_proc: Optional[subprocess.Popen] = None
            cl_proc: Optional[subprocess.Popen] = None
            met_proc: Optional[subprocess.Popen] = None
            bag_proc: Optional[subprocess.Popen] = None
            status = "ok"
            error_msg = ""

            run_start_utc = now_utc()
            t_start = time.monotonic()
            duration_est = float("nan")
            metrics_duration = float("nan")
            bag_return = None
            metrics_return = None

            cmd_dump: Dict[str, List[str]] = {}

            try:
                stage = stage_bag(
                    bag_dir,
                    stage_root=stage_root,
                    mode=args.stage_mode,
                    keep_copy=bool(args.keep_staged_copy),
                )

                # Save bag metadata snapshot for reproducibility.
                try:
                    shutil.copy2(stage.staged_bag / "metadata.yaml", run_dir / "metadata.yaml")
                except Exception:
                    pass

                # ros2 bag info (text) for traceability.
                try:
                    info = subprocess.run(
                        ["ros2", "bag", "info", str(stage.staged_bag)],
                        check=False,
                        stdout=subprocess.PIPE,
                        stderr=subprocess.STDOUT,
                        text=True,
                    )
                    (run_dir / "bag_info.txt").write_text(info.stdout or "", encoding="utf-8")
                except Exception as exc:
                    (run_dir / "bag_info.txt").write_text(
                        f"Failed to run ros2 bag info: {exc}\n", encoding="utf-8"
                    )

                duration_est = read_bag_duration_sec(
                    stage.staged_bag, fallback_sec=args.fallback_bag_duration_s
                )
                metrics_duration = max(1.0, duration_est / max(0.01, args.play_rate) + args.tail_s)

                # 1) Launch motion compensation
                ds_cmd = [
                    "ros2",
                    "launch",
                    args.motion_launch_package,
                    args.motion_launch_file,
                    f"lambda_a:={a}",
                    f"lambda_b:={b}",
                ]
                if args.motion_params_file.strip():
                    ds_cmd.append(f"params_file:={args.motion_params_file.strip()}")
                ds_cmd.extend(args.motion_extra_arg)
                cmd_dump["motion_launch"] = ds_cmd
                ds_proc = start_logged_process(ds_cmd, run_dir / "motion_compensation.log")

                # 2) Launch segmentation (optional)
                if enable_segmentation:
                    seg_cmd = [
                        "ros2",
                        "launch",
                        args.seg_launch_package,
                        args.seg_launch_file,
                    ]
                    if args.seg_params_file.strip():
                        seg_cmd.append(f"params_file:={args.seg_params_file.strip()}")
                    seg_cmd.extend(args.seg_extra_arg)
                    cmd_dump["segmentation_launch"] = seg_cmd
                    seg_proc = start_logged_process(seg_cmd, run_dir / "segmentation.log")

                # 3) Launch clustering (optional)
                if enable_clustering:
                    cl_cmd = [
                        "ros2",
                        "launch",
                        args.cluster_launch_package,
                        args.cluster_launch_file,
                    ]
                    if args.cluster_params_file.strip():
                        cl_cmd.append(f"params_file:={args.cluster_params_file.strip()}")
                    cl_cmd.extend(args.cluster_extra_arg)
                    cmd_dump["clustering_launch"] = cl_cmd
                    cl_proc = start_logged_process(cl_cmd, run_dir / "clustering.log")

                time.sleep(max(0.0, args.startup_s))

                # 4) Start metrics extractor
                metrics_csv = run_dir / "metrics_summary.csv"
                met_cmd = [
                    "python3",
                    str(metrics_script),
                    "--duration",
                    f"{metrics_duration:.3f}",
                    "--run-name",
                    run_name,
                    "--summary-csv",
                    str(metrics_csv),
                    "--sync-tolerance-ms",
                    f"{args.metrics_sync_tolerance_ms}",
                    "--cache-age-s",
                    f"{args.metrics_cache_age_s}",
                    "--max-latency-ms",
                    f"{args.metrics_max_latency_ms}",
                ]
                cmd_dump["metrics"] = met_cmd
                met_proc = start_logged_process(met_cmd, run_dir / "metrics.log")

                # 5) Play bag once
                bag_cmd = [
                    "ros2",
                    "bag",
                    "play",
                    str(stage.staged_bag),
                    "--rate",
                    f"{args.play_rate}",
                ]
                cmd_dump["bag_play"] = bag_cmd
                bag_proc = start_logged_process(bag_cmd, run_dir / "bag_play.log")

                bag_timeout = duration_est / max(0.01, args.play_rate) + args.bag_timeout_pad_s
                bag_return = wait_with_timeout(bag_proc, timeout_s=bag_timeout)
                if bag_return is None:
                    status = "bag_timeout"
                    error_msg = f"Bag playback timeout after {bag_timeout:.1f}s"
                    stop_process_tree(bag_proc)
                elif bag_return != 0:
                    status = "bag_error"
                    error_msg = f"Bag playback returned code {bag_return}"

                metrics_timeout = metrics_duration + args.metrics_timeout_pad_s
                metrics_return = wait_with_timeout(met_proc, timeout_s=metrics_timeout)
                if metrics_return is None:
                    status = "metrics_timeout" if status == "ok" else status
                    if not error_msg:
                        error_msg = f"Metrics timeout after {metrics_timeout:.1f}s"
                    stop_process_tree(met_proc)
                elif metrics_return != 0 and status == "ok":
                    status = "metrics_error"
                    error_msg = f"Metrics process returned code {metrics_return}"

            except Exception as exc:
                status = "exception"
                error_msg = str(exc)
            finally:
                # Stop all processes (best effort).
                for p in [bag_proc, met_proc, cl_proc, seg_proc, ds_proc]:
                    stop_process_tree(p)
                for p in [bag_proc, met_proc, cl_proc, seg_proc, ds_proc]:
                    close_proc_log(p)

                # Restore/cleanup staging.
                if stage is not None:
                    try:
                        unstage_bag(stage)
                    except Exception as exc:
                        if status == "ok":
                            status = "unstage_error"
                            error_msg = str(exc)
                        else:
                            error_msg = f"{error_msg} | unstage_error={exc}"

            elapsed = time.monotonic() - t_start
            summary_row = load_single_row_csv(run_dir / "metrics_summary.csv")

            metadata = {
                "run_name": run_name,
                "status": status,
                "error": error_msg,
                "bag_original": str(bag_dir),
                "bag_campaign_rel": str(rel_bag_parent),
                "a": a,
                "b": b,
                "start_utc": run_start_utc,
                "elapsed_s": elapsed,
                "duration_est_s": duration_est,
                "metrics_duration_s": metrics_duration,
                "bag_return_code": bag_return,
                "metrics_return_code": metrics_return,
                "stage_mode": args.stage_mode,
                "stage_kept_copy": bool(args.keep_staged_copy),
                "commands": cmd_dump,
            }
            (run_dir / "run_metadata.json").write_text(
                json.dumps(metadata, indent=2, ensure_ascii=True),
                encoding="utf-8",
            )

            manifest_row: Dict[str, object] = {
                "run_name": run_name,
                "status": status,
                "error": error_msg,
                "bag_path": str(bag_dir),
                "campaign_rel": str(rel_bag_parent),
                "a": f"{a:.6f}",
                "b": f"{b:.6f}",
                "elapsed_s": f"{elapsed:.3f}",
                "duration_est_s": f"{duration_est:.3f}" if duration_est == duration_est else "nan",
                "metrics_duration_s": f"{metrics_duration:.3f}" if metrics_duration == metrics_duration else "nan",
                "run_dir": str(run_dir),
            }
            for key in SUMMARY_KEYS:
                manifest_row[key] = summary_row.get(key, "")

            append_csv_row(manifest_csv, manifest_row)

            print(
                f"  -> status={status} elapsed={elapsed:.1f}s "
                f"m2_density_gain={manifest_row.get('m2_density_gain_mean', '')} "
                f"m2_blur_red={manifest_row.get('m2_blur_reduction_mean', '')} "
                f"m3_iso={manifest_row.get('m3_iso_post_mean', '')} "
                f"m4_fg={manifest_row.get('m4_fg_ratio_mean', '')}"
            )

    print(f"Done. Manifest: {manifest_csv}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("Interrupted by user.")
        raise
