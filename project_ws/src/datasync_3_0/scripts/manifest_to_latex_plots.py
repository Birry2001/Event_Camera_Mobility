#!/usr/bin/env python3
"""
Generate LaTeX tables and plots by campaign from automation manifest.csv.

Input:
  - manifest.csv produced by automate_ab_bag_campaign.py

Outputs (under --output-dir):
  - per-campaign folders preserving campaign hierarchy
  - aggregated CSV files
  - LaTeX tables (top-k + full)
  - heatmaps and scatter plots
  - campaigns_report.tex (global include file for LaTeX report)
"""

from __future__ import annotations

import argparse
import csv
import math
import re
from collections import defaultdict
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Tuple

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


NUMERIC_KEYS = [
    "a",
    "b",
    "elapsed_s",
    "duration_est_s",
    "metrics_duration_s",
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
    # Legacy field kept for backward compatibility with older manifests.
    "m3_noise_score_mean",
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


HEATMAP_SPECS_COMPENSATION = [
    ("m2_density_gain_mean_mean", "Gain de densite (M2, ΔD)", "viridis", False),
    ("m2_blur_reduction_mean_mean", "Reduction du flou (M2, Δblur)", "coolwarm", False),
    ("m1_comp_lat_mean_ms_mean", "Latence compensation (M1, ms)", "cividis_r", True),
]

HEATMAP_SPECS_DETECTION = [
    ("m4_fg_ratio_mean_mean", "Ratio foreground (M4)", "magma", False),
    ("m6_mask_iou_mean_mean", "Continuite du masque (M6)", "plasma", False),
    ("m1_comp_lat_mean_ms_mean", "Latence compensation (M1, ms)", "cividis_r", True),
    ("m5_seg_lat_mean_ms_mean", "Latence segmentation (M5, ms)", "cividis_r", True),
]


def parse_float(v: str) -> float:
    try:
        x = float(v)
    except Exception:
        return float("nan")
    if not math.isfinite(x):
        return float("nan")
    return x


def sanitize_component(name: str) -> str:
    out = name.replace("/", "_").replace("\x00", "")
    out = re.sub(r"[^A-Za-z0-9._-]+", "_", out)
    out = out.strip("._-")
    if not out:
        out = "x"
    return out


def latex_escape(s: str) -> str:
    repl = {
        "\\": r"\textbackslash{}",
        "&": r"\&",
        "%": r"\%",
        "$": r"\$",
        "#": r"\#",
        "_": r"\_",
        "{": r"\{",
        "}": r"\}",
        "~": r"\textasciitilde{}",
        "^": r"\textasciicircum{}",
    }
    return "".join(repl.get(ch, ch) for ch in s)


def campaign_short_label(campaign: str) -> str:
    parts = [p for p in campaign.split("/") if p]
    if not parts:
        return campaign

    keep: List[str] = []
    keep.append(parts[0])
    if len(parts) >= 2:
        keep.append(parts[1])
    if len(parts) >= 3:
        keep.append(parts[2])
    if len(parts) >= 4:
        keep.append(parts[-1])

    dedup: List[str] = []
    seen = set()
    for p in keep:
        if p in seen:
            continue
        seen.add(p)
        dedup.append(p)

    pretty = " - ".join(dedup)
    pretty = pretty.replace("_", " ")
    return pretty


def campaign_group_label(campaign: str) -> str:
    if campaign.startswith("Campagne_1/"):
        return "Campagne 1"
    if campaign.startswith("Campagne__2/") or campaign.startswith("Campagne_2/"):
        return "Campagne 2"
    return "Autres"


def campaign_test_id(campaign: str) -> str:
    c = campaign.lower()
    if "no_mobile_obstacles/rotation_roll" in c:
        return "T1"
    if "no_mobile_obstacles/rotation_pitch" in c:
        return "T2"
    if "no_mobile_obstacles/rotation_yaw" in c:
        return "T3"
    if "mobile_obstacles/rotation_pitch/rapide_camera_lent_convoyeur" in c:
        return "T4"
    if "mobile_obstacles/rotation_pitch/rapide_camera_moins_lent_convoyeur" in c:
        return "T5"
    if "objet_bougeant_dans_le_meme_sens_que_la_camera" in c:
        return "T6"
    if "sens_oppos" in c:
        return "T7"
    return ""


def campaign_family(campaign: str) -> str:
    c = campaign.lower()
    if "no_mobile_obstacles" in c:
        return "compensation"
    return "detection"


def test_family_from_id(test_id: str) -> str:
    if test_id in {"T1", "T2", "T3"}:
        return "compensation"
    if test_id in {"T4", "T5", "T6", "T7"}:
        return "detection"
    return "other"


def test_label(test_id: str) -> str:
    labels = {
        "T1": "rotation roll (sans objet)",
        "T2": "rotation pitch (sans objet)",
        "T3": "rotation yaw (sans objet)",
        "T4": "objet mobile, camera rapide/convoyeur lent",
        "T5": "objet mobile, camera rapide/convoyeur moins lent",
        "T6": "objet mobile, meme sens que la camera",
        "T7": "objet mobile, sens oppose a la camera",
    }
    return labels.get(test_id, "")


def format_metric(v: float, precision: int = 3) -> str:
    if not math.isfinite(v):
        return "nan"
    return f"{v:.{precision}f}"


def read_manifest(path: Path, keep_status: Optional[str]) -> List[Dict[str, object]]:
    rows: List[Dict[str, object]] = []
    with path.open("r", encoding="utf-8", newline="") as f:
        reader = csv.DictReader(f)
        for raw in reader:
            status = str(raw.get("status", "")).strip()
            if keep_status and status != keep_status:
                continue
            row: Dict[str, object] = dict(raw)
            for k in NUMERIC_KEYS:
                row[k] = parse_float(str(raw.get(k, "")))
            # Backward compatibility for M2 naming:
            # old manifests exposed m2_keep_ratio_mean, while new ones expose
            # m2_warp_valid_ratio_mean (same numeric definition in this pipeline).
            m2_keep = float(row.get("m2_keep_ratio_mean", float("nan")))
            m2_warp = float(row.get("m2_warp_valid_ratio_mean", float("nan")))
            if not math.isfinite(m2_warp) and math.isfinite(m2_keep):
                row["m2_warp_valid_ratio_mean"] = m2_keep
                m2_warp = m2_keep
            if not math.isfinite(m2_keep) and math.isfinite(m2_warp):
                row["m2_keep_ratio_mean"] = m2_warp
            m2_loss = float(row.get("m2_projection_loss_mean", float("nan")))
            if not math.isfinite(m2_loss) and math.isfinite(m2_warp):
                row["m2_projection_loss_mean"] = 1.0 - m2_warp
            # Backward compatibility: old manifests used m3_noise_score_mean.
            if not math.isfinite(float(row.get("m3_iso_post_mean", float("nan")))):
                legacy_m3 = float(row.get("m3_noise_score_mean", float("nan")))
                if math.isfinite(legacy_m3):
                    row["m3_iso_post_mean"] = legacy_m3
            row["campaign_rel"] = str(raw.get("campaign_rel", "")).strip()
            row["status"] = status
            rows.append(row)
    return rows


def finite_values(rows: Iterable[Dict[str, object]], key: str) -> List[float]:
    vals: List[float] = []
    for r in rows:
        v = r.get(key, float("nan"))
        if isinstance(v, (int, float)) and math.isfinite(float(v)):
            vals.append(float(v))
    return vals


def agg_mean(rows: Iterable[Dict[str, object]], key: str) -> float:
    vals = finite_values(rows, key)
    if not vals:
        return float("nan")
    return float(np.mean(vals))


def agg_std(rows: Iterable[Dict[str, object]], key: str) -> float:
    vals = finite_values(rows, key)
    if not vals:
        return float("nan")
    return float(np.std(vals))


def m3_iso_value(entry: Dict[str, object]) -> float:
    """Primary M3 value: post-compensation isolated-event ratio (iso_post)."""
    v = float(entry.get("m3_iso_post_mean_mean", float("nan")))
    if math.isfinite(v):
        return v
    # Fallback for older report assets.
    return float(entry.get("m3_noise_score_mean_mean", float("nan")))


def build_campaign_combo_aggregates(rows: List[Dict[str, object]]) -> Dict[str, List[Dict[str, object]]]:
    by_campaign: Dict[str, Dict[Tuple[float, float], List[Dict[str, object]]]] = defaultdict(lambda: defaultdict(list))

    for r in rows:
        campaign = str(r["campaign_rel"])
        a = float(r.get("a", float("nan")))
        b = float(r.get("b", float("nan")))
        if not (math.isfinite(a) and math.isfinite(b)):
            continue
        by_campaign[campaign][(round(a, 6), round(b, 6))].append(r)

    out: Dict[str, List[Dict[str, object]]] = {}
    for campaign, combos in by_campaign.items():
        entries: List[Dict[str, object]] = []
        for (a, b), rr in sorted(combos.items(), key=lambda x: (x[0][0], x[0][1])):
            e: Dict[str, object] = {
                "campaign_rel": campaign,
                "a": float(a),
                "b": float(b),
                "n_runs": len(rr),
            }
            for k in NUMERIC_KEYS:
                if k in ("a", "b"):
                    continue
                e[f"{k}_mean"] = agg_mean(rr, k)
                e[f"{k}_std"] = agg_std(rr, k)
            entries.append(e)
        out[campaign] = entries
    return out


def normalize_map(values: Dict[Tuple[float, float], float], lower_better: bool) -> Dict[Tuple[float, float], float]:
    finite_items = [(k, v) for k, v in values.items() if math.isfinite(v)]
    if not finite_items:
        return {k: float("nan") for k in values}

    arr = np.array([v for _, v in finite_items], dtype=float)
    vmin = float(np.min(arr))
    vmax = float(np.max(arr))
    out: Dict[Tuple[float, float], float] = {}
    if abs(vmax - vmin) < 1e-12:
        for k in values:
            out[k] = 0.5 if math.isfinite(values[k]) else float("nan")
        return out

    for k, v in values.items():
        if not math.isfinite(v):
            out[k] = float("nan")
            continue
        n = (v - vmin) / (vmax - vmin)
        out[k] = float(n if lower_better else (1.0 - n))
    return out


def compute_selection_scores(entries: List[Dict[str, object]], family: str) -> None:
    key = lambda e: (float(e["a"]), float(e["b"]))

    if family == "compensation":
        m_blur = {key(e): float(e.get("m2_blur_reduction_mean_mean", float("nan"))) for e in entries}
        m_density = {key(e): float(e.get("m2_density_gain_mean_mean", float("nan"))) for e in entries}
        m_lat = {key(e): float(e.get("m1_comp_lat_mean_ms_mean", float("nan"))) for e in entries}

        c_blur = normalize_map(m_blur, lower_better=False)
        c_density = normalize_map(m_density, lower_better=False)
        c_lat = normalize_map(m_lat, lower_better=True)

        for e in entries:
            k = key(e)
            comps = [c_blur.get(k, float("nan")), c_density.get(k, float("nan")), c_lat.get(k, float("nan"))]
            if any(not math.isfinite(x) for x in comps):
                e["selection_score"] = float("nan")
            else:
                # lower score is better after normalization
                e["selection_score"] = 0.50 * comps[0] + 0.30 * comps[1] + 0.20 * comps[2]
            e["selection_family"] = "compensation"
        return

    m_fg = {key(e): float(e.get("m4_fg_ratio_mean_mean", float("nan"))) for e in entries}
    m_iou = {key(e): float(e.get("m6_mask_iou_mean_mean", float("nan"))) for e in entries}
    m_lat = {key(e): float(e.get("m1_comp_lat_mean_ms_mean", float("nan"))) for e in entries}

    c_fg = normalize_map(m_fg, lower_better=False)
    c_iou = normalize_map(m_iou, lower_better=False)
    c_lat = normalize_map(m_lat, lower_better=True)

    for e in entries:
        k = key(e)
        comps = [c_fg.get(k, float("nan")), c_iou.get(k, float("nan")), c_lat.get(k, float("nan"))]
        if any(not math.isfinite(x) for x in comps):
            e["selection_score"] = float("nan")
        else:
            # lower score is better after normalization
            e["selection_score"] = 0.45 * comps[0] + 0.35 * comps[1] + 0.20 * comps[2]
        e["selection_family"] = "detection"


def compute_composite_scores(
    entries: List[Dict[str, object]],
    w_m3: float,
    w_fg: float,
    w_iou: float,
    w_lat: float,
) -> None:
    key = lambda e: (float(e["a"]), float(e["b"]))
    m_m3 = {key(e): m3_iso_value(e) for e in entries}
    m_fg = {key(e): float(e.get("m4_fg_ratio_mean_mean", float("nan"))) for e in entries}
    m_iou = {key(e): float(e.get("m6_mask_iou_mean_mean", float("nan"))) for e in entries}
    m_lat = {key(e): float(e.get("m1_comp_lat_mean_ms_mean", float("nan"))) for e in entries}

    c_m3 = normalize_map(m_m3, lower_better=True)
    c_fg = normalize_map(m_fg, lower_better=False)
    c_iou = normalize_map(m_iou, lower_better=False)
    c_lat = normalize_map(m_lat, lower_better=True)

    for e in entries:
        k = key(e)
        comps = [c_m3.get(k, float("nan")), c_fg.get(k, float("nan")), c_iou.get(k, float("nan")), c_lat.get(k, float("nan"))]
        if any(not math.isfinite(x) for x in comps):
            e["composite_score"] = float("nan")
            continue
        score = w_m3 * comps[0] + w_fg * comps[1] + w_iou * comps[2] + w_lat * comps[3]
        e["composite_score"] = float(score)


def write_csv(path: Path, rows: List[Dict[str, object]], fieldnames: List[str]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames, extrasaction="ignore")
        writer.writeheader()
        for r in rows:
            writer.writerow(r)


def table_rows_sorted(entries: List[Dict[str, object]], score_key: str = "selection_score") -> List[Dict[str, object]]:
    def k(e: Dict[str, object]) -> Tuple[float, float]:
        s = float(e.get(score_key, float("nan")))
        if not math.isfinite(s):
            s = float("inf")
        return (s, float(e["a"]), float(e["b"]))

    return sorted(entries, key=k)


def write_latex_table(
    path: Path,
    campaign: str,
    rows: List[Dict[str, object]],
    family: str,
    top_k: Optional[int] = None,
) -> None:
    sel = rows if top_k is None else rows[:top_k]
    cap_suffix = f" (top {len(sel)})" if top_k is not None else ""
    short = campaign_short_label(campaign)
    lines = []
    lines.append(r"\begin{table}[H]")
    lines.append(r"\centering")
    lines.append(
        r"\caption{"
        + latex_escape(f"{short} - synthese des combinaisons (a,b){cap_suffix}")
        + r"}"
    )
    if family == "compensation":
        lines.append(r"\begin{tabular}{rrrrrrr}")
        lines.append(r"\toprule")
        lines.append(
            r"$a$ & $b$ & $N$ & $\Delta D$ (M2) $\uparrow$ & $\Delta blur$ (M2) $\uparrow$ & "
            r"M1(ms) $\downarrow$ & Critere $\downarrow$ \\"
        )
    else:
        lines.append(r"\begin{tabular}{rrrrrrr}")
        lines.append(r"\toprule")
        lines.append(
            r"$a$ & $b$ & $N$ & FG (M4) $\uparrow$ & IoU (M6) $\uparrow$ & "
            r"M1(ms) $\downarrow$ & Critere $\downarrow$ \\"
        )
    lines.append(r"\midrule")
    for e in sel:
        if family == "compensation":
            lines.append(
                " & ".join(
                    [
                        format_metric(float(e["a"]), 3),
                        format_metric(float(e["b"]), 3),
                        str(int(e.get("n_runs", 0))),
                        format_metric(float(e.get("m2_density_gain_mean_mean", float("nan"))), 4),
                        format_metric(float(e.get("m2_blur_reduction_mean_mean", float("nan"))), 4),
                        format_metric(float(e.get("m1_comp_lat_mean_ms_mean", float("nan"))), 3),
                        format_metric(float(e.get("selection_score", float("nan"))), 4),
                    ]
                )
                + r" \\"
            )
        else:
            lines.append(
                " & ".join(
                    [
                        format_metric(float(e["a"]), 3),
                        format_metric(float(e["b"]), 3),
                        str(int(e.get("n_runs", 0))),
                        format_metric(float(e.get("m4_fg_ratio_mean_mean", float("nan"))), 4),
                        format_metric(float(e.get("m6_mask_iou_mean_mean", float("nan"))), 4),
                        format_metric(float(e.get("m1_comp_lat_mean_ms_mean", float("nan"))), 3),
                        format_metric(float(e.get("selection_score", float("nan"))), 4),
                    ]
                )
                + r" \\"
            )
    lines.append(r"\bottomrule")
    lines.append(r"\end{tabular}")
    lines.append(r"\vspace{0.2em}")
    lines.append(
        r"{\footnotesize \emph{Lecture des fleches:} $\uparrow$ = plus grand est meilleur, "
        r"$\downarrow$ = plus petit est meilleur.}"
    )
    lines.append(r"\end{table}")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def plot_heatmap(
    entries: List[Dict[str, object]],
    metric_key: str,
    title: str,
    cmap: str,
    out_path: Path,
    lower_better: bool,
) -> None:
    a_vals = sorted({float(e["a"]) for e in entries})
    b_vals = sorted({float(e["b"]) for e in entries})
    if not a_vals or not b_vals:
        return

    mat = np.full((len(b_vals), len(a_vals)), np.nan, dtype=float)
    for e in entries:
        a = float(e["a"])
        b = float(e["b"])
        v = float(e.get(metric_key, float("nan")))
        i = b_vals.index(b)
        j = a_vals.index(a)
        mat[i, j] = v

    fig_w = max(6.0, 1.2 * len(a_vals) + 2.5)
    fig_h = max(4.5, 0.8 * len(b_vals) + 2.0)
    fig, ax = plt.subplots(figsize=(fig_w, fig_h))
    im = ax.imshow(mat, origin="lower", aspect="auto", cmap=cmap)
    ax.set_xticks(np.arange(len(a_vals)))
    ax.set_xticklabels([f"{x:.2f}" for x in a_vals])
    ax.set_yticks(np.arange(len(b_vals)))
    ax.set_yticklabels([f"{x:.2f}" for x in b_vals])
    ax.set_xlabel("a")
    ax.set_ylabel("b")
    arrow = "↓" if lower_better else "↑"
    ax.set_title(f"{title} {arrow}")

    for i in range(len(b_vals)):
        for j in range(len(a_vals)):
            v = mat[i, j]
            if not math.isfinite(float(v)):
                continue
            txt = f"{v:.3f}"
            ax.text(j, i, txt, ha="center", va="center", fontsize=8, color="white")

    cbar = fig.colorbar(im, ax=ax)
    cbar.ax.set_ylabel(title, rotation=90)
    fig.tight_layout()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=160)
    plt.close(fig)


def plot_pareto(entries: List[Dict[str, object]], family: str, out_path: Path) -> None:
    if family == "compensation":
        x = np.array([float(e.get("m1_comp_lat_mean_ms_mean", float("nan"))) for e in entries], dtype=float)
        y = np.array([float(e.get("m2_blur_reduction_mean_mean", float("nan"))) for e in entries], dtype=float)
        c = np.array([float(e.get("m2_density_gain_mean_mean", float("nan"))) for e in entries], dtype=float)
        title = "Pareto: latence compensation vs reduction du flou"
        xlabel = "Latence compensation M1 (ms, plus petit = meilleur)"
        ylabel = "Reduction du flou M2 (plus grand = meilleur)"
        cbar_label = "Gain de densite M2 (plus grand = meilleur)"
    else:
        x = np.array([float(e.get("m1_comp_lat_mean_ms_mean", float("nan"))) for e in entries], dtype=float)
        y = np.array([float(e.get("m4_fg_ratio_mean_mean", float("nan"))) for e in entries], dtype=float)
        c = np.array([float(e.get("m6_mask_iou_mean_mean", float("nan"))) for e in entries], dtype=float)
        title = "Pareto: latence compensation vs preservation du foreground"
        xlabel = "Latence compensation M1 (ms, plus petit = meilleur)"
        ylabel = "Ratio foreground M4 (plus grand = meilleur)"
        cbar_label = "Continuite masque M6 (plus grand = meilleur)"

    s = np.array([float(e.get("n_runs", 1)) for e in entries], dtype=float)
    score = np.array([float(e.get("selection_score", float("nan"))) for e in entries], dtype=float)
    a = np.array([float(e["a"]) for e in entries], dtype=float)
    b = np.array([float(e["b"]) for e in entries], dtype=float)

    valid = np.isfinite(x) & np.isfinite(y) & np.isfinite(c)
    if not np.any(valid):
        return

    fig, ax = plt.subplots(figsize=(7.0, 5.2))
    sizes = 40.0 + 18.0 * np.clip(s[valid], 0.0, 8.0)
    sc = ax.scatter(x[valid], y[valid], c=c[valid], s=sizes, cmap="viridis", alpha=0.9, edgecolors="k", linewidths=0.5)
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    ax.grid(True, alpha=0.3, linestyle="--")
    cb = fig.colorbar(sc, ax=ax)
    cb.ax.set_ylabel(cbar_label)

    # Annotate top 5 by family-specific selection score.
    idx = np.argsort(np.where(np.isfinite(score), score, np.inf))[:5]
    for i in idx:
        if not valid[i]:
            continue
        ax.annotate(
            f"({a[i]:.2f},{b[i]:.2f})",
            (x[i], y[i]),
            textcoords="offset points",
            xytext=(6, 4),
            fontsize=8,
        )

    fig.tight_layout()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=170)
    plt.close(fig)


def write_campaign_pareto_main_tex(
    campaign: str,
    out_path: Path,
    family: str,
    pareto_rel: Path,
    best_entry: Dict[str, object],
) -> None:
    short = campaign_short_label(campaign)
    esc = latex_escape(short)
    lines: List[str] = []
    lines.append(r"\subsection*{" + esc + r"}")
    lines.append("")
    lines.append(r"\begin{figure}[H]")
    lines.append(r"\centering")
    lines.append(r"\includegraphics[width=0.62\linewidth]{" + pareto_rel.as_posix() + r"}")
    if family == "compensation":
        pareto_caption = (
            f"{short} - Pareto du compromis latence (M1) vs reduction du flou (M2), "
            f"couleur = gain de densite (M2)"
        )
    else:
        pareto_caption = (
            f"{short} - Pareto du compromis latence (M1) vs preservation du foreground (M4), "
            f"couleur = continuite (M6)"
        )
    lines.append(r"\caption{" + latex_escape(pareto_caption) + r"}")
    lines.append(r"\end{figure}")
    a = float(best_entry.get("a", float("nan")))
    b = float(best_entry.get("b", float("nan")))
    if family == "compensation":
        blur = float(best_entry.get("m2_blur_reduction_mean_mean", float("nan")))
        dens = float(best_entry.get("m2_density_gain_mean_mean", float("nan")))
        lat = float(best_entry.get("m1_comp_lat_mean_ms_mean", float("nan")))
        lines.append(
            r"\noindent\textit{Conclusion locale.} "
            + latex_escape(
                f"Le meilleur compromis visible sur ce Pareto est obtenu pour (a,b)=({a:.2f},{b:.2f}), "
                f"avec Δblur={blur:.4f}, ΔD={dens:.4f} et M1={lat:.2f} ms."
            )
        )
    else:
        fg = float(best_entry.get("m4_fg_ratio_mean_mean", float("nan")))
        iou = float(best_entry.get("m6_mask_iou_mean_mean", float("nan")))
        lat = float(best_entry.get("m1_comp_lat_mean_ms_mean", float("nan")))
        lines.append(
            r"\noindent\textit{Conclusion locale.} "
            + latex_escape(
                f"Le meilleur compromis visible sur ce Pareto est obtenu pour (a,b)=({a:.2f},{b:.2f}), "
                f"avec FG={fg:.4f}, IoU={iou:.4f} et M1={lat:.2f} ms."
            )
        )
    lines.append("")
    out_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def write_campaign_heatmap_annex_tex(
    campaign: str,
    out_path: Path,
    family: str,
    heatmaps: List[Path],
) -> None:
    short = campaign_short_label(campaign)
    esc = latex_escape(short)
    lines: List[str] = []
    lines.append(r"\subsection*{" + esc + r"}")
    lines.append("")
    if family == "compensation":
        lines.append(
            r"\noindent Cette annexe rassemble les heatmaps de compensation pour ce test "
            r"(gain de densite M2, reduction du flou M2, latence M1) en fonction du couple $(a,b)$."
        )
    else:
        lines.append(
            r"\noindent Cette annexe rassemble les heatmaps de segmentation/clustering pour ce test "
            r"(FG M4, IoU M6, latences M1 et M5) en fonction du couple $(a,b)$."
        )
    lines.append("")
    lines.append(r"\begin{figure}[H]")
    lines.append(r"\centering")
    for i, h in enumerate(heatmaps):
        lines.append(r"\includegraphics[width=0.48\linewidth]{" + h.as_posix() + r"}")
        if i % 2 == 1:
            lines.append(r"\\")
    if family == "compensation":
        heat_caption = f"{short} - heatmaps des metriques de compensation (M2, M1) selon (a,b)"
    else:
        heat_caption = f"{short} - heatmaps des metriques de segmentation/clustering (M4, M5, M6) selon (a,b)"
    lines.append(r"\caption{" + latex_escape(heat_caption) + r"}")
    lines.append(r"\end{figure}")
    lines.append("")
    out_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def write_best_by_campaign_tex(path: Path, rows: List[Dict[str, object]]) -> None:
    lines: List[str] = []
    test_order = {"T1": 1, "T2": 2, "T3": 3, "T4": 4, "T5": 5, "T6": 6, "T7": 7}

    det_rows: List[Dict[str, object]] = []
    for r in rows:
        campaign = str(r.get("campaign_rel", ""))
        tid = campaign_test_id(campaign)
        fam = test_family_from_id(tid)
        rr = dict(r)
        rr["test_id"] = tid
        rr["test_label"] = test_label(tid)
        if fam == "detection":
            det_rows.append(rr)

    det_rows = sorted(det_rows, key=lambda r: test_order.get(str(r.get("test_id", "")), 999))

    if det_rows:
        lines.append(r"\begin{table}[ht]")
        lines.append(r"\centering")
        lines.append(
            r"\caption{"
            + latex_escape(
                "Meilleurs couples (a,b) par test avec objet mobile (T4-T7) "
                "- selection sur criteres de segmentation/clustering"
            )
            + r"}"
        )
        lines.append(r"\resizebox{\linewidth}{!}{%")
        lines.append(r"\begin{tabular}{p{0.9cm}p{4.7cm}rrrrrr}")
        lines.append(r"\toprule")
        lines.append(
            r"Test & Configuration & $a$ & $b$ & FG (M4) $\uparrow$ & "
            r"IoU (M6) $\uparrow$ & M1 (ms) $\downarrow$ & M5 (ms) $\downarrow$ \\"
        )
        lines.append(r"\midrule")
        for r in det_rows:
            lines.append(
                " & ".join(
                    [
                        latex_escape(str(r.get("test_id", ""))),
                        latex_escape(str(r.get("test_label", ""))),
                        format_metric(float(r.get("a", float("nan"))), 3),
                        format_metric(float(r.get("b", float("nan"))), 3),
                        format_metric(float(r.get("m4_fg_ratio_mean_mean", float("nan"))), 4),
                        format_metric(float(r.get("m6_mask_iou_mean_mean", float("nan"))), 4),
                        format_metric(float(r.get("m1_comp_lat_mean_ms_mean", float("nan"))), 2),
                        format_metric(float(r.get("m5_seg_lat_mean_ms_mean", float("nan"))), 2),
                    ]
                )
                + r" \\"
            )
        lines.append(r"\bottomrule")
        lines.append(r"\end{tabular}")
        lines.append(r"}")
        lines.append(r"\vspace{0.2em}")
        lines.append(
            r"{\footnotesize \emph{Lecture des fleches:} $\uparrow$ = plus grand est meilleur, "
            r"$\downarrow$ = plus petit est meilleur.}"
        )
        # Small scientific-style comment on optimal (a,b) values.
        ab_freq: Dict[Tuple[float, float], int] = defaultdict(int)
        for r in det_rows:
            aa = float(r.get("a", float("nan")))
            bb = float(r.get("b", float("nan")))
            if math.isfinite(aa) and math.isfinite(bb):
                ab_freq[(round(aa, 3), round(bb, 3))] += 1
        if ab_freq:
            dominant = sorted(ab_freq.items(), key=lambda kv: (-kv[1], kv[0][0], kv[0][1]))
            dom_ab, dom_n = dominant[0]
            fg_vals = [float(r.get("m4_fg_ratio_mean_mean", float("nan"))) for r in det_rows]
            iou_vals = [float(r.get("m6_mask_iou_mean_mean", float("nan"))) for r in det_rows]
            m1_vals = [float(r.get("m1_comp_lat_mean_ms_mean", float("nan"))) for r in det_rows]
            m5_vals = [float(r.get("m5_seg_lat_mean_ms_mean", float("nan"))) for r in det_rows]

            def finite_range(vals: List[float]) -> Tuple[float, float]:
                f = [v for v in vals if math.isfinite(v)]
                if not f:
                    return float("nan"), float("nan")
                return min(f), max(f)

            fg_min, fg_max = finite_range(fg_vals)
            iou_min, iou_max = finite_range(iou_vals)
            m1_min, m1_max = finite_range(m1_vals)
            m5_min, m5_max = finite_range(m5_vals)

            top_fg = max(
                [r for r in det_rows if math.isfinite(float(r.get("m4_fg_ratio_mean_mean", float("nan"))))],
                key=lambda r: float(r.get("m4_fg_ratio_mean_mean", float("-inf"))),
                default=None,
            )
            top_iou = max(
                [r for r in det_rows if math.isfinite(float(r.get("m6_mask_iou_mean_mean", float("nan"))))],
                key=lambda r: float(r.get("m6_mask_iou_mean_mean", float("-inf"))),
                default=None,
            )

            lines.append(r"\vspace{0.2em}")
            comment_parts: List[str] = []
            comment_parts.append(
                f"Le couple le plus recurrent est ({dom_ab[0]:.3f},{dom_ab[1]:.3f}), "
                f"retenu dans {dom_n} test(s) sur {len(det_rows)}."
            )
            comment_parts.append(
                f"Sur l'ensemble des tests, le ratio de foreground M4 varie de {fg_min:.4f} a {fg_max:.4f}, "
                f"la continuite M6 (IoU) de {iou_min:.4f} a {iou_max:.4f}, "
                f"la latence de compensation M1 de {m1_min:.2f} a {m1_max:.2f} ms "
                f"et la latence de segmentation M5 de {m5_min:.2f} a {m5_max:.2f} ms."
            )
            if top_fg is not None:
                comment_parts.append(
                    f"Le maximum de foreground est obtenu sur {top_fg.get('test_id', '')} "
                    f"avec le reglage ({float(top_fg.get('a', float('nan'))):.3f},{float(top_fg.get('b', float('nan'))):.3f})."
                )
            if top_iou is not None:
                comment_parts.append(
                    f"La meilleure continuite temporelle est observee sur {top_iou.get('test_id', '')} "
                    f"avec ({float(top_iou.get('a', float('nan'))):.3f},{float(top_iou.get('b', float('nan'))):.3f})."
                )
            comment_parts.append(
                "Ces resultats confirment qu'il n'existe pas de reglage universel: "
                "le choix final de (a,b) doit rester un compromis adapte a la dynamique du test."
            )
            lines.append(
                r"{\footnotesize \emph{Commentaire detaille.} "
                + latex_escape(" ".join(comment_parts))
                + r"}"
            )
        lines.append(r"\end{table}")
        lines.append("")

    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Generate LaTeX tables and plots by campaign from manifest.csv."
    )
    parser.add_argument(
        "--manifest",
        required=True,
        help="Path to manifest.csv produced by automation.",
    )
    parser.add_argument(
        "--output-dir",
        default="",
        help="Output directory (default: <manifest_dir>/report_assets).",
    )
    parser.add_argument(
        "--status",
        default="ok",
        help="Keep only rows with this status (default: ok). Use '' to keep all.",
    )
    parser.add_argument("--top-k", type=int, default=8, help="Top K combos shown in LaTeX top table.")
    parser.add_argument(
        "--weight-m3",
        type=float,
        default=0.50,
        help="Weight for M3 (iso_post) in composite score.",
    )
    parser.add_argument(
        "--weight-noise",
        type=float,
        default=None,
        help="Deprecated alias of --weight-m3 (kept for backward compatibility).",
    )
    parser.add_argument("--weight-fg", type=float, default=0.25)
    parser.add_argument("--weight-iou", type=float, default=0.15)
    parser.add_argument("--weight-lat", type=float, default=0.10)
    parser.add_argument(
        "--campaign-filter",
        default="",
        help="Regex filter on campaign_rel (optional).",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    manifest = Path(args.manifest).expanduser().resolve()
    if not manifest.exists():
        raise FileNotFoundError(f"Manifest not found: {manifest}")

    out_root = (
        Path(args.output_dir).expanduser().resolve()
        if args.output_dir.strip()
        else (manifest.parent / "report_assets").resolve()
    )
    out_root.mkdir(parents=True, exist_ok=True)

    status = args.status.strip()
    keep_status = status if status else None
    rows = read_manifest(manifest, keep_status=keep_status)
    if not rows:
        print("No rows after filtering.")
        return 1

    if args.campaign_filter.strip():
        rx = re.compile(args.campaign_filter.strip())
        rows = [r for r in rows if rx.search(str(r.get("campaign_rel", "")))]
        if not rows:
            print("No rows after campaign filter.")
            return 1

    agg = build_campaign_combo_aggregates(rows)
    campaigns = sorted(agg.keys())
    if not campaigns:
        print("No campaign aggregates.")
        return 1

    index_main_pareto_sections: List[str] = []
    index_annex_heatmap_sections: List[str] = []
    global_best_rows: List[Dict[str, object]] = []
    weight_m3 = float(args.weight_m3 if args.weight_noise is None else args.weight_noise)

    for campaign in campaigns:
        entries = agg[campaign]
        if not entries:
            continue

        family = campaign_family(campaign)
        compute_selection_scores(entries, family=family)
        sorted_rows = table_rows_sorted(entries, score_key="selection_score")

        campaign_dir = out_root / Path(*[sanitize_component(p) for p in campaign.split("/") if p])
        campaign_dir.mkdir(parents=True, exist_ok=True)

        # Save aggregated CSV for the campaign.
        csv_fields = [
            "campaign_rel",
            "a",
            "b",
            "n_runs",
            "m1_comp_lat_mean_ms_mean",
            "m2_density_pre_mean_mean",
            "m2_density_post_mean_mean",
            "m2_density_gain_mean_mean",
            "m2_blur_pre_mean_mean",
            "m2_blur_post_mean_mean",
            "m2_blur_reduction_mean_mean",
            "m2_warp_valid_ratio_mean_mean",
            "m2_projection_loss_mean_mean",
            "m4_fg_ratio_mean_mean",
            "m5_seg_lat_mean_ms_mean",
            "m6_mask_iou_mean_mean",
            "selection_score",
            "selection_family",
        ]
        write_csv(campaign_dir / "campaign_aggregated.csv", sorted_rows, csv_fields)

        # Save LaTeX tables.
        table_all = campaign_dir / "table_all.tex"
        table_top = campaign_dir / "table_top.tex"
        write_latex_table(table_all, campaign, sorted_rows, family=family, top_k=None)
        write_latex_table(table_top, campaign, sorted_rows, family=family, top_k=max(1, int(args.top_k)))

        # Save plots.
        heatmap_specs = HEATMAP_SPECS_COMPENSATION if family == "compensation" else HEATMAP_SPECS_DETECTION
        heat_rel_paths: List[Path] = []
        for metric_key, title, cmap, lower_better in heatmap_specs:
            fname = f"heatmap_{metric_key}.png"
            out = campaign_dir / fname
            plot_heatmap(
                sorted_rows,
                metric_key=metric_key,
                title=title,
                cmap=cmap,
                out_path=out,
                lower_better=lower_better,
            )
            heat_rel_paths.append(out.relative_to(out_root))

        pareto_name = "pareto_compensation.png" if family == "compensation" else "pareto_detection.png"
        pareto = campaign_dir / pareto_name
        plot_pareto(sorted_rows, family=family, out_path=pareto)

        # campaign section tex for annex heatmaps (mobile-object tests only)
        heatmap_annex_tex = campaign_dir / "campaign_heatmaps_annex.tex"
        write_campaign_heatmap_annex_tex(
            campaign=campaign,
            out_path=heatmap_annex_tex,
            family=family,
            heatmaps=heat_rel_paths,
        )
        if family == "detection":
            index_annex_heatmap_sections.append(heatmap_annex_tex.relative_to(out_root).as_posix())

        # campaign section tex for main pareto figures (mobile-object tests only)
        if family == "detection":
            pareto_main_tex = campaign_dir / "campaign_pareto_main.tex"
            write_campaign_pareto_main_tex(
                campaign=campaign,
                out_path=pareto_main_tex,
                family=family,
                pareto_rel=pareto.relative_to(out_root),
                best_entry=sorted_rows[0],
            )
            index_main_pareto_sections.append(pareto_main_tex.relative_to(out_root).as_posix())

        # best row for global summary
        best = sorted_rows[0]
        global_best_rows.append(
            {
                "campaign_rel": campaign,
                "a": best["a"],
                "b": best["b"],
                "family": family,
                "m2_density_gain_mean_mean": best.get("m2_density_gain_mean_mean"),
                "m2_blur_reduction_mean_mean": best.get("m2_blur_reduction_mean_mean"),
                "m4_fg_ratio_mean_mean": best.get("m4_fg_ratio_mean_mean"),
                "m6_mask_iou_mean_mean": best.get("m6_mask_iou_mean_mean"),
                "m1_comp_lat_mean_ms_mean": best.get("m1_comp_lat_mean_ms_mean"),
                "m5_seg_lat_mean_ms_mean": best.get("m5_seg_lat_mean_ms_mean"),
                "selection_score": best.get("selection_score"),
            }
        )

    # Global best-per-campaign CSV.
    global_best_csv = out_root / "best_by_campaign.csv"
    global_fields = [
        "campaign_rel",
        "a",
        "b",
        "family",
        "m2_density_gain_mean_mean",
        "m2_blur_reduction_mean_mean",
        "m4_fg_ratio_mean_mean",
        "m6_mask_iou_mean_mean",
        "m1_comp_lat_mean_ms_mean",
        "m5_seg_lat_mean_ms_mean",
        "selection_score",
    ]
    write_csv(global_best_csv, global_best_rows, global_fields)
    write_best_by_campaign_tex(out_root / "best_by_campaign.tex", global_best_rows)

    # Global LaTeX include file for main report: pareto only, detection-family tests.
    main_pareto_tex = out_root / "campaigns_pareto_main.tex"
    tex_lines_main: List[str] = []
    tex_lines_main.append("% Auto-generated by manifest_to_latex_plots.py")
    tex_lines_main.append("% Include from your report with: \\input{<path>/campaigns_pareto_main.tex}")
    tex_lines_main.append("")
    for sec in index_main_pareto_sections:
        tex_lines_main.append(r"\input{" + sec + r"}")
        tex_lines_main.append("")
    main_pareto_tex.write_text("\n".join(tex_lines_main) + "\n", encoding="utf-8")

    # Global LaTeX include file for annexes: mobile-object heatmaps only.
    annex_heatmaps_tex = out_root / "annex_heatmaps.tex"
    tex_lines_annex: List[str] = []
    tex_lines_annex.append("% Auto-generated by manifest_to_latex_plots.py")
    tex_lines_annex.append("% Include from annex with: \\input{<path>/annex_heatmaps.tex}")
    tex_lines_annex.append("")
    for sec in index_annex_heatmap_sections:
        tex_lines_annex.append(r"\input{" + sec + r"}")
        tex_lines_annex.append("")
    annex_heatmaps_tex.write_text("\n".join(tex_lines_annex) + "\n", encoding="utf-8")

    print(f"Done. Output: {out_root}")
    print(f"Pareto sections (main): {len(index_main_pareto_sections)}")
    print(f"Heatmap sections (annex): {len(index_annex_heatmap_sections)}")
    print(f"Main include file: {main_pareto_tex}")
    print(f"Annex include file: {annex_heatmaps_tex}")
    print(f"Best combos CSV: {global_best_csv}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
