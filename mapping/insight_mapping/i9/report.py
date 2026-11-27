"""Stage 6 - reconstruction quality report (JSON + Markdown). Every statement carries its evidence class and what is NOT proven."""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np


def _fmt(x, nd=3):
    return "n/a" if x is None else (f"{x:.{nd}f}" if isinstance(x, float) else str(x))


def grade(rep: dict) -> dict:
    """Traffic-light grading per aspect with explicit thresholds (first-pass, to be tuned on real data)."""
    g: dict = {}
    vo = rep.get("vo", {})
    cal = rep.get("calibration", {}).get("verdict", {})
    g["calibration_metric"] = cal.get("calibration_ok_for_metric_reconstruction", "n/a")
    if vo:
        lf = vo["lost_fraction"]
        g["tracking"] = "GOOD" if lf < 0.002 else ("OK" if lf < 0.02 else "POOR")
        rms = vo["reproj_rms_px"]["median"]
        g["reprojection"] = "GOOD" if rms < 0.8 else ("OK" if rms < 1.5 else "POOR")
    c = rep.get("depth_consistency", {})
    if c.get("pairs"):
        e = c["median_rel_depth_err"]
        g["multiview_depth_consistency"] = "GOOD" if e < 0.01 else ("OK" if e < 0.03 else "POOR")
    pl = rep.get("planarity", {}).get("planes") or []
    if pl:
        r = pl[0]["rms_mm"]
        g["dominant_plane_noise"] = "GOOD" if r < 8 else ("OK" if r < 20 else "POOR")
    d = rep.get("dense", {})
    if d:
        v = d["valid_depth_fraction"]["median"]
        g["depth_coverage"] = "GOOD" if v > 0.5 else ("OK" if v > 0.25 else "POOR")
    return g


def write(out_dir, rep: dict):
    out = Path(out_dir)
    rep["grades"] = grade(rep)
    (out / "reconstruction_report.json").write_text(json.dumps(rep, indent=1, default=float))
    L = []
    a = L.append
    a(f"# Stereo reconstruction report\n")
    a(f"* input: `{rep['input']}`  ({rep['scan']['storage']})")
    a(f"* data origin: **{rep.get('data_origin', 'UNSPECIFIED')}**")
    a(f"* tool: insight_mapping.i9, contract: capture/DATA_CONTRACT.md v0.3 (fw 2.1.5), geometric inputs: stereo + stereo CameraInfo only\n")
    a("## 1. Import / topics")
    a("| role | topic | type | n | delivered Hz | count loss % | frame_id |\n|---|---|---|---|---|---|---|")
    for t, e in rep["scan"]["topics"].items():
        a(f"| {e['role'] or '-'} | `{t}` | {e['type'].split('/')[-1]} | {e.get('count', e['bag_messages'])} | {_fmt(e.get('delivered_hz'), 2)} | {_fmt(e.get('count_loss_pct'), 2)} | {','.join(e['frame_ids'])} |")
    a("\nFindings:")
    for f in rep["scan"]["findings"]:
        a(f"* [{f['severity']}] {f['message']}")
    cal = rep.get("calibration")
    if cal:
        a("\n## 2. Stereo sync, CameraInfo, calibration")
        s = cal["sync"]
        a(f"* L/R: {s['left']} / {s['right']} frames, {s['exact_stamp_pairs']} identical-stamp pairs, max skew {_fmt(s['max_skew_ms'], 3)} ms")
        ci = cal["camera_info"]
        a(f"* K(left) = {ci['left']['K'][0]:.3f}, {ci['left']['K'][2]:.3f}, {ci['left']['K'][5]:.3f} (fx, cx, cy); baseline from P = {_fmt(ci['baseline_from_P_m'], 5)} m; from TF = {_fmt(ci['baseline_from_tf_m'], 5)} m; |Δ| = {_fmt(ci.get('baseline_P_vs_tf_diff_mm'), 3)} mm")
        e = cal.get("epipolar", {})
        if e.get("dy_px"):
            a(f"* epipolar check ({e['matches_used']} matches, {e['pairs_sampled']} pairs): dy median {e['dy_px']['median']:.3f} px, σ {e['dy_px']['mad_sigma']:.3f} px; "
              f"x-drift {e['dy_model_px']['edge_drift_x_px']:.2f} px, y-drift {e['dy_model_px']['edge_drift_y_px']:.2f} px; positive disparity {e['disparity_px']['positive_fraction']:.1%}; "
              f"disparity median {e['disparity_px']['median']:.1f} px (p95 {e['disparity_px']['p95']:.1f})")
        dc = cal.get("depth_stream_cross_check")
        if dc and dc.get("frames_compared"):
            a(f"* Depth-stream cross-check (HYPOTHESIS ONLY, unit unverified): median depth/stereo(m) = {dc['median_ratio_depth_over_stereo_m']:.1f} (min {dc['min']:.1f}, max {dc['max']:.1f})")
        th = cal.get("depth_uncertainty_theory")
        if th:
            a("* theoretical σ_Z at 0.25 px: " + ", ".join(f"{z} m → {v['sigma_z_mm_at_0.25px']:.1f} mm" for z, v in th.items() if isinstance(v, dict)))
        v = cal["verdict"]
        a(f"\n**Metric reconstruction justified by calibration: {v['calibration_ok_for_metric_reconstruction']}**")
        for m in v["errors"] + v["warnings"]:
            a(f"* {m}")
        for m in v["caveats"]:
            a(f"* caveat: {m}")
    vo = rep.get("vo")
    if vo:
        a("\n## 3. Stereo visual odometry")
        a(f"* {vo['frames']} frames, {vo['keyframes']} keyframes, lost {vo['lost_frames']} ({vo['lost_fraction']:.2%}), {vo['fps']:.1f} fps offline")
        a(f"* path length {vo['path_length_m']:.2f} m (metric by vendor baseline), end-to-start distance {vo['end_to_start_m']:.2f} m, max step {vo['max_step_m']:.3f} m")
        a(f"* inliers median {vo['inliers']['median']:.0f} (p5 {vo['inliers']['p5']:.0f}); reprojection RMS median {vo['reproj_rms_px']['median']:.2f} px (p95 {vo['reproj_rms_px']['p95']:.2f})")
    ref = rep.get("vs_reference")
    if ref:
        a(f"* vs reference ({ref['name']}): {json.dumps(ref['metrics'], default=float)}")
    d = rep.get("dense")
    if d:
        a("\n## 4. Dense reconstruction")
        a(f"* {d['frames_used']} frames fused; map {d['points']} points ({d['points_before_outlier_filter']} before outlier filter), mesh {d['mesh_vertices']} vertices / {d['mesh_triangles']} triangles")
        a(f"* extent (m): {[round(x, 2) for x in d['map_extent_m']]}; valid depth/frame median {d['valid_depth_fraction']['median']:.1%}; colour: {d['colour']}")
        a(f"* time: {json.dumps({k: round(v, 1) for k, v in d['timings_s'].items()})}; GPU MiB {d['gpu_mem_used_mib']}")
    dc = rep.get("depth_consistency")
    if dc and dc.get("pairs"):
        a(f"* multi-view depth consistency (gap {dc['gap_frames']} frames, {dc['pairs']} pairs): median relative error {dc['median_rel_depth_err']:.2%}, p90 of pair medians {dc['p90_of_pair_medians']:.2%}")
    pl = rep.get("planarity", {}).get("planes")
    if pl:
        a("* dominant planes (RMS to plane): " + "; ".join(f"{p['rms_mm']:.1f} mm ({p['fraction_of_map']:.0%} of map)" for p in pl))
    a("\n## 5. Grades (first-pass thresholds)")
    for k, v in rep["grades"].items():
        a(f"* {k}: **{v}**")
    a("\n## 6. Not covered / not proven")
    for m in rep.get("limitations", []):
        a(f"* {m}")
    (out / "reconstruction_report.md").write_text("\n".join(L) + "\n")
