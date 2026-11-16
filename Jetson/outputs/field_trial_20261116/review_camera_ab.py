"""Offline camera trial audit; reads existing artifacts, never contacts devices."""
import hashlib
import json
from pathlib import Path

import numpy as np
from PIL import Image


BASE = Path(__file__).resolve().parent
ROIS = {
    "neutral_floor": [500, 400, 800, 600],
    "left_red_interior": [700, 80, 900, 130],
    "right_red_interior": [1250, 100, 1450, 140],
    "near_purple_wall": [900, 190, 1000, 220],
}


def read(path):
    return json.loads(path.read_text(encoding="utf-8-sig"))


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def image_stats(path):
    rgb = np.asarray(Image.open(path).convert("RGB"), dtype=np.float64)
    result = {"path": str(path.relative_to(BASE)), "sha256": digest(path),
              "width": rgb.shape[1], "height": rgb.shape[0], "rois": {}}
    for name, (x0, y0, x1, y1) in ROIS.items():
        r = rgb[y0:y1, x0:x1]
        assert r.shape[:2] == (y1-y0, x1-x0)
        luma = r @ np.array([0.299, 0.587, 0.114])
        hi, lo = r.max(axis=2), r.min(axis=2)
        saturation = np.divide(hi-lo, hi, out=np.zeros_like(hi), where=hi != 0)
        result["rois"][name] = {
            "rgb_mean": r.mean(axis=(0, 1)).tolist(),
            "luma_8bit_mean": float(luma.mean()),
            "luma_8bit_median": float(np.median(luma)),
            "luma_8bit_p95": float(np.quantile(luma, 0.95)),
            "hsv_saturation_0to1_mean": float(saturation.mean()),
            "hsv_saturation_0to1_median": float(np.median(saturation)),
            "any_channel_ge250_fraction": float((hi >= 250).mean()),
            "all_channels_ge250_fraction": float((lo >= 250).mean()),
        }
    return result


def review_run(name):
    summary = read(BASE / name / "summary.json")
    comparison_path = BASE / (name + "_comparison.json")
    comparison = read(comparison_path)
    result = {"comparison_sha256": digest(comparison_path),
              "seconds_per_trial": summary["duration_s"],
              "restore": summary["restore"], "trials": []}
    for item in comparison["trials"]:
        trial_name = item["trial"]
        plan = next(t for t in summary["trials"] if t["name"] == trial_name)
        expected = dict(summary["baseline"])
        expected.update(plan["changes_from_baseline"])
        controls = {}
        for phase in ["after", "end"]:
            control_file = BASE / name / trial_name / ("controls_"+phase+".json")
            data = read(control_file)
            actual = data["controls"]
            controlled_mismatches, inactive_differences = {}, {}
            for alias, key in summary["control_names"].items():
                value = actual[key]["value"]
                if value != expected[alias]:
                    target = inactive_differences if actual[key]["inactive"] else controlled_mismatches
                    target[alias] = {"expected": expected[alias], "readback": value}
            controls[phase] = {
                "returncode": data["command"]["returncode"],
                "controlled_mismatches": controlled_mismatches,
                "inactive_readback_differences": inactive_differences,
                "values": {alias: actual[key]["value"] for alias, key in summary["control_names"].items()},
                "unsupported_privacy_read": "error 32 getting ext_ctrl Privacy" in data["command"]["stdout"],
            }
        geometry = item["geometry_by_frame_id"]["camera"]
        result["trials"].append({
            "name": trial_name, "changes_from_baseline": plan["changes_from_baseline"],
            "frames": item["counts"]["unique_source_frames"],
            "valid_fraction": item["valid_and_committed_fraction"],
            "received_fps": item["unique_received_fps"],
            "corner_order": geometry["corner_order"],
            "corner_median_xy_px": [c["median_xy_px"] for c in geometry["corners"]],
            "corner_p95_distance_from_window_median_px": [c["distance_from_median_px"]["p95"] for c in geometry["corners"]],
            "corner_max_distance_from_window_median_px": [c["distance_from_median_px"]["max"] for c in geometry["corners"]],
            "corner_adjacent_rms_px": geometry["corner_adjacent_rms_px"],
            "center_p95_distance_from_window_median_px": {
                region: data["projective_center"]["distance_from_median_px"]["p95"]
                for region, data in geometry["region_centers"].items()},
            "controls": controls,
        })
    baseline = next(t for t in result["trials"] if t["name"] == "baseline")
    saturation = next(t for t in result["trials"] if t["name"] == "saturation90")
    result["saturation90_vs_baseline"] = {
        "bl_p95_reduction_fraction": 1 - saturation["corner_p95_distance_from_window_median_px"][3] / baseline["corner_p95_distance_from_window_median_px"][3],
        "br_p95_reduction_fraction": 1 - saturation["corner_p95_distance_from_window_median_px"][2] / baseline["corner_p95_distance_from_window_median_px"][2],
        "fps_ratio": saturation["received_fps"] / baseline["received_fps"],
        "median_corner_position_difference_px": [
            float(np.linalg.norm(np.asarray(a)-np.asarray(b)))
            for a, b in zip(saturation["corner_median_xy_px"], baseline["corner_median_xy_px"])],
    }
    return result


def main():
    runs = {name: review_run(name) for name in ["camera_ab_01", "camera_ab_02"]}
    images = {"before_control_rewrite": image_stats(BASE / "camera_before.png")}
    for run in runs:
        for trial in runs[run]["trials"]:
            key = run + "/" + trial["name"]
            images[key] = image_stats(BASE / run / trial["name"] / "matched/raw.png")
    before = images["before_control_rewrite"]["rois"]
    baseline = images["camera_ab_01/baseline"]["rois"]
    report = {
        "schema": 1,
        "runs": runs,
        "image_method": {
            "rois_xyxy_exclusive": ROIS,
            "luma": "0.299 R + 0.587 G + 0.114 B on stored 8-bit RGB; display-brightness proxy, not calibrated radiance/exposure",
            "saturation": "HSV S = (max(R,G,B)-min(R,G,B))/max(R,G,B), zero for black; no OpenCV dependence",
            "sampling": "One saved raw image per trial, captured after its geometry sampling window; fixed pixel ROIs, no geometric registration",
        },
        "images": images,
        "before_to_rewritten_baseline_luma_mean_ratio": {
            key: baseline[key]["luma_8bit_mean"] / before[key]["luma_8bit_mean"] for key in ROIS},
        "assessment": [
            "Both trial orders support lower near-corner temporal scatter at saturation 90 in this static view; corner p95 is relative to each window median, not ground-truth error.",
            "All eight 12-second trials and both 30-second repeats report 100% valid and committed; validity does not establish absolute geometry accuracy.",
            "The large brightness change occurred before the first rewritten baseline. Its nominal controls match the pre-run controls, but the script disabled AF/AWB, rewrote the full control set, then restored automatic modes. Do not attribute this global improvement to saturation or field dimensions.",
            "Active requested controls match in after/end readbacks. AF-enabled trials show varying inactive focus_absolute readbacks; this is not a controlled fixed-focus experiment and not evidence that the inactive value was enforced.",
            "Automatic white balance remains enabled except for its dedicated trial. Nominal readback/restoration does not establish identical hidden ISP/automatic state.",
            "No motion, varied distance, illumination, object-color segmentation, or calibrated physical ground truth is evaluated here. The results justify a local camera-setting candidate, not a guarantee of operational accuracy.",
            "Privacy control enumeration prints error 32 while v4l2-ctl returns zero; requested camera controls were parsed and matched, and Privacy was not modified.",
        ],
    }
    out = BASE / "camera_ab_independent_review.json"
    out.write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False)+"\n", encoding="utf-8")
    print(out)
    print(json.dumps(report["before_to_rewritten_baseline_luma_mean_ratio"]))
    for key in ["before_control_rewrite", "camera_ab_01/baseline", "camera_ab_01/saturation90", "camera_ab_02/saturation90", "camera_ab_02/baseline"]:
        print(key, json.dumps({roi: {"Y": val["luma_8bit_mean"], "S": val["hsv_saturation_0to1_mean"]} for roi, val in images[key]["rois"].items()}))


if __name__ == "__main__":
    main()
