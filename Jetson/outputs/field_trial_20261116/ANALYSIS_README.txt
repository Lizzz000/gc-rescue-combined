Static field trial analysis (offline only)

Input: vision_probe JSON with raw.field entries. Every row should retain
source_stamp_ns, receive_monotonic_s, frame_id, valid, committed, state, quality,
reason, hold_reason, measured_edges, switched, edge_states, bev_width/height,
split_x, split_measured, split_suspect, split_confidence, red_ratio, score,
jump_ratio and corners/left_region/right_region/split_line/candidate_corners.
Points are [x,y,z]; non-finite source values should be JSON null.

Usage:
  python analyze_static_field.py --input current_field_ros.json \
    --output current_field_analysis.json --frame-size 1920 1080 \
    --scene-note "Robot and camera stationary; field boundaries visible"

Metrics:
- Deduplicated valid AND committed fraction; received FPS and timing gaps.
- Continuous invalid/uncommitted runs, switches, split suspicion and diagnostics.
- Original-image point jitter around each coordinate's window median (p95/max),
  adjacent valid-frame RMS displacement and area variation.
- Diagonal-intersection projective-center jitter for outer/left/right quads,
  matching interface_pkg/app/core/projective_center.hpp semantics.
- Split position / (BEV width - 1), kept separate from source-image pixels.

No automatic accuracy verdict or changed threshold is introduced. Stable but
wrong boundaries remain possible. Inspect a synchronized raw image + overlay,
and annotate ground-truth boundaries before claiming localization accuracy.
Window duration and rates are taken from the source probe; the analyzer never
subtracts ROS source time from monotonic receive time. Missing geometry is
reported as missing; candidate/HOLD geometry never substitutes for valid output.
Do not combine camera movement, camera-coordinate changes, or multiple field
poses into one stability window. Use separate reports for each stationary pose.

Offline self-checks completed 2026-11-16:
- Known (3,4) translation gives 5 px adjacent RMS and 2.5 px median displacement.
- Projective centers and polygon areas match the synthetic rectangle.
- Duplicates do not inflate validity and invalid gaps do not bridge jitter pairs.
- Malformed/nonzero-z geometry is excluded; legacy missing fields are explicit.
- Out-of-order source timestamps do not enter adjacent-frame displacement.
- Output serializes with allow_nan=False.
