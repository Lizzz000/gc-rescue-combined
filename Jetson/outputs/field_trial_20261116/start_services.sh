#!/usr/bin/env bash
set -euo pipefail
ROOT=/home/nvidia/WorkSpace/GC_Rescue/Jetson
CONFIG="$ROOT/outputs/field_trial_20261116/field_rescue.yaml"
MODE="${1:---vision-only}"
case "$MODE" in --all|--vision-only) ;; *) echo 'Usage: start_services.sh [--all|--vision-only]' >&2; exit 2;; esac
test -f "$ROOT/install/setup.bash"
test -f "$CONFIG"
if [[ "$MODE" == --all ]]; then
python3 - "$CONFIG" <<'PY'
import sys, pathlib, yaml
c=yaml.safe_load(pathlib.Path(sys.argv[1]).read_text())
p=pathlib.Path(c['link']['device'])
if not p.exists():
    raise SystemExit('Configured STM32 USB is absent: '+str(p))
PY
fi
if ! systemctl --user is-active --quiet gc-preflight-vision; then
  systemd-run --user --unit=gc-preflight-vision --collect \
    --property="WorkingDirectory=$ROOT" --setenv=PYTHONNOUSERSITE=1 \
    --setenv="RESCUE_RUN_DIR=$ROOT/outputs/field_trial_20261116/vision_run" \
    /bin/bash -c 'source /opt/ros/humble/setup.bash; source "$1/install/setup.bash"; export PATH=/usr/local/cuda/bin:$PATH; exec ros2 launch bringup_pkg rescue.launch.py run:=vision config:="$2"' _ "$ROOT" "$CONFIG"
  # This camera resets effective ISP state when streaming starts despite unchanged
  # control readback. Apply the measured profile only after a new image exists.
  if ! python3 "$ROOT/outputs/field_trial_20261116/wait_camera_frame.py"; then
    systemctl --user stop gc-preflight-vision
    exit 1
  fi
  CAMERA_LOG="$ROOT/outputs/field_trial_20261116/camera_apply_$(date +%Y%m%d_%H%M%S)_$$"
  if ! python3 "$ROOT/outputs/field_trial_20261116/apply_camera_profile.py" \
    --profile "$ROOT/outputs/field_trial_20261116/camera_profile.json" --output "$CAMERA_LOG"; then
    systemctl --user stop gc-preflight-vision
    exit 1
  fi
fi
if [[ "$MODE" == --all ]] && ! systemctl --user is-active --quiet gc-preflight-manual; then
  systemd-run --user --unit=gc-preflight-manual --collect \
    --property="WorkingDirectory=$ROOT" --setenv=PYTHONNOUSERSITE=1 \
    /bin/bash -c 'source /opt/ros/humble/setup.bash; source "$1/install/setup.bash"; exec python3 "$1/src/teleop_pkg/manual_control.py" --config "$2" --arm-zero-s 0.6 --max-linear-mps 0.25 --max-angular-radps 0.5 --max-timed-s 1.5' _ "$ROOT" "$CONFIG"
fi
systemctl --user is-active gc-preflight-vision
if [[ "$MODE" == --all ]]; then
  systemctl --user is-active gc-preflight-manual
fi
