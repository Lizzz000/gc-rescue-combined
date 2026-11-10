#!/usr/bin/env bash
# 本机/板端 g++ 编译 field 纯核心并跑环境鲁棒性回归；需要 OpenCV（pkg-config opencv4）。
# 用法：run.sh [--report]。产物放在 ${TMPDIR:-/tmp}/gc_rescue_host/vision_pkg。
set -euo pipefail
here="$(cd "$(dirname "$0")" && pwd)"
package="$(cd "$here/../.." && pwd)"
out="${TMPDIR:-/tmp}/gc_rescue_host/vision_pkg"
mkdir -p "$out"
python3 "$here/field_config.py" "$out/field_config_values.inc"
start=$(date +%s%3N)
${CXX:-g++} -std=c++17 -O2 -Wall -Wextra -I"$here" -I"$package" -I"$out" \
  $(pkg-config --cflags opencv4) \
  "$here/test_main.cpp" "$here/test_colour.cpp" "$here/test_field.cpp" \
  "$package"/field/*.cpp "$package"/field/opencv/*.cpp \
  $(pkg-config --libs opencv4) -o "$out/field_host_tests"
"$out/field_host_tests" "$@"
echo "elapsed_ms $(( $(date +%s%3N) - start ))"
