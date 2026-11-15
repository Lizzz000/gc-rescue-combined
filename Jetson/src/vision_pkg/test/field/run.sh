#!/usr/bin/env bash
# PC/板端不经 colcon 直接编译并运行安全区测试；需要 g++ 与 OpenCV（pkg-config opencv4）。
# 用法：run.sh [all|matrix|sequence|replay|colour|centers|truth|split|geometry|corners] [测试参数...]
# corners 只需 g++，不需要 OpenCV、Python 或生成配置。
# 产物放在 ${TMPDIR:-/tmp}/gc_rescue_field_tests；退出码 77 表示跳过（回放无数据）。
set -eo pipefail
here="$(cd "$(dirname "$0")" && pwd)"
package="$(cd "$here/../.." && pwd)"
out="${TMPDIR:-/tmp}/gc_rescue_field_tests"
which="${1:-all}"
[ $# -gt 0 ] && shift
mkdir -p "$out"
cxx="${CXX:-g++}"
status=0
if [ "$which" = all ] || [ "$which" = corners ]; then
  "$cxx" -std=c++17 -O2 -Wall -Wextra -I"$package" -I"$here/../host" \
    "$here/test_corner_order.cpp" -o "$out/test_corner_order"
  set +e
  "$out/test_corner_order" "$@"
  code=$?
  set -e
  [ "$which" = corners ] && exit "$code"
  if [ "$code" -ne 0 ]; then status=1; fi
fi
mkdir -p "$out/obj"
python3 "$here/field_config.py" "$out/field_config_values.inc"
flags=(-std=c++17 -O2 -Wall -Wextra -DFIELD_DATA_DIR="\"$here/data\"" -I"$here" -I"$here/../host" -I"$package" -I"$out" $(pkg-config --cflags opencv4))
libs=($(pkg-config --libs opencv4))

# vision_field 纯核心编成静态库（与板端 CMake 的 vision_field 同一组源文件）
objects=()
for src in "$package"/field/*.cpp "$package"/field/opencv/*.cpp; do
  obj="$out/obj/$(basename "$(dirname "$src")")_$(basename "$src" .cpp).o"
  if [ ! -f "$obj" ] || [ "$src" -nt "$obj" ] || [ "$out/field_config_values.inc" -nt "$obj" ]; then
    "$cxx" "${flags[@]}" -c "$src" -o "$obj"
  fi
  objects+=("$obj")
done
rm -f "$out/libvision_field.a"
ar rcs "$out/libvision_field.a" "${objects[@]}"

build() {
  extra=()
  if [ "$1" = colour ]; then extra+=("$here/../host/test_colour.cpp"); fi
  "$cxx" "${flags[@]}" "$here/test_field_$1.cpp" "${extra[@]}" "$out/libvision_field.a" "${libs[@]}" -o "$out/test_field_$1"
}

for name in matrix sequence replay colour centers truth split geometry; do
  [ "$which" = all ] || [ "$which" = "$name" ] || continue
  build "$name"
  start=$(date +%s%3N)
  set +e
  "$out/test_field_$name" "$@"
  code=$?
  set -e
  echo "test_field_$name exit=$code elapsed_ms=$(( $(date +%s%3N) - start ))"
  if [ $code -ne 0 ] && [ $code -ne 77 ]; then status=1; fi
done
exit $status
