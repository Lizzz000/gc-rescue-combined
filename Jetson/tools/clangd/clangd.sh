#!/usr/bin/env bash
# Copyright 2026 li
# SPDX-License-Identifier: Apache-2.0
# 仅给本进程使用项目隔离的 clangd 与运行库，不改系统或业务进程的环境。
set -euo pipefail
clangd_project="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
clangd_local="$clangd_project/.deps/clangd"
clangd_binary=""
clangd_resource=""
if [[ -d "$clangd_local" ]]; then
  while IFS= read -r candidate; do
    if [[ -x "$candidate" && "$candidate" =~ /llvm-([0-9]+)/ && "${BASH_REMATCH[1]}" -ge 15 ]]; then
      clangd_binary="$candidate"
      clangd_llvm_root="${candidate%/bin/clangd}"
      for header in "$clangd_llvm_root"/lib/clang/*/include/stddef.h; do
        if [[ -f "$header" ]]; then
          clangd_resource="${header%/include/stddef.h}"
          break
        fi
      done
      break
    fi
  done < <(printf '%s\n' "$clangd_local"/usr/lib/llvm-*/bin/clangd | sort -Vr)
  if [[ -z "$clangd_binary" || -z "$clangd_resource" ]]; then
    printf '%s\n' 'Incomplete local clangd installation or missing matching resource headers.' >&2
    exit 1
  fi
  export LD_LIBRARY_PATH="$clangd_local/usr/lib/aarch64-linux-gnu${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
else
  printf '%s\n' 'clangd is unavailable; provide the documented isolated tool.' >&2
  exit 1
fi
clangd_options=()
[[ -z "$clangd_resource" ]] || clangd_options+=("--resource-dir=$clangd_resource")
clangd_override=false
clangd_query=false
for option in "$@"; do
  case "$option" in
    --compile-commands-dir|--compile-commands-dir=*) clangd_override=true ;;
    --version|--help|--help-hidden) clangd_query=true ;;
  esac
done
if [[ "$clangd_override" == false ]]; then
  if [[ "$clangd_query" == false && ! -f "$clangd_project/build/unified-clangd/compile_commands.json" &&
        ! -f "$clangd_project/build/record-clangd/compile_commands.json" ]]; then
    printf '%s\n' 'Generate this host compilation database with tools/clangd/clangd_database.py before starting clangd.' >&2
    exit 1
  fi
  # 由源码附近的 .clangd 选库；强制全局目录会盖过独立包配置。
fi
exec "$clangd_binary" "${clangd_options[@]}" "$@"
