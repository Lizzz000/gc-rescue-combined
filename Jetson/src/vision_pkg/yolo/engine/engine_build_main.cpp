// Copyright 2026 jetson_yolo integration
// SPDX-License-Identifier: AGPL-3.0
// vision_engine_build 的命令行入口：从 ONNX 构建或复用本机 TensorRT 引擎缓存。
#include <iostream>
#include <stdexcept>

#include "yolo/engine/engine.hpp"

int main(int argc, char ** argv)
{
  if (argc < 2 || argc > 5) {
    std::cerr << "usage: vision_engine_build <model.onnx> [cache_dir] [optimization_level] [workspace_mb]\n";
    return 2;
  }
  BuildOptions options;
  options.precision = "fp16";
  options.workspace_mb = 1024;
  options.builder_optimization_level = 5;
  options.dla_core = -1;
  options.use_timing_cache = true;
  if (argc >= 3 && argv[2][0] != '\0') {options.cache_dir = argv[2];}
  try {
    if (argc >= 4) {
      options.builder_optimization_level = static_cast<int>(
        vision::parseBuildInteger(argv[3], 0, 5, "optimization_level"));
    }
    if (argc >= 5) {
      options.workspace_mb = vision::parseBuildInteger(
        argv[4], 1, vision::kMaxWorkspaceMiB, "workspace_mb");
    }
  } catch (const std::invalid_argument & error) {
    std::cerr << error.what() << '\n';
    return 2;
  }
  const BuildResult result = resolveEngine(argv[1], options);
  if (result.cancelled || result.engine_path.empty()) {
    std::cerr << result.error << '\n';
    return result.cancelled ? 3 : 1;
  }
  std::cout << result.engine_path.string() << '\n';
  std::cout << "precision " << result.precision_used
            << (result.fallback_applied ? " fallback" : "") << '\n';
  return 0;
}
