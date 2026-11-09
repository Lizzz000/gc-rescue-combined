#!/usr/bin/env python3
"""Copy actual sources into a new diagnostic build tree and add timing only."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def replace_once(text, old, new, label):
    count = text.count(old)
    if count != 1:
        raise RuntimeError(f"{label}: expected one matching block, got {count}")
    return text.replace(old, new, 1)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", type=Path, required=True,
                        help="repository root with src/vision_pkg/yolo")
    parser.add_argument("--upstream", type=Path, required=True,
                        help="actual build vendor YOLOs-CPP-TensorRT-3.1.0 directory")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    source = args.source.resolve()
    upstream = args.upstream.resolve()
    output = args.output.resolve()
    if output.exists():
        raise RuntimeError(f"refusing to overwrite {output}")
    project_files = source / "src/vision_pkg/yolo"
    upstream_files = upstream / "include"
    if not (project_files / "yolo.cpp").is_file():
        raise RuntimeError("missing actual production yolo.cpp")
    if not (upstream_files / "yolos/core/trt_session_base.hpp").is_file():
        raise RuntimeError("missing actual build vendor header")
    for input_root in (project_files, upstream_files):
        if output == input_root or input_root in output.parents:
            raise RuntimeError("diagnostic output must be outside the copied source directories")
    inputs = []
    for base, prefix in ((project_files, "yolo"), (upstream_files, "upstream/include")):
        for path in sorted(base.rglob("*")):
            if path.is_file():
                inputs.append({"source": str(path), "copy": f"{prefix}/{path.relative_to(base).as_posix()}",
                               "bytes": path.stat().st_size, "sha256": sha(path)})
    output.mkdir(parents=True)
    shutil.copytree(project_files, output / "yolo")
    shutil.copytree(upstream_files, output / "upstream/include")
    here = Path(__file__).resolve().parent
    for name in ("profile_hooks.hpp", "probe.cpp", "capture.cpp", "CMakeLists.txt"):
        shutil.copyfile(here / name, output / name)
    header = output / "upstream/include/yolos/core/trt_session_base.hpp"
    original_header = header.read_text(encoding="utf-8")
    start = original_header.index("    void inferGpu(")
    end = original_header.index("    void infer(const float* blob", start)
    body = original_header[start:end]
    body = replace_once(body,
        "        std::memcpy(pinnedSrc_.data(), image.data, srcBytes);",
        "        rescue_yolo_profile::pinBegin();\n"
        "        std::memcpy(pinnedSrc_.data(), image.data, srcBytes);\n"
        "        rescue_yolo_profile::pinEnd(srcBytes);\n"
        "        rescue_yolo_profile::event(0, stream_.get());", "host staging")
    body = replace_once(body,
        "        // 2. CUDA letterbox",
        "        rescue_yolo_profile::event(1, stream_.get());\n"
        "        // 2. CUDA letterbox", "H2D event")
    body = replace_once(body,
        "        // 3. Inference",
        "        rescue_yolo_profile::event(2, stream_.get());\n"
        "        rescue_yolo_profile::graph(graphCaptured_ && graphExec_);\n"
        "        // 3. Inference", "letterbox event")
    body = replace_once(body,
        "        // 4. D2H copy",
        "        rescue_yolo_profile::event(3, stream_.get());\n"
        "        // 4. D2H copy", "TRT event")
    # This exact call appears once in inferGpu, whether the source uses its old
    # full-output loop or the candidate's bounded loop.
    body = replace_once(body,
        "                cudaMemcpyDeviceToHost, stream_.get()));",
        "                cudaMemcpyDeviceToHost, stream_.get()));\n"
        "            rescue_yolo_profile::copied(out.bytes);", "D2H bytes")
    body = replace_once(body,
        "        stream_.synchronize();",
        "        rescue_yolo_profile::event(4, stream_.get());\n"
        "        rescue_yolo_profile::syncBegin();\n"
        "        stream_.synchronize();\n"
        "        rescue_yolo_profile::syncEnd();", "original synchronization")
    header.write_text('#include "profile_hooks.hpp"\n' +
                      original_header[:start] + body + original_header[end:],
                      encoding="utf-8", newline="\n")
    implementation = output / "yolo/yolo.cpp"
    original_cpp = implementation.read_text(encoding="utf-8")
    cpp = replace_once(original_cpp,
        "  const cv::Mat packed = image.isContinuous() ? image : image.clone();",
        "  rescue_yolo_profile::packBegin();\n"
        "  const cv::Mat packed = image.isContinuous() ? image : image.clone();\n"
        "  rescue_yolo_profile::packEnd();", "non-contiguous packing")
    implementation.write_text(cpp, encoding="utf-8", newline="\n")
    identity = {
        "kind": "diagnostic-only timed copy of actual FastSegmentor",
        "production_repository": str(source), "actual_vendor": str(upstream),
        "engine_build": False,
        "limitations": [
            "Timing events add instrumentation overhead; this is not production throughput.",
            "CUDA event time includes stream scheduling delays between markers.",
            "Original final synchronization is retained; no per-stage synchronization added.",
            "CPU sync wait overlaps queued device work and must not be added to CUDA stage sum.",
            "This isolated probe excludes camera, ROS publication, composition and JPEG.",
        ],
        "inputs": inputs,
        "generated": [
            {"path": str(p.relative_to(output)), "bytes": p.stat().st_size, "sha256": sha(p)}
            for p in (header, implementation, output / "profile_hooks.hpp",
                      output / "probe.cpp", output / "capture.cpp", output / "CMakeLists.txt")
        ],
        "prepare_script": {"path": str(Path(__file__).resolve()),
                           "sha256": sha(Path(__file__).resolve())},
    }
    (output / "source-identity.json").write_text(
        json.dumps(identity, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(output)


if __name__ == "__main__":
    main()
