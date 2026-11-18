"""Compare actual STM32 algorithm C on the host against a Git baseline; no board access.

Repeated alternating runs reduce order bias. Timings are PC nanoseconds/call, not
STM32 WCET, and do not establish control accuracy (run_native.py covers behavior).
"""
import argparse
import json
from pathlib import Path
import shutil
import statistics
import subprocess
import tempfile

ROOT=Path(__file__).resolve().parents[2]
CORE = Path("STM32/project/Core")
# 这些入口跨版本保持兼容；支持 Kalman → Vision 与里程计组合器迁入 Control。
REQUIRED_UNITS = ("AHRS", "MahonyAHRS", "pid", "adaptive_kalman", "motor_chassis", "chassis_odometry")
OPTIONAL_UNITS = ("heading_eskf", "wheel_alignment", "planar_odometry")


def benchmark_sources(directory, revision=None):
    """Resolve sources and headers from the same tree so a baseline cannot inherit current definitions."""
    locations = (CORE / "Algorithm", CORE / "Control/Chassis")
    if revision:
        names = subprocess.check_output(["git", "ls-tree", "-r", "--name-only", revision, "--",
                                         *(p.as_posix() for p in locations)], cwd=ROOT, text=True).splitlines()
        files = [Path(name) for name in names]
    else:
        files = [p.relative_to(ROOT) for location in locations for p in (ROOT / location).rglob("*")]

    sources = [p for p in files if p.suffix == ".c" and p.stem in REQUIRED_UNITS + OPTIONAL_UNITS]
    for unit in REQUIRED_UNITS:
        if sum(p.stem == unit for p in sources) != 1:
            raise RuntimeError(f"{revision or 'working tree'}: expected one {unit}.c for this benchmark")
    folders = sorted({p.parent for p in sources})
    if revision:
        headers = [p for p in files if p.suffix == ".h" and p.parent in folders]
        for p in sources + headers:
            target = directory / p
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes(subprocess.check_output(["git", "show", f"{revision}:{p.as_posix()}"], cwd=ROOT))
    return [str(directory / p) for p in sources], ["-I" + str(directory / p) for p in folders]

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--baseline",default="HEAD")
    p.add_argument("--output",type=Path,required=True)
    p.add_argument("--cc", default=shutil.which("gcc") or
                   "C:/Program Files/JetBrains/CLion 2026.2.3.1/bin/mingw/bin/gcc.exe")
    args=p.parse_args()
    base=subprocess.check_output(["git","rev-parse",args.baseline],cwd=ROOT,text=True).strip()
    collected={"before":{},"after":{}}
    with tempfile.TemporaryDirectory(prefix="stm32_algorithm_bench_") as tmp:
        tmp=Path(tmp)
        executables={}
        for name, sources_and_includes in (("before", benchmark_sources(tmp/"baseline", base)),
                                           ("after", benchmark_sources(ROOT))):
            exe=tmp/(name+".exe")
            cmd=[args.cc,"-O2","-std=c11","-Wall","-Wextra","-Werror"]
            sources, includes = sources_and_includes
            cmd += includes + [str(ROOT/"STM32/tests/native/benchmark_algorithms.c")] + sources
            subprocess.run(cmd+["-lm","-o",str(exe)],check=True)
            executables[name]=exe
        for repeat in range(7):
            for name in (("before","after") if repeat%2==0 else ("after","before")):
                output=subprocess.check_output([str(executables[name])],text=True)
                for line in output.splitlines():
                    key,value=line.split();collected[name].setdefault(key,[]).append(float(value))
    report={"baseline":base,"host_only":True,"compiler":args.cc,"optimization":"-O2, no fast-math",
            "iterations_per_run":1000000,"runs":7,"results":{}}
    for key in collected["before"]:
        before=statistics.median(collected["before"][key]);after=statistics.median(collected["after"][key])
        report["results"][key]={"before_median":before,"after_median":after,"ratio":after/before,
                                "before_runs":collected["before"][key],"after_runs":collected["after"][key]}
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(report,indent=2),encoding="utf-8")
    print(json.dumps(report,indent=2))

if __name__=="__main__":main()
