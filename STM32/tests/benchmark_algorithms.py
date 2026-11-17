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
SOURCES=["AHRS/AHRS.c","AHRS/MahonyAHRS.c","PID/pid.c","Kalman/adaptive_kalman.c",
         "Chassis/motor_chassis.c","Chassis/chassis_odometry.c"]
HEADERS=["AHRS/AHRS.h","AHRS/MahonyAHRS.h","PID/pid.h","Kalman/adaptive_kalman.h",
         "Chassis/motor_chassis.h","Chassis/chassis_odometry.h"]

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--baseline",default="HEAD")
    p.add_argument("--output",type=Path,required=True)
    p.add_argument("--cc",default="C:/Program Files/JetBrains/CLion 2026.2.3.1/bin/mingw/bin/gcc.exe")
    args=p.parse_args()
    base=subprocess.check_output(["git","rev-parse",args.baseline],cwd=ROOT,text=True).strip()
    source_root=ROOT/"STM32/project/Core/Algorithm"
    collected={"before":{},"after":{}}
    with tempfile.TemporaryDirectory(prefix="stm32_algorithm_bench_") as tmp:
        tmp=Path(tmp)
        copies=tmp/"baseline";copies.mkdir()
        optional=[]
        for path in ("Chassis/heading_eskf.c","Chassis/heading_eskf.h"):
            exists=subprocess.run(["git","cat-file","-e",f"{base}:STM32/project/Core/Algorithm/{path}"],cwd=ROOT,capture_output=True)
            if exists.returncode==0:optional.append(path)
        for path in SOURCES+HEADERS+optional:
            dest=copies/path;dest.parent.mkdir(parents=True,exist_ok=True)
            dest.write_bytes(subprocess.check_output(["git","show",f"{base}:STM32/project/Core/Algorithm/{path}"],cwd=ROOT))
        executables={}
        for name,root in (("before",copies),("after",source_root)):
            exe=tmp/(name+".exe")
            cmd=[args.cc,"-O2","-std=c11","-Wall","-Wextra","-Werror"]
            cmd += ["-I"+str(root/folder) for folder in ("AHRS","PID","Kalman","Chassis")]
            cmd += [str(ROOT/"STM32/tests/native/benchmark_algorithms.c")]+[str(root/file) for file in SOURCES]
            if (root/"Chassis/heading_eskf.c").exists():
                cmd.append(str(root/"Chassis/heading_eskf.c"))
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
