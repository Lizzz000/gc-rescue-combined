#!/usr/bin/env python3
"""Compare actual baseline/candidate public output captures, never timing."""
import argparse
import hashlib
import json
from pathlib import Path


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--baseline", required=True, type=Path)
    parser.add_argument("--candidate", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    if args.output.exists():
        raise RuntimeError("refusing to overwrite comparison output")
    manifests = []
    summaries = []
    for root in (args.baseline, args.candidate):
        summary = json.loads((root / "summary.json").read_text(encoding="utf-8"))
        if not summary.get("complete"):
            raise RuntimeError(f"incomplete capture: {root}")
        summaries.append(summary)
        manifests.append((root / "cases.tsv").read_bytes())
    if manifests[0] != manifests[1]:
        raise RuntimeError("case manifests differ; output cases are not aligned")
    if summaries[0] != summaries[1]:
        raise RuntimeError("capture counts/options differ")
    names = [line.split("\t")[0] for line in manifests[0].decode().splitlines()[1:]]
    if len(set(names)) != len(names) or len(names) != summaries[0]["cases"]:
        raise RuntimeError("duplicate/missing cases")
    cases = []
    mismatches = []
    for name in names:
        if Path(name).name != name:
            raise RuntimeError("unsafe case path")
        before = args.baseline / name
        after = args.candidate / name
        same = before.read_bytes() == after.read_bytes()
        cases.append({"name": name, "baseline_bytes": before.stat().st_size,
                      "candidate_bytes": after.stat().st_size,
                      "baseline_sha256": sha(before), "candidate_sha256": sha(after),
                      "equal": same})
        if not same:
            mismatches.append(name)
    result = {"equal": not mismatches, "baseline": str(args.baseline.resolve()),
              "candidate": str(args.candidate.resolve()), "summary": summaries[0],
              "manifest_sha256": hashlib.sha256(manifests[0]).hexdigest(),
              "mismatches": mismatches, "cases": cases}
    with args.output.open("x", encoding="utf-8") as stream:
        json.dump(result, stream, indent=2)
        stream.write("\n")
    print(json.dumps({"equal": result["equal"], "cases": len(cases),
                      "mismatches": mismatches}))
    if mismatches:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
