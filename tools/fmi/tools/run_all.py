#!/usr/bin/env python3
"""Build and run every FIRE FMI harness; write a summary.

usage: run_all.py [PATTERN...]   (substring filters on harness names)

Each harness runs `make run` in its directory. The result per harness is one
of: pass (simulated and every check passed), checks-failed, run-failed,
build-failed, or refused-as-expected / not-refused for EXPECT_REFUSAL
harnesses. Results go to results/summary.json and results/summary.md;
per-harness logs to results/logs/.
"""
import concurrent.futures, json, os, subprocess, sys
from pathlib import Path

FMI_ROOT = Path(__file__).resolve().parent.parent
HARNESSES = FMI_ROOT / "harnesses"
RESULTS = FMI_ROOT / "results"


def classify(directory: Path, build, run) -> str:
    expect = "EXPECT_REFUSAL" in (directory / "Makefile").read_text()
    if expect:
        return "refused-as-expected" if run.returncode == 0 else "not-refused"
    if build.returncode != 0:
        return "build-failed"
    if "checks:" not in run.stdout:
        return "run-failed"
    return "pass" if run.returncode == 0 else "checks-failed"


def one(directory: Path) -> dict:
    env = dict(os.environ, RAYON_NUM_THREADS="1")
    timeout = int(os.environ.get("TMO", "1800"))
    try:
        build = subprocess.run(["make", "-s", "all"], cwd=directory, capture_output=True, text=True,
                               env=env, timeout=timeout)
        run = subprocess.run(["make", "-s", "run"], cwd=directory, capture_output=True, text=True,
                             env=env, timeout=timeout) if build.returncode == 0 else build
        status = classify(directory, build, run)
        log = build.stdout + build.stderr + (run.stdout + run.stderr if run is not build else "")
    except subprocess.TimeoutExpired:
        status, log = "timeout", ""
    logs = RESULTS / "logs"
    logs.mkdir(parents=True, exist_ok=True)
    (logs / f"{directory.name}.log").write_text(log)
    tail = [line for line in log.splitlines() if line.startswith(("FAIL", "checks:", "== ", "pass"))]
    return {"harness": directory.name, "status": status, "summary": tail[-6:]}


def main() -> int:
    patterns = sys.argv[1:]
    directories = sorted(d for d in HARNESSES.iterdir()
                         if (d / "Makefile").exists() and (not patterns or any(p in d.name for p in patterns)))
    with concurrent.futures.ThreadPoolExecutor(int(os.environ.get("JOBS", "8"))) as pool:
        results = list(pool.map(one, directories))
    RESULTS.mkdir(exist_ok=True)
    (RESULTS / "summary.json").write_text(json.dumps(results, indent=1))
    counts: dict[str, int] = {}
    for r in results:
        counts[r["status"]] = counts.get(r["status"], 0) + 1
    lines = ["| harness | status |", "|---|---|"] + [f"| {r['harness']} | {r['status']} |" for r in results]
    (RESULTS / "summary.md").write_text("\n".join(lines) + "\n\n" + json.dumps(counts) + "\n")
    for r in results:
        if r["status"] not in ("pass", "refused-as-expected"):
            print(f"{r['status']:20} {r['harness']}")
    print(json.dumps(counts))
    ok = sum(counts.get(k, 0) for k in ("pass", "refused-as-expected"))
    print(f"{ok} / {len(results)} harnesses pass")
    return 0 if ok == len(results) else 1


if __name__ == "__main__":
    sys.exit(main())
