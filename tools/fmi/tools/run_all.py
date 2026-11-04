#!/usr/bin/env python3
"""Build and run every FIRE FMI harness; write a summary.

usage: run_all.py [PATTERN...]              (substring filters on harness names)
       run_all.py --vehicles [CONFIG...]    the generic vehicle harness (vehicle/)

Each harness runs `make run` in its directory. The result per harness is one
of: pass (simulated and every check passed), checks-failed, run-failed,
build-failed, or refused-as-expected / not-refused for EXPECT_REFUSAL
harnesses. Results go to results/summary.json and results/summary.md;
per-harness logs to results/logs/.

--vehicles flies the default mission with each fire-compose config (default:
every <library>/configs/*.toml and vehicle/test-configs/*.toml) through
`make -C vehicle VEHICLE=CONFIG run`. A vehicle that fails to build is
classified emit-failed (fire-compose) or export-failed (no FMU; the summary
quotes Rumoca's reason). Results go to results/vehicles/ with the key flight
errors of vehicle/tools/vehicle.py metrics.
Environment: JOBS (parallel runs, default 8), TMO (seconds per make, default 1800).
"""
import concurrent.futures, json, os, re, subprocess, sys
from pathlib import Path

FMI_ROOT = Path(__file__).resolve().parent.parent
HARNESSES = FMI_ROOT / "harnesses"
RESULTS = FMI_ROOT / "results"
VEHICLE = FMI_ROOT / "vehicle"
LIBRARY_ROOT = FMI_ROOT.parent.parent


def classify(directory: Path, build, run) -> str:
    expect = "EXPECT_REFUSAL" in (directory / "Makefile").read_text()
    if expect:
        return "refused-as-expected" if run.returncode == 0 else "not-refused"
    if build.returncode != 0:
        return "build-failed"
    if "checks:" not in run.stdout:
        return "run-failed"
    return "pass" if run.returncode == 0 else "checks-failed"


def make(cwd: Path, *args: str):
    """(build, run, log) of `make all` then `make run`; raises TimeoutExpired."""
    env = dict(os.environ, RAYON_NUM_THREADS="1")
    timeout = int(os.environ.get("TMO", "1800"))
    build = subprocess.run(["make", "-s", *args, "all"], cwd=cwd, capture_output=True, text=True,
                           env=env, timeout=timeout)
    run = subprocess.run(["make", "-s", *args, "run"], cwd=cwd, capture_output=True, text=True,
                         env=env, timeout=timeout) if build.returncode == 0 else build
    log = build.stdout + build.stderr + (run.stdout + run.stderr if run is not build else "")
    return build, run, log


def write_log(results: Path, name: str, log: str) -> list[str]:
    logs = results / "logs"
    logs.mkdir(parents=True, exist_ok=True)
    (logs / f"{name}.log").write_text(log)
    return [line for line in log.splitlines() if line.startswith(("FAIL", "KNOWN", "FIXED", "checks:", "== ", "pass"))]


def one(directory: Path) -> dict:
    try:
        build, run, log = make(directory)
        status = classify(directory, build, run)
    except subprocess.TimeoutExpired:
        status, log = "timeout", ""
    tail = write_log(RESULTS, directory.name, log)
    return {"harness": directory.name, "status": status, "summary": tail[-6:]}


def export_reason(log: str) -> str:
    """The first Rumoca diagnostic of a failed export, without terminal colour."""
    lines = [line.strip(" │|×") for line in re.sub(r"\x1b\[[0-9;]*m", "", log).splitlines()]
    for k, text in enumerate(lines):
        if re.search(r"unsupported|error|not supported|failed", text, re.I) and "warning" not in text:
            following = []
            for more in lines[k + 1:k + 4]:  # Rumoca wraps a diagnostic over a few lines
                if not more or more.startswith(("╭", "[", "-->", "help", "=")):
                    break
                following.append(more)
            return " ".join([text, *following])[:240]
    return ""


def one_vehicle(config: Path) -> dict:
    name, results = config.stem, RESULTS / "vehicles"
    out = VEHICLE / "out" / name
    entry = {"config": str(config.relative_to(LIBRARY_ROOT)), "harness": name}
    try:
        build, run, log = make(VEHICLE, f"VEHICLE={config}")
        if build.returncode != 0:
            if "fire-compose emit" in log:
                entry["status"] = "emit-failed"
            elif not (out / "out" / "model.fmu").exists():
                entry["status"] = "export-failed"
            else:
                entry["status"] = "build-failed"
            entry["reason"] = export_reason(log)
        else:
            entry["status"] = "run-failed" if "checks:" not in run.stdout else (
                "pass" if run.returncode == 0 else "checks-failed")
            if entry["status"] == "run-failed":
                entry["reason"] = next((line for line in log.splitlines() if "insufficient thrust" in line),
                                       export_reason(log))
    except subprocess.TimeoutExpired:
        entry["status"], log = "timeout", ""
    tail = write_log(results, name, log)
    checks = [line for line in tail if line.startswith("checks:")]
    if checks:
        entry["checks"] = checks[-1].split()[1]
        metrics = subprocess.run([sys.executable, str(VEHICLE / "tools" / "vehicle.py"), "metrics", str(out)],
                                 capture_output=True, text=True)
        if metrics.returncode == 0:
            entry["metrics"] = json.loads(metrics.stdout)
    entry["summary"] = [line for line in tail if line.startswith("FAIL")][:6]
    return entry


def metrics_text(m: dict) -> str:
    """hover altitude/position error, worst reach error, max tilt."""
    if not m:
        return ""
    hold = next((v for k, v in m.items() if k.endswith("_hold")), {})
    reach = [v for k, v in m.items() if isinstance(v, dict) and "yaw_deg" in v]
    parts = []
    if hold:
        parts.append(f"hover |dz| {hold['max_abs_z_m'] * 100:.2f} cm, xy {hold['max_xy_m'] * 100:.2f} cm, "
                     f"cmd <= {hold['max_command']:.3f}")
    if reach:
        parts.append(f"waypoints xy <= {max(r['xy_m'] for r in reach) * 100:.1f} cm, "
                     f"|dz| <= {max(abs(r['z_m']) for r in reach) * 100:.2f} cm, "
                     f"|dyaw| <= {max(abs(r['yaw_deg']) for r in reach):.2f} deg")
    parts.append(f"tilt <= {m['max_tilt_deg']:.1f} deg")
    return "; ".join(parts)


def vehicles(arguments: list[str]) -> int:
    configs = [Path(a).resolve() if Path(a).exists() else (LIBRARY_ROOT / a).resolve() for a in arguments] or \
        sorted((LIBRARY_ROOT / "configs").glob("*.toml")) + sorted((VEHICLE / "test-configs").glob("*.toml"))
    with concurrent.futures.ThreadPoolExecutor(int(os.environ.get("JOBS", "8"))) as pool:
        results = list(pool.map(one_vehicle, configs))
    out = RESULTS / "vehicles"
    out.mkdir(parents=True, exist_ok=True)
    if arguments and (out / "summary.json").exists():  # a subset: update those entries only
        ran = {r["config"] for r in results}
        results = sorted([r for r in json.loads((out / "summary.json").read_text()) if r["config"] not in ran]
                         + results, key=lambda r: r["config"])
    (out / "summary.json").write_text(json.dumps(results, indent=1))
    counts: dict[str, int] = {}
    for r in results:
        counts[r["status"]] = counts.get(r["status"], 0) + 1
    lines = ["| config | status | checks | key errors / reason |", "|---|---|---|---|"]
    lines += [f"| {r['config']} | {r['status']} | {r.get('checks', '')} | "
              f"{metrics_text(r.get('metrics', {})) or r.get('reason', '')} |" for r in results]
    (out / "summary.md").write_text("\n".join(lines) + "\n\n" + json.dumps(counts) + "\n")
    for r in results:
        print(f"{r['status']:15} {r['config']:40} {r.get('checks', ''):8} "
              f"{metrics_text(r.get('metrics', {})) or r.get('reason', '')}")
    print(json.dumps(counts))
    return 0 if counts.get("pass", 0) == len(results) else 1


def main() -> int:
    if sys.argv[1:2] == ["--vehicles"]:
        return vehicles(sys.argv[2:])
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
