#!/usr/bin/env python3
"""Export every library model to an FMU with one Rumoca binary.

usage: sweep.py OUTDIR [extra rumoca args]

Environment: TARGET (default fmi3), ONLY ("A|B" substring filter), JOBS, TMO
(seconds per model), plus the settings of tools/fire_compile.py. Writes
OUTDIR/status.jsonl and prints the models that failed.
"""
import concurrent.futures, json, os, subprocess, sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent / "tools"))
import fire_compile  # noqa: E402


def one(entry, outdir: Path, extra) -> dict:
    short = entry["model"].removeprefix(fire_compile.PREFIX)
    work = outdir / short
    work.mkdir(exist_ok=True)
    command = fire_compile.compile_command(fire_compile.LIBRARY_ROOT / entry["file"], entry["model"],
                                           "--target", os.environ.get("TARGET", "fmi3"), *extra)
    try:
        p = subprocess.run(command, capture_output=True, text=True, cwd=work,
                           timeout=int(os.environ.get("TMO", "900")), env=dict(os.environ, RAYON_NUM_THREADS="1"))
        fmus = sorted(str(f) for f in work.rglob("*.fmu"))
        status = "ok" if p.returncode == 0 and fmus else "failed"
        error = "" if status == "ok" else "\n".join(l for l in p.stderr.splitlines() if "warning[" not in l)[-1500:]
    except subprocess.TimeoutExpired:
        status, error, fmus = "timeout", "", []
    return {"model": short, "status": status, "fmus": fmus, "stderr": error}


def main() -> int:
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    outdir, extra = Path(sys.argv[1]).resolve(), sys.argv[2:]
    outdir.mkdir(parents=True, exist_ok=True)
    models = fire_compile.models()
    if only := os.environ.get("ONLY"):
        models = [m for m in models if any(o in m["model"] for o in only.split("|"))]
    with concurrent.futures.ThreadPoolExecutor(int(os.environ.get("JOBS", "12"))) as pool:
        results = sorted(pool.map(lambda e: one(e, outdir, extra), models), key=lambda r: r["model"])
    (outdir / "status.jsonl").write_text("".join(json.dumps(r) + "\n" for r in results))
    for r in results:
        if r["status"] != "ok":
            print(r["status"], r["model"])
    ok = sum(r["status"] == "ok" for r in results)
    print(ok, "/", len(results), "ok")
    return 0 if ok == len(results) else 1


if __name__ == "__main__":
    sys.exit(main())
