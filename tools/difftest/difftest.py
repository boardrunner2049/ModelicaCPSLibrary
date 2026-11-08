#!/usr/bin/env python3
"""Differential testing of simulators: run two, or take two runs' traces,
and measure how far apart they are, channel by channel.

  difftest.py run SPEC.toml [--out DIR] [--no-build] [--reuse]
      produce both traces as the spec says and compare them (specs/*.toml);
      --reuse compares the last run's traces again
  difftest.py compare A B [options]
      compare two traces: trace CSVs, ArduPilot logs (.bin), or any CSV with
      --map-a/--map-b column maps (see sources.map_csv)
  difftest.py convert IN OUT.csv [--map MAP.toml]
      any source to a trace CSV
  difftest.py replay MODEL INPUT [-o OUT.csv] [--set NAME=VALUE]...
      a CP Models vehicle driven open loop by INPUT's servo outputs

A spec names its two sides, A (the reference) and B, each one of:

  [a]
  trace = "run.bin"                       # an existing trace or log
  map = "gazebo_map.toml"                 #   with a column map, for other CSVs
  [a.sitl]                                # fly ArduPilot SITL with a mission check
  model = "quad"                          #   any --model: ArduPilot's own physics,
  vehicle = "copter"                      #   json, gazebo-..., cpvehicle:NAME
  parameters = ["my.parm"]                #   (default: the vehicle's default_params)
  config = "configs/quad.toml"            #   or a CP Models vehicle: cpvehicle physics,
                                          #   its parameters and home, built first
  speedup = 2, instance = 6, home = "lat,lon,alt,hdg", every = 1
  [b.replay]                              # a CP Models vehicle, open loop,
  model = "quad"                          #   driven by the other side's servo
  inputs = "a"                            #   outputs ("a", "b", or a trace path)
  parameters = { lat0 = 40.41 }

and how to compare them ([compare] and [[check]], see compare.py). Results go
to build/difftest/SPEC/: a.csv, b.csv, report.html, result.json. Exit status:
0 passed or nothing checked, 1 a check failed, 2 an error.
"""
from __future__ import annotations

import argparse
from concurrent.futures import ThreadPoolExecutor
import json
from pathlib import Path
import sys
import tomllib

sys.path.insert(0, str(Path(__file__).resolve().parent))
import compare as cmp  # noqa: E402
import report  # noqa: E402
import sources  # noqa: E402
from traces import Trace  # noqa: E402

ROOT = sources.ROOT


def _settings(args) -> dict:
    return {"align": args.align, "offset": args.offset, "xcorr": args.xcorr,
            "max_shift": args.max_shift, "start": args.start, "end": args.end, "rate": args.rate,
            "channels": args.channels.split(",") if args.channels else None}


def _finish(result: cmp.Result, out_html: Path | None, out_json: Path | None, title: str) -> int:
    print(cmp.table(result))
    if out_html:
        out_html.write_text(report.render(result, title))
        print(f"report: {out_html}")
    if out_json:
        out_json.write_text(json.dumps(cmp.to_json(result), indent=1, default=str))
    return 0 if result.ok or not result.checked else 1


def cmd_compare(args) -> int:
    a = sources.load(args.a, args.map_a)
    b = sources.load(args.b, args.map_b)
    a.meta.setdefault("name", Path(args.a).name)
    b.meta.setdefault("name", Path(args.b).name)
    checks = []
    if args.checks:
        with open(args.checks, "rb") as f:
            checks = tomllib.load(f).get("check", [])
    result = cmp.compare(a, b, _settings(args), checks)
    return _finish(result, args.html, args.json, args.title or f"{a.name} vs {b.name}")


def cmd_convert(args) -> int:
    trace = sources.load(args.input, args.map)
    trace.write(args.output)
    print(f"{args.output}: {len(trace.t)} samples, {trace.t[-1] - trace.t[0]:.2f} s; "
          f"{', '.join(trace.names())}")
    return 0


def cmd_replay(args) -> int:
    params = {}
    for s in args.set or []:
        k, _, v = s.partition("=")
        params[k] = float(v)
    trace = sources.replay(args.model, sources.load(args.input), params, args.rate_hz,
                           args.output_hz)
    out = args.output or Path(f"replay_{Path(args.model).stem}.csv")
    trace.write(out)
    print(f"{out}: {len(trace.t)} samples")
    return 0


def _side(key: str, table: dict, spec_dir: Path, out: Path, build: bool, others: dict) -> Trace:
    """Produce side `key` of a spec."""
    def path(p):
        p = Path(p)
        return p if p.is_absolute() else (spec_dir / p if (spec_dir / p).exists() else ROOT / p)

    kinds = [k for k in ("trace", "sitl", "replay") if k in table]
    if len(kinds) != 1:
        raise ValueError(f"[{key}] needs exactly one of trace, sitl, replay")
    if "trace" in table:
        trace = sources.load(path(table["trace"]), path(table["map"]) if "map" in table else None)
        trace.meta.setdefault("name", table.get("name", Path(table["trace"]).name))
        return trace
    if "sitl" in table:
        s = dict(table["sitl"])
        if "config" in s:
            v = sources.sitl_vehicle(path(s.pop("config")), build)
            s.setdefault("model", v["model"])
            s.setdefault("vehicle", v["vehicle"])
            s.setdefault("parameters", v["parameters"])
            s.setdefault("home", v["home"])
        if "parameters" in s:
            s["parameters"] = [str(path(p)) for p in s["parameters"]]
        if "mission" in s:
            s["mission"] = [str(path(m)) if m.endswith(".py") else m for m in s["mission"]]
        s.setdefault("instance", 6 if key == "a" else 7)
        unknown = set(s) - {"model", "vehicle", "parameters", "home", "speedup", "instance",
                            "every", "mission", "timeout"}
        if unknown:
            raise ValueError(f"[{key}.sitl] unknown keys: {', '.join(sorted(unknown))}")
        trace, outcome = sources.run_sitl(work=out / key, log=lambda m: print(m, flush=True), **s)
        trace.meta["name"] = table.get("name", f"SITL {s['model']}")
        trace.meta["mission_check"] = "passed" if outcome.get("mission_status") == 0 else "failed"
        return trace
    r = dict(table["replay"])
    inputs = r.pop("inputs", "a" if key == "b" else "b")
    source = others[inputs] if inputs in ("a", "b") else sources.load(path(inputs))
    trace = sources.replay(r.pop("model"), source, r.pop("parameters", {}),
                           r.pop("rate_hz", 1200), r.pop("output_hz", None))
    if r:
        raise ValueError(f"[{key}.replay] unknown keys: {', '.join(sorted(r))}")
    trace.meta["name"] = table.get("name", f"replay {trace.meta['model']} ← {source.name}")
    return trace


def cmd_run(args) -> int:
    spec_path = Path(args.spec).resolve()
    with open(spec_path, "rb") as f:
        spec = tomllib.load(f)
    unknown = set(spec) - {"title", "a", "b", "compare", "check"}
    if unknown:
        raise ValueError(f"{spec_path.name}: unknown tables {', '.join(sorted(unknown))}")
    out = args.out or ROOT / "build" / "difftest" / spec_path.stem
    out.mkdir(parents=True, exist_ok=True)
    build = not args.no_build
    a_spec, b_spec = spec["a"], spec["b"]
    traces: dict[str, Trace] = {}
    if args.reuse:
        traces = {k: Trace.read(out / f"{k}.csv") for k in ("a", "b")}
        result = cmp.compare(traces["a"], traces["b"], spec.get("compare"), spec.get("check", []))
        return _finish(result, out / "report.html", out / "result.json",
                       spec.get("title", spec_path.stem))
    order = ["a", "b"]
    if "replay" in a_spec:
        order = ["b", "a"]
    if "sitl" in a_spec and "sitl" in b_spec:
        # Two SITL runs at once, on their own instances. A CP Models build is
        # shared: build sequentially first.
        if build:
            for t in (a_spec, b_spec):
                if "config" in t["sitl"]:
                    sources.sitl_vehicle(_spec_path(t["sitl"]["config"], spec_path), True)
        with ThreadPoolExecutor(2) as pool:
            futures = {k: pool.submit(_side, k, spec[k], spec_path.parent, out, False, traces)
                       for k in order}
            traces = {k: f.result() for k, f in futures.items()}
    else:
        for k in order:
            traces[k] = _side(k, spec[k], spec_path.parent, out, build, traces)
    for k, t in traces.items():
        t.write(out / f"{k}.csv")
    result = cmp.compare(traces["a"], traces["b"], spec.get("compare"), spec.get("check", []))
    return _finish(result, out / "report.html", out / "result.json",
                   spec.get("title", spec_path.stem))


def _spec_path(p: str, spec_path: Path) -> Path:
    p = Path(p)
    if p.is_absolute():
        return p
    return spec_path.parent / p if (spec_path.parent / p).exists() else ROOT / p


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest="cmd", required=True)

    r = sub.add_parser("run", help="produce and compare two traces as a spec says")
    r.add_argument("spec")
    r.add_argument("--out", type=Path, help="results directory (default build/difftest/SPEC)")
    r.add_argument("--no-build", action="store_true", help="use CP Models vehicles as built")
    r.add_argument("--reuse", action="store_true",
                   help="compare the traces of the last run again (a.csv, b.csv): new checks, no runs")
    r.set_defaults(func=cmd_run)

    c = sub.add_parser("compare", help="compare two traces")
    c.add_argument("a")
    c.add_argument("b")
    c.add_argument("--map-a")
    c.add_argument("--map-b")
    c.add_argument("--align", default="start")
    c.add_argument("--offset", type=float, default=0.0)
    c.add_argument("--xcorr")
    c.add_argument("--max-shift", type=float, default=2.0)
    c.add_argument("--start", type=float, default=0.0)
    c.add_argument("--end", type=float)
    c.add_argument("--rate", type=float, default=50.0)
    c.add_argument("--channels", help="comma-separated, globs allowed")
    c.add_argument("--checks", help="a TOML file with [[check]] tables")
    c.add_argument("--html", type=Path)
    c.add_argument("--json", type=Path)
    c.add_argument("--title")
    c.set_defaults(func=cmd_compare)

    v = sub.add_parser("convert", help="any source to a trace CSV")
    v.add_argument("input")
    v.add_argument("output", type=Path)
    v.add_argument("--map")
    v.set_defaults(func=cmd_convert)

    y = sub.add_parser("replay", help="drive a CP Models vehicle with a trace's servo outputs")
    y.add_argument("model")
    y.add_argument("input")
    y.add_argument("-o", "--output", type=Path)
    y.add_argument("--set", action="append", metavar="NAME=VALUE")
    y.add_argument("--rate-hz", type=float, default=1200)
    y.add_argument("--output-hz", type=float, help="default: every frame of a SITL trace, else 100")
    y.set_defaults(func=cmd_replay)

    args = p.parse_args()
    try:
        return args.func(args)
    except (ValueError, FileNotFoundError, RuntimeError, KeyError) as e:
        print(f"difftest: {e}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
