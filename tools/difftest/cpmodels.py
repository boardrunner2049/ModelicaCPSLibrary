"""CP Models for inception (https://github.com/Arslan8/inception), the
differential tester: the kinds of side a spec here can name.

    plugins = ["../cpmodels.py"]

    [a.ardupilot]           # ArduPilot SITL from third_party/ardupilot, with
    model = "x"             #   this repo's mission checks
    vehicle = "copter"      #   (inception's ardupilot_sitl keys)
    [b.cpvehicle]           # a CP Models vehicle in SITL: cpvehicle physics,
    config = "configs/quad.toml"   # its parameters and home; built first
    speedup = 2             #   (unless --no-build); other ardupilot_sitl keys
    [b.cpv_replay]          # a CP Models vehicle driven open loop by the
    model = "quad"          #   other side's servo outputs (runtime cpv_replay)
    inputs = "a"
    parameters = { lat0 = 40.41 }
    rate_hz = 1200          #   a SITL trace is replayed on its own frames
    output_hz = 100         #   default: every frame of a SITL trace, else 100

Replay from the shell: build/venv-gcs/bin/python tools/difftest/cpmodels.py replay MODEL INPUT.csv -o OUT.csv
"""
from __future__ import annotations

from pathlib import Path
import shlex
import subprocess
import sys

from inception import ardupilot_sitl, sources
from inception.traces import Trace

ROOT = Path(__file__).resolve().parents[2]
HOST = ROOT / "runtime" / "hosts" / "ardupilot"
AP = ROOT / "third_party" / "ardupilot"
REPLAY = ROOT / "build" / "runtime-release" / "cpv_replay"
PYTHON = ROOT / "build" / "venv-gcs" / "bin" / "python"
MISSION = {
    "copter": [str(HOST / "examples" / "fly_mission.py")],
    "rover": [str(HOST / "examples" / "fly_auto.py"), "rover"],
    "plane": [str(HOST / "examples" / "fly_auto.py"), "plane"],
}


def _ours(t: dict) -> dict:
    """ArduPilot SITL settings with this repo's tree, mission checks and Python."""
    s = dict(t)
    s.setdefault("ardupilot", str(AP))
    if "vehicle" in s:
        s.setdefault("mission", MISSION[s["vehicle"]])
    if PYTHON.exists():
        s.setdefault("python", str(PYTHON))
    return s


def ardupilot(t: dict, side) -> Trace:
    return ardupilot_sitl(_ours(t), side)


def vehicle(config: Path) -> dict:
    """SITL settings of a CP Models vehicle (runspec.py): model cpvehicle:NAME."""
    out = subprocess.run([sys.executable, str(HOST / "runspec.py"), str(config)],
                         check=True, capture_output=True, text=True).stdout
    env = {}
    for line in out.splitlines():
        k, _, v = line.partition("=")
        env[k] = shlex.split(v)[0] if v else ""
    kind = {"arducopter": "copter", "ardurover": "rover", "arduplane": "plane"}[env["AUTOPILOT"]]
    return {"model": f"cpvehicle:{env['NAME']}", "vehicle": kind,
            "parameters": env["PARAMETERS"].split(","), "home": env["HOME_LOCATION"]}


def cpvehicle(t: dict, side) -> Trace:
    s = dict(t)
    v = vehicle(side.path(s.pop("config")))
    for k, val in v.items():
        s.setdefault(k, val)
    trace = ardupilot_sitl(_ours(s), side)
    trace.meta["config"] = str(t["config"])
    return trace


def _build(t: dict, side) -> None:
    """Both sides' vehicles, one after the other: builds share build/."""
    if not side.build:
        return
    config = side.path(t["config"])
    side.log(f"[{side.key}] building {config.name}")
    done = subprocess.run([str(HOST / "build.sh"), str(config)], capture_output=True, text=True)
    if done.returncode:
        raise RuntimeError(f"build.sh {config} failed:\n{done.stdout[-2000:]}{done.stderr[-2000:]}")


cpvehicle.prepare = _build


def replay(model: str, inputs: Trace, work: Path, parameters: dict | None = None,
           rate_hz: float = 1200, output_hz: float | None = None, log=print) -> Trace:
    """Drive linked model `model` (or a descriptor path) with the pwm channels
    of `inputs`, held between samples, from the first sample's time on, in
    steps of 1/rate_hz, or on the input's own frames if it is a SITL trace.
    Output: every step of a SITL trace's frames (so both compare sample for
    sample), else 100 Hz, unless output_hz says otherwise."""
    if not REPLAY.exists():
        raise FileNotFoundError(f"{REPLAY}: build it (cmake --build build/runtime-release "
                                "--target cpv_replay)")
    frames = float(inputs.meta.get("frame_rate_hz", 0) or 0)
    rate = "input" if frames else str(rate_hz)
    if output_hz is None:
        output_hz = frames or 100.0
    every = max(1, round((frames or rate_hz) / output_hz))
    argv = [str(REPLAY), model, "{inputs}", "--rate", rate, "--every", str(every)]
    for k, v in (parameters or {}).items():
        argv += ["--set", f"{k}={v}"]
    trace = sources.command(argv, work, inputs, ["pwm*"], output="stdout", log=log)
    trace.meta.update(source="cpv-replay", model=model, inputs=inputs.name)
    return trace


def cpv_replay(t: dict, side) -> Trace:
    r = dict(t)
    if "inputs" not in r:
        raise ValueError(f"[{side.key}.cpv_replay] needs inputs: \"a\", \"b\" or a trace")
    inputs = side.inputs(r.pop("inputs"))
    trace = replay(r.pop("model"), inputs, side.work, r.pop("parameters", {}),
                   r.pop("rate_hz", 1200), r.pop("output_hz", None), side.log)
    if r:
        raise ValueError(f"[{side.key}.cpv_replay] unknown keys: {', '.join(sorted(r))}")
    trace.meta["name"] = f"replay {trace.meta['model']} ← {inputs.name}"
    return trace


SOURCES = {"ardupilot": ardupilot, "cpvehicle": cpvehicle, "cpv_replay": cpv_replay}


if __name__ == "__main__":
    import argparse
    import tempfile

    p = argparse.ArgumentParser(description="a CP Models vehicle driven by a trace's servo outputs")
    p.add_argument("cmd", choices=["replay"])
    p.add_argument("model")
    p.add_argument("input")
    p.add_argument("-o", "--output", type=Path)
    p.add_argument("--set", action="append", metavar="NAME=VALUE", default=[])
    p.add_argument("--rate-hz", type=float, default=1200)
    p.add_argument("--output-hz", type=float, help="default: every frame of a SITL trace, else 100")
    a = p.parse_args()
    params = {k: float(v) for k, _, v in (s.partition("=") for s in a.set)}
    with tempfile.TemporaryDirectory() as d:
        tr = replay(a.model, sources.load(a.input), Path(d), params, a.rate_hz, a.output_hz)
    out = a.output or Path(f"replay_{Path(a.model).stem}.csv")
    tr.write(out)
    print(f"{out}: {len(tr.t)} samples")
