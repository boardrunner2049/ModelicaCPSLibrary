"""Where traces come from. Each source gives a trace.Trace.

  load(path, map=None)   a trace file: a trace CSV, an ArduPilot DataFlash log
                         (.bin: SITL truth, IMU, compass, barometer, GNSS,
                         servo outputs; also real flights, without truth), or
                         any simulator's CSV with a column map (map_csv)
  replay(model, inputs)  a CP Models vehicle driven open loop by the servo
                         outputs of another trace (runtime cpv_replay)
  run_sitl(...)          fly ArduPilot SITL with any physics backend (its own
                         --model, or cpvehicle:NAME from a vehicle TOML) and a
                         mission check; the backend's every frame is recorded
                         (SITL_TRACE, hooks.patch)
"""
from __future__ import annotations

from pathlib import Path
import os
import shlex
import subprocess
import sys
import tempfile
import time
import tomllib

import numpy as np

from traces import Trace, euler_from_quaternion, quaternion_multiply

ROOT = Path(__file__).resolve().parents[2]
HOST = ROOT / "runtime" / "hosts" / "ardupilot"
AP = ROOT / "third_party" / "ardupilot"
REPLAY = ROOT / "build" / "runtime-release" / "cpv_replay"
PYTHON = ROOT / "build" / "venv-gcs" / "bin" / "python"


def load(path: str | Path, map: str | Path | dict | None = None) -> Trace:
    path = Path(path)
    if map is not None:
        return map_csv(path, map)
    if path.suffix.lower() == ".bin":
        return ardupilot_log(path)
    return Trace.read(path)


# --- ArduPilot DataFlash logs -------------------------------------------------

# message: (instance field or None, [(field, channel, scale)])
_LOG_FIELDS = {
    "SIM2": (None, [("PN", "pos_n", 1), ("PE", "pos_e", 1), ("PD", "pos_d", 1),
                    ("VN", "vel_n", 1), ("VE", "vel_e", 1), ("VD", "vel_d", 1)]),
    "SIM": (None, [("Q1", "qw", 1), ("Q2", "qx", 1), ("Q3", "qy", 1), ("Q4", "qz", 1)]),
    "IMU": ("I", [("AccX", "accel_x", 1), ("AccY", "accel_y", 1), ("AccZ", "accel_z", 1),
                  ("GyrX", "gyro_x", 1), ("GyrY", "gyro_y", 1), ("GyrZ", "gyro_z", 1)]),
    "MAG": ("I", [("MagX", "mag_x", 1e-3), ("MagY", "mag_y", 1e-3), ("MagZ", "mag_z", 1e-3)]),
    "BARO": ("I", [("Press", "baro_press", 1), ("Alt", "baro_alt", 1)]),
    "GPS": ("I", [("Lat", "gps_lat", 1), ("Lng", "gps_lon", 1), ("Alt", "gps_alt", 1),
                  ("VZ", "gps_vd", 1)]),
    "RCOU": (None, [(f"C{i}", f"pwm{i}", 1) for i in range(1, 15)]),
    "RCO2": (None, [(f"C{i}", f"pwm{i}", 1) for i in range(15, 19)]),
}


def ardupilot_log(path: Path) -> Trace:
    """An ArduPilot DataFlash log. SIM/SIM2 (truth) exist only in SITL logs.
    Events: arm and disarm times (meta event.arm, event.disarm)."""
    from pymavlink import DFReader

    log = DFReader.DFReader_binary(str(path), zero_time_base=False)
    samples: dict[str, list[tuple[float, float]]] = {}
    events: dict[str, float] = {}
    while True:
        m = log.recv_match(type=list(_LOG_FIELDS) + ["ARM", "GPS"])
        if m is None:
            break
        typ = m.get_type()
        t = m.TimeUS * 1e-6
        if typ == "ARM":
            events.setdefault("arm" if m.ArmState else "disarm", t)
            continue
        inst, fields = _LOG_FIELDS[typ]
        if inst and getattr(m, inst, 0) != 0:
            continue
        for fld, ch, scale in fields:
            samples.setdefault(ch, []).append((t, float(getattr(m, fld)) * scale))
        if typ == "GPS":
            spd, crs = float(m.Spd), np.radians(float(m.GCrs))
            samples.setdefault("gps_vn", []).append((t, spd * np.cos(crs)))
            samples.setdefault("gps_ve", []).append((t, spd * np.sin(crs)))
    # Unused servo outputs log 0.
    for ch in [c for c in samples if c.startswith("pwm")]:
        if not any(v for _, v in samples[ch]):
            del samples[ch]
    trace = _merge(samples, {"source": "ardupilot-log", "log": str(path)})
    if trace.has("qw"):
        r, p, y = euler_from_quaternion(*(trace.channels[c] for c in ("qw", "qx", "qy", "qz")))
        trace.channels.update(roll=r, pitch=p, yaw=y)
    for k, v in events.items():
        trace.meta[f"event.{k}"] = f"{v:.6f}"
    return trace


def _merge(samples: dict[str, list[tuple[float, float]]], meta: dict) -> Trace:
    """Channels sampled at their own times, on the union of the times (NaN elsewhere)."""
    if not samples:
        raise ValueError("no channels found")
    grid = np.unique(np.concatenate([np.array([t for t, _ in s]) for s in samples.values()]))
    channels = {}
    for ch, s in samples.items():
        a = np.array(s)
        col = np.full(len(grid), np.nan)
        col[np.searchsorted(grid, a[:, 0])] = a[:, 1]
        channels[ch] = col
    return Trace(grid, channels, meta)


# --- Any simulator's CSV ------------------------------------------------------

def map_csv(path: Path, map: str | Path | dict) -> Trace:
    """A CSV from any simulator, described by a map (TOML file or dict):

        delimiter = ","                 # optional
        t = "timestamp * 1e-6"          # an expression of the file's columns
        [channels]                       # trace channel = expression
        pos_n = "y"                      # e.g. ENU position to NED
        pos_e = "x"
        pos_d = "-z"
        rate_x = "wx"
        rate_y = "-wy"                   # FLU body rates to FRD
        [attitude]                       # optional: a quaternion's columns
        frame = "enu_flu"                # or "ned_frd"
        q = ["qw", "qx", "qy", "qz"]     # scalar first

    Expressions see the file's columns as arrays (names that are not
    identifiers as col["a.b"]) and numpy as np. Attitude in ENU/FLU (ROS,
    Gazebo) is turned into the body FRD to NED quaternion and Euler angles.
    """
    if not isinstance(map, dict):
        with open(map, "rb") as f:
            map = tomllib.load(f)
    delim = map.get("delimiter", ",")
    raw = np.genfromtxt(path, delimiter=delim, names=True, dtype=float, comments="#",
                        deletechars="", invalid_raise=False)
    cols = {n: np.asarray(raw[n], dtype=float) for n in raw.dtype.names}
    scope = {"np": np, "col": cols, "__builtins__": {}}
    scope.update({n: v for n, v in cols.items() if n.isidentifier()})

    def ev(expr):
        return np.broadcast_to(np.asarray(eval(expr, scope), dtype=float), (len(raw),)).copy()

    t = ev(map.get("t", "t"))
    channels = {name: ev(expr) for name, expr in map.get("channels", {}).items()}
    att = map.get("attitude")
    if att:
        q = tuple(ev(c) for c in att["q"])
        frame = att.get("frame", "ned_frd")
        if frame == "enu_flu":
            h = np.sqrt(0.5)
            ned_enu = (0.0, h, h, 0.0)       # 180 deg about (north+east)/sqrt2
            flu_frd = (0.0, 1.0, 0.0, 0.0)   # 180 deg about x
            q = quaternion_multiply(quaternion_multiply(ned_enu, q), flu_frd)
        elif frame != "ned_frd":
            raise ValueError(f"attitude frame must be enu_flu or ned_frd, not {frame}")
        sign = np.where(q[0] < 0, -1.0, 1.0)
        q = tuple(np.asarray(c) * sign for c in q)
        channels.update(zip(("qw", "qx", "qy", "qz"), q))
        channels.update(zip(("roll", "pitch", "yaw"), euler_from_quaternion(*q)))
    return Trace(t, channels, {"source": "csv", "path": str(path)})


# --- CP Models vehicle, open loop ---------------------------------------------

def replay(model: str, inputs: Trace, parameters: dict | None = None,
           rate_hz: float = 1200, output_hz: float | None = None) -> Trace:
    """Drive linked model `model` (or a descriptor path) with the pwm channels
    of `inputs`, held between samples, from the first sample's time on, in
    steps of 1/rate_hz, or on the input's own frames if it is a SITL trace.
    Output: every step of a SITL trace's frames (so both compare sample for
    sample), else 100 Hz, unless output_hz says otherwise."""
    if not REPLAY.exists():
        raise FileNotFoundError(f"{REPLAY}: build it (cmake --build build/runtime-release "
                                "--target cpv_replay)")
    pwm = sorted((c for c in inputs.names() if c.startswith("pwm")), key=lambda c: int(c[3:]))
    if not pwm:
        raise ValueError(f"{inputs.name}: no pwm channels to replay")
    # A frame-by-frame trace (SITL_TRACE) is replayed on its own frames: the
    # steps of the run that recorded it.
    frames = float(inputs.meta.get("frame_rate_hz", 0) or 0)
    rate = "input" if frames else str(rate_hz)
    if output_hz is None:
        output_hz = frames or 100.0
    every = max(1, round((frames or rate_hz) / output_hz))
    args = [str(REPLAY), model, "", "--rate", rate, "--every", str(every)]
    for k, v in (parameters or {}).items():
        args += ["--set", f"{k}={v}"]
    with tempfile.TemporaryDirectory() as d:
        src = Path(d) / "inputs.csv"
        Trace(inputs.t, {c: inputs.channels[c] for c in pwm}).write(src)
        args[2] = str(src)
        out = Path(d) / "out.csv"
        with open(out, "w") as f:
            done = subprocess.run(args, stdout=f, stderr=subprocess.PIPE, text=True)
        if done.returncode:
            raise RuntimeError(f"cpv_replay failed: {done.stderr.strip()}")
        trace = Trace.read(out)
    trace.meta.update(source="cpv-replay", model=model, inputs=inputs.name)
    return trace


# --- ArduPilot SITL, any physics --------------------------------------------

FLY = {
    "copter": [str(HOST / "examples" / "fly_mission.py")],
    "rover": [str(HOST / "examples" / "fly_auto.py"), "rover"],
    "plane": [str(HOST / "examples" / "fly_auto.py"), "plane"],
}
AUTOPILOT = {"copter": "arducopter", "rover": "ardurover", "plane": "arduplane"}
# ArduPilot's defaults for its own models, as sim_vehicle.py uses them.
DEFAULT_PARAMETERS = {
    "copter": AP / "Tools" / "autotest" / "default_params" / "copter.parm",
    "rover": AP / "Tools" / "autotest" / "default_params" / "rover.parm",
    "plane": AP / "Tools" / "autotest" / "models" / "plane.parm",
}
DEFAULT_HOME = "40.414929,-86.932387,149.0,0.0"


def sitl_vehicle(vehicle_toml: str | Path, build: bool = True) -> dict:
    """SITL settings of a CP Models vehicle (runspec.py): model cpvehicle:NAME."""
    out = subprocess.run([sys.executable, str(HOST / "runspec.py"), str(vehicle_toml)],
                         check=True, capture_output=True, text=True).stdout
    env = {}
    for line in out.splitlines():
        k, _, v = line.partition("=")
        env[k] = shlex.split(v)[0] if v else ""
    if build:
        done = subprocess.run([str(HOST / "build.sh"), str(vehicle_toml)],
                              capture_output=True, text=True)
        if done.returncode:
            raise RuntimeError(f"build.sh {vehicle_toml} failed:\n{done.stdout[-2000:]}"
                               f"{done.stderr[-2000:]}")
    vehicle = {"arducopter": "copter", "ardurover": "rover", "arduplane": "plane"}[env["AUTOPILOT"]]
    return {"model": f"cpvehicle:{env['NAME']}", "vehicle": vehicle,
            "parameters": env["PARAMETERS"].split(","), "home": env["HOME_LOCATION"],
            "name": env["NAME"]}


def run_sitl(model: str, vehicle: str, work: Path, parameters: list[str] | None = None,
             home: str = DEFAULT_HOME, speedup: float = 1, instance: int = 6,
             every: int = 1, mission: list[str] | None = None, timeout: float = 1800,
             log=print) -> tuple[Trace, dict]:
    """Fly `model` (an ArduPilot --model: quad, rover, plane, cpvehicle:NAME,
    json, gazebo-..., ...) under `vehicle`'s autopilot with its mission check.
    Returns the frame-by-frame trace and the run's outcome (mission check
    status, DataFlash log, SITL output)."""
    work.mkdir(parents=True, exist_ok=True)
    binary = AP / "build" / "sitl" / "bin" / AUTOPILOT[vehicle]
    if parameters is None:
        parameters = [str(DEFAULT_PARAMETERS[vehicle])]
    trace_path = work / "trace.csv"
    trace_path.unlink(missing_ok=True)
    env = dict(os.environ, SITL_TRACE=str(trace_path), SITL_TRACE_EVERY=str(every))
    port = 5760 + 10 * instance
    sitl_log = open(work / "sitl.log", "w")
    sitl = subprocess.Popen(
        [str(binary), "--model", model, "--defaults", ",".join(map(str, parameters)),
         "--home", home, "--speedup", str(speedup), f"-I{instance}", "-w"],
        cwd=work, env=env, stdout=sitl_log, stderr=subprocess.STDOUT)
    outcome = {"model": model, "vehicle": vehicle, "work": str(work)}
    try:
        time.sleep(2)
        if sitl.poll() is not None:
            raise RuntimeError(f"SITL exited: {(work / 'sitl.log').read_text()[-2000:]}")
        python = str(PYTHON if PYTHON.exists() else sys.executable)
        check = [python, "-u", "-W", "ignore"] + (mission or FLY[vehicle]) + \
                ["--connect", f"tcp:127.0.0.1:{port}"]
        log(f"[{model}] {' '.join(Path(c).name if '/' in c else c for c in check[4:])}")
        # SITL that panics waits for a debugger instead of exiting: watch its output.
        with open(work / "mission.log", "w") as f:
            flight = subprocess.Popen(check, stdout=f, stderr=subprocess.STDOUT)
            deadline = time.monotonic() + timeout
            while flight.poll() is None:
                died = sitl.poll() is not None or "PANIC" in (work / "sitl.log").read_text(errors="replace")
                if died or time.monotonic() > deadline:
                    flight.kill()
                    flight.wait()
                    raise RuntimeError(
                        f"[{model}] " + ("SITL failed:\n" + "\n".join(
                            (work / "sitl.log").read_text(errors="replace").splitlines()[-8:])
                            if died else f"mission check still running after {timeout:.0f} s"))
                time.sleep(1)
        outcome["mission_status"] = flight.returncode
        outcome["mission_log"] = str(work / "mission.log")
        tail = (work / "mission.log").read_text().strip().splitlines()[-2:]
        log(f"[{model}] mission check {'passed' if flight.returncode == 0 else 'FAILED'}: "
            + " / ".join(tail))
    finally:
        sitl.terminate()
        try:
            sitl.wait(5)
        except subprocess.TimeoutExpired:
            sitl.kill()
            sitl.wait()
        sitl_log.close()
    logs = sorted((work / "logs").glob("*.BIN"), key=lambda p: p.stat().st_mtime)
    if logs:
        outcome["dataflash"] = str(logs[-1])
    if not trace_path.exists():
        raise RuntimeError(f"[{model}] SITL wrote no trace ({trace_path}); its output: {work / 'sitl.log'}")
    trace = Trace.read(trace_path)
    for ch in [c for c in trace.channels if c.startswith("pwm")]:
        if not np.any(np.nan_to_num(trace.channels[ch])):
            del trace.channels[ch]  # never driven
    trace.meta.update(model=model, vehicle=vehicle)
    if logs:
        # Arming time from the log: its clock is SITL's, the trace's.
        try:
            events = ardupilot_log(logs[-1]).meta
            trace.meta.update({k: v for k, v in events.items() if k.startswith("event.")})
        except Exception as e:  # noqa: BLE001 - a log without events still compares
            log(f"[{model}] no events from {logs[-1].name}: {e}")
    return trace, outcome
