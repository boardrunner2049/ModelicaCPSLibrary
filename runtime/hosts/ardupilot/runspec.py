#!/usr/bin/env python3
"""Resolve a vehicle for the ArduPilot host: print it as shell assignments.

usage: runspec.py TARGET

TARGET is a configuration name (configs/NAME.toml) or a TOML file. The file
is either a vehicle configuration (fire-compose schema) or a run file:

    [sitl]                      # optional; every key optional
    name = "my_rover"           # SITL model name: --model cpvehicle:NAME (default: file stem)
    speedup = 1
    home = [40.414929, -86.932387, 149.0, 0.0]   # lat, lon, alt (m), heading (deg)
    parameters = ["extra.parm"] # added after the defaults, relative to this file
    devices = ["adsb", "gripper"] # ArduPilot optional hardware SITL simulates (devices.py)

    [physics]                   # the vehicle configuration (as configs/*.toml)
    schema_version = 3
    vehicle = "rover"
    ...

Prints NAME, CONFIG, TABLE, KIND, AUTOPILOT, WAF_TARGET, SPEEDUP, HOME_LOCATION,
PARAMETERS (comma-separated: the autopilot's defaults, the vehicle kind's
cpvehicle parameters, the devices' (build/vehicle-interface/NAME.devices.parm:
the vehicle's [peripherals] and the run's devices), then the run file's),
SERIAL (SITL --serialN devices) and NOTES.
"""
from __future__ import annotations

from pathlib import Path
import re

import devices as optional
import shlex
import sys
import tomllib

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
AP = ROOT / "third_party" / "ardupilot"
EXAMPLES = HERE / "examples"
NAME = re.compile(r"[a-z][a-z0-9_]{0,47}")
# Vehicle kind: autopilot binary, waf target, default parameter files.
KINDS = {
    "multirotor": ("arducopter", "copter", [AP / "Tools/autotest/default_params/copter.parm",
                                            EXAMPLES / "cpvehicle_quad.parm"]),
    "rover": ("ardurover", "rover", [AP / "Tools/autotest/default_params/rover.parm",
                                     EXAMPLES / "cpvehicle_rover.parm"]),
    "fixed_wing": ("arduplane", "plane", [AP / "Tools/autotest/models/plane.parm",
                                          EXAMPLES / "cpvehicle_plane.parm"]),
}


def fail(message: str) -> None:
    sys.exit(f"runspec: {message}")


def resolve(target: str) -> dict[str, str]:
    path = Path(target)
    if path.suffix != ".toml":
        path = ROOT / "configs" / f"{target}.toml"
    if not path.is_file():
        fail(f"{path} not found")
    path = path.resolve()
    try:
        doc = tomllib.loads(path.read_text())
    except tomllib.TOMLDecodeError as e:
        fail(f"{path}: {e}")
    sitl = doc.get("sitl", {})
    if "physics" in doc:
        physics, table = doc["physics"], "physics"
        extra = set(doc) - {"sitl", "physics"}
        if extra:
            fail(f"{path}: a run file has [sitl] and [physics] only, not {sorted(extra)}")
    else:
        physics, table = doc, ""
        if "sitl" in doc:
            fail(f"{path}: put the vehicle under [physics] when the file has [sitl]")
    unknown = set(sitl) - {"name", "speedup", "home", "parameters", "devices"}
    if unknown:
        fail(f"{path}: unknown [sitl] keys {sorted(unknown)}")
    name = sitl.get("name") or re.sub(r"[^a-z0-9_]", "_", path.stem.lower())
    if not NAME.fullmatch(name):
        fail(f"SITL name {name!r} must match {NAME.pattern}; set [sitl] name")
    kind = physics.get("vehicle", "multirotor")
    if kind not in KINDS:
        fail(f"{path}: unknown vehicle kind {kind!r}")
    autopilot, waf_target, parameters = KINDS[kind]
    home = sitl.get("home", [40.414929, -86.932387, 149.0, 0.0])
    if len(home) != 4:
        fail(f"{path}: [sitl] home is [lat, lon, alt, heading]")
    try:
        plan = optional.resolve(physics, list(sitl.get("devices", [])), home)
    except ValueError as e:
        fail(f"{path}: {e}")
    if plan.params:
        out = ROOT / "build" / "vehicle-interface"
        out.mkdir(parents=True, exist_ok=True)
        parm = out / f"{name}.devices.parm"
        parm.write_text("".join(f"{k} {v}\n" for k, v in plan.params.items()))
        parameters = [*parameters, parm]
    for extra in sitl.get("parameters", []):
        p = (path.parent / extra).resolve()
        if not p.is_file():
            fail(f"{path}: parameter file {p} not found")
        parameters = [*parameters, p]
    return {
        "NAME": name, "CONFIG": str(path), "TABLE": table, "KIND": kind,
        "AUTOPILOT": autopilot, "WAF_TARGET": waf_target,
        "SPEEDUP": str(sitl.get("speedup", 1)),
        "HOME_LOCATION": ",".join(str(x) for x in home),
        "PARAMETERS": ",".join(str(p) for p in parameters),
        "SERIAL": " ".join(plan.serial),
        "NOTES": "; ".join(plan.notes),
    }


if __name__ == "__main__":
    if len(sys.argv) != 2:
        fail("usage: runspec.py TARGET")
    for key, value in resolve(sys.argv[1]).items():
        print(f"{key}={shlex.quote(value)}")
