"""The vehicle parts a configuration can pick, and composing a choice into TOML.

Sources, so nothing here restates the library:
  * `fire-compose catalog`: vehicle kinds, their configurable parameters, the
    models of each part (IMU I0-I3 and the [imu] keys each accepts, multirotor
    propulsion P0-P4);
  * the Modelica plant classes: each parameter's default and description;
  * configs/: the airframes (quad.toml, rover.toml, plane.toml) and the example
    parameters of each level (quad_iN.toml, quad_pN.toml).

A field is one configuration value: `path` (TOML keys), `value` (TOML literal
text the airframe or level starts from; empty keeps the Modelica default),
`default` and `description` (from Modelica, where known).
"""
from __future__ import annotations

import copy
import json
from pathlib import Path
import re
import subprocess
import tomllib

ROOT = Path(__file__).resolve().parents[2]
CONFIGS = ROOT / "configs"
COMPOSE = ROOT / "tools" / "fire-compose"

# Airframes: name -> (vehicle kind, base configuration).
AIRFRAMES = {
    "quad": ("multirotor", "quad"),
    "rover": ("rover", "rover"),
    "plane": ("fixed_wing", "plane"),
}
# Measured in ArduPilot SITL with the motors running (runtime/hosts/ardupilot/README.md).
SPEED = {
    "I0": "14-70x real time",
    "I1": "3-9x real time",
    "I2": "3-9x real time",
    "I3": "0.04-0.1x real time: a mission takes 15-45 min",
    "P0": "~12x real time",
    "P1": "~13x real time",
    "P2": "~12x real time",
    "P3": "~0.01x real time with the motors running: a mission takes hours",
    "P4": "far below 0.01x with the motors running: not practical in SITL",
}
PROPULSION_LABEL = {
    "P0": "first-order speed",
    "P1": "voltage, first-order",
    "P2": "DC motor",
    "P3": "3-phase, averaged",
    "P4": "3-phase, switching",
}
IMU_ABOUT = {
    "ideal": "truth at the mount, continuous",
    "digital": "sampled, filtered, quantized and clipped (ADC and digital filter)",
    "errors": "digital, plus noise, bias drift, scale, misalignment and temperature",
    "electromechanical": "MEMS gyro and accelerometer resonator dynamics behind the ADC",
}
SENSOR_ABOUT = {
    "gnss": "G0 ideal antenna position and velocity (G1 statistical: blocked by a Rumoca "
            "discrete-loop bug inside vehicles; G2 needs satellite inputs)",
    "magnetometer": "ideal mounted field (one level in the library)",
    "barometer": "ideal pressure, temperature and altitude (one level in the library)",
}
SENSOR_MOUNTS = {
    "imu": ["imuPosition_b", "R_bImu"],
    "gnss": ["gnssPosition_b"],
    "magnetometer": ["R_bMag"],
    "barometer": ["barometerPosition_b"],
}
PLANT_FILES = {
    "rover": ["Vehicles/Rover/RoverPlant.mo", "Vehicles/Rover/RoverWithSensors.mo"],
    "fixed_wing": ["Vehicles/FixedWing/FixedWingPlant.mo",
                   "Vehicles/FixedWing/FixedWingWithSensors.mo", "Exports/FMU/FixedWing.mo"],
}
RECORD_FILES = {"aero": "Physical/Mechanical/Aerodynamics/Airframe/Coefficients.mo"}
INITIAL_TARGET = {"altitude": "start_altitude_m", "speed": "start_speed_mps"}


# --- TOML text ---------------------------------------------------------------

def literal(v) -> str:
    if isinstance(v, bool):
        return "true" if v else "false"
    if isinstance(v, (int, float)):
        return repr(v)
    if isinstance(v, str):
        return json.dumps(v)
    if isinstance(v, list):
        return "[" + ", ".join(literal(x) for x in v) + "]"
    raise TypeError(f"cannot write {v!r}")


def dumps(doc: dict, path: tuple = ()) -> str:
    """TOML with inline arrays: keys first, then each subtable."""
    lines = [f"{k} = {literal(v)}" for k, v in doc.items() if not isinstance(v, dict)]
    for k, v in doc.items():
        if isinstance(v, dict):
            header = [] if v and all(isinstance(x, dict) for x in v.values()) else \
                [f"[{'.'.join((*path, k))}]"]
            lines += ["", *header, dumps(v, (*path, k))]
    return "\n".join(lines).replace("\n\n\n", "\n\n").strip("\n")


def parse_value(text: str, label: str):
    try:
        return tomllib.loads(f"v = {text}")["v"]
    except tomllib.TOMLDecodeError as e:
        raise ValueError(f"{label}: not a TOML value ({e})") from None


def load_config(name: str) -> dict:
    return tomllib.loads((CONFIGS / f"{name}.toml").read_text())


# --- sources -----------------------------------------------------------------

def compose_binary() -> Path:
    for profile in ("release", "debug"):
        exe = COMPOSE / "target" / profile / "fire-compose"
        if exe.is_file():
            return exe
    subprocess.run(["cargo", "build", "-q", "--manifest-path", str(COMPOSE / "Cargo.toml")],
                   check=True)
    return COMPOSE / "target" / "debug" / "fire-compose"


def compose_catalog() -> dict:
    out = subprocess.run([str(compose_binary()), "catalog"], check=True,
                         capture_output=True, text=True).stdout
    return json.loads(out)


def modelica_parameters(*files: str) -> dict[str, tuple[str, str]]:
    """name -> (default expression, description) of `parameter` declarations."""
    found = {}
    for f in files:
        text = re.sub(r"//[^\n]*", "", (ROOT / f).read_text())
        for stmt in re.findall(r'(?:[^;"]|"[^"]*")+', text):
            stmt = " ".join(stmt.split())
            m = re.search(r"\bparameter\s+[\w.]+(?:\s*\([^)]*\))?\s+(\w+)\s*(?:\[[^\]]*\])?"
                          r"\s*(?:\((?:[^()]|\([^()]*\))*\))?\s*=\s*(.*)$", stmt)
            if not m or "final parameter" in stmt:
                continue
            rest = m.group(2)
            desc = re.search(r'"([^"]*)"\s*$', rest)
            default = rest[:desc.start()].strip() if desc else rest.strip()
            found[m.group(1)] = (default, desc.group(1) if desc else "")
    return found


def table_fields(doc: dict, path: tuple = ()) -> list[dict]:
    """Every leaf of a TOML table as a field holding its value."""
    out = []
    for k, v in doc.items():
        if isinstance(v, dict):
            out += table_fields(v, (*path, k))
        else:
            out.append({"path": [*path, k], "value": literal(v), "default": "", "description": ""})
    return out


# --- catalog -----------------------------------------------------------------

def catalog() -> dict:
    """Everything the builder offers, per airframe: parts, levels and fields."""
    cc = compose_catalog()
    airframes = {}
    for name, (kind, base) in AIRFRAMES.items():
        doc = load_config(base)
        if kind == "multirotor":
            fixed = {k: v for k, v in doc.items()
                     if k in ("geometry", "mass", "rotor", "initial", "landing_gear", "interface")}
            fixed["rotor"] = {k: v for k, v in fixed.get("rotor", {}).items() if k != "J"}
            airframe = table_fields(fixed)
            mounts = {}
        else:
            spec = cc["vehicles"][kind]
            mo = modelica_parameters(*PLANT_FILES[kind])
            set_ = doc.get("plant", {})

            def field(path, name, default_name=None):
                default, desc = mo.get(default_name or name, ("", ""))
                cur = set_.get(name) if path[0] == "plant" and len(path) == 2 else None
                return {"path": path, "value": "" if cur is None else literal(cur),
                        "default": default, "description": desc}

            airframe = [field(["plant", p["name"]], p["name"]) for p in spec["plant_parameters"]]
            for record, fields in spec["records"].items():
                rec = modelica_parameters(RECORD_FILES[record])
                airframe += [{"path": ["plant", record, f], "value": "",
                              "default": rec.get(f, ("", ""))[0],
                              "description": rec.get(f, ("", ""))[1]} for f in fields]
            airframe += [field(["initial", p["name"]], p["name"],
                               INITIAL_TARGET.get(p["name"], p["name"]))
                         for p in spec["initial"]]
            mounts = {part: [field(["plant", m], m) for m in names]
                      for part, names in SENSOR_MOUNTS.items()}
        parts = [{"id": "airframe", "title": "Airframe" if kind == "multirotor" else "Plant",
                  "levels": [{"id": "-", "label": base + ".toml", "description":
                              f"configs/{base}.toml; empty fields keep the Modelica default",
                              "select": {}, "fields": airframe}]}]
        if kind == "multirotor":
            parts.append(propulsion_part(cc, doc))
        parts.append(imu_part(cc, mounts.get("imu", [])))
        for sensor in ("gnss", "magnetometer", "barometer"):
            parts.append({"id": sensor, "title": sensor.upper() if sensor == "gnss"
                          else sensor.capitalize(), "levels": [{
                              "id": "ideal", "label": "ideal", "select": {},
                              "description": SENSOR_ABOUT[sensor], "fields": []}],
                          "fields": mounts.get(sensor, [])})
        airframes[name] = {"vehicle": kind, "model_name": doc.get("model_name", ""),
                           "parts": parts}
    return {"airframes": airframes}


def imu_part(cc: dict, mounts: list[dict]) -> dict:
    levels = []
    for spec in cc["models"]["imu"]:
        n = int(spec["level"][1:])
        template = load_config(f"quad_i{n}")["imu"] if n else {}
        fields = table_fields({"imu": template})
        have = {f["path"][1] for f in fields}
        fields += [{"path": ["imu", k], "value": "", "default": "library default",
                    "description": ""} for k in spec["keys"] if k not in have]
        levels.append({"id": spec["level"], "label": f"{spec['level']} {spec['model']}",
                       "description": f"{IMU_ABOUT.get(spec['model'], '')}. "
                                      f"Speed: {SPEED[spec['level']]}.",
                       "select": {"models.imu.model": spec["model"]}, "fields": fields})
    return {"id": "imu", "title": "IMU", "levels": levels, "fields": mounts}


def propulsion_part(cc: dict, base: dict) -> dict:
    levels = []
    for spec in cc["models"]["propulsion"]:
        n = int(spec["level"][1:])
        src = load_config(f"quad_p{n}") if n else base
        tables = {t: src[t] for t in ("motor", "esc", "power") if t in src}
        if "J" in src.get("rotor", {}):
            tables["rotor"] = {"J": src["rotor"]["J"]}
        levels.append({"id": spec["level"], "label": f"{spec['level']} {PROPULSION_LABEL[spec['level']]}",
                       "description": f"{spec['description']}. Speed: {SPEED[spec['level']]}.",
                       "select": {"models.esc": spec["esc"], "models.motor": spec["motor"]},
                       "fields": table_fields(tables)})
    return {"id": "propulsion", "title": "Propulsion (ESC and motor)", "levels": levels}


# --- composing ---------------------------------------------------------------

def compose(choice: dict, cat: dict | None = None) -> tuple[dict, dict]:
    """A builder choice -> (run file document, physics document).

    choice = {"airframe": "quad", "model_name": "...", "levels": {part: level},
              "values": {"plant.mass": "4.0", ...}, "sitl": {...}}
    """
    cat = cat or catalog()
    frame = cat["airframes"][choice["airframe"]]
    physics: dict = {"schema_version": 3}
    if frame["vehicle"] != "multirotor":
        physics["vehicle"] = frame["vehicle"]
    sitl_name = choice.get("sitl", {}).get("name") or ""
    physics["model_name"] = (choice.get("model_name") or
                             "".join(w.capitalize() for w in sitl_name.split("_")) or
                             frame["model_name"])
    allowed = set()
    for part in frame["parts"]:
        level = next((lv for lv in part["levels"]
                      if lv["id"] == choice.get("levels", {}).get(part["id"])), part["levels"][0])
        for dotted, value in level["select"].items():
            put(physics, dotted.split("."), value)
        allowed |= {".".join(f["path"]) for f in level["fields"] + part.get("fields", [])}
    for dotted, text in choice.get("values", {}).items():
        if dotted not in allowed:
            continue  # a field of a level not selected
        if str(text).strip():
            put(physics, dotted.split("."), parse_value(str(text), dotted))
    if frame["vehicle"] == "multirotor":
        physics.setdefault("models", {}).setdefault("rotor", "quadratic_blade")
    sitl = {k: v for k, v in choice.get("sitl", {}).items() if v not in (None, "", [])}
    run = {"sitl": sitl, "physics": physics} if sitl else {"physics": physics}
    return run, physics


def put(doc: dict, path: list[str], value) -> None:
    for key in path[:-1]:
        doc = doc.setdefault(key, {})
    doc[path[-1]] = value


def run_file_text(run: dict, header: str = "") -> str:
    return (header + dumps(run) + "\n").lstrip("\n")


def apply_level(doc: dict, part: str, level: int) -> dict:
    """configs airframe + one part at a level (the levels' example parameters)."""
    doc = copy.deepcopy(doc)
    if part == "imu" and level:
        src = load_config(f"quad_i{level}")
        doc.setdefault("models", {})["imu"] = src["models"]["imu"]
        doc["imu"] = src["imu"]
    elif part == "propulsion" and level:
        src = load_config(f"quad_p{level}")
        for key in ("esc", "motor"):
            doc["models"][key] = src["models"][key]
        for table in ("motor", "esc", "power"):
            doc.pop(table, None)
            if table in src:
                doc[table] = src[table]
        if "J" in src.get("rotor", {}):
            doc["rotor"]["J"] = src["rotor"]["J"]
    return doc
