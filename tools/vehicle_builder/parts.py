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
# Example parameters of the sampled sensor levels (M1, B1, G1).
SENSOR_EXAMPLE = ROOT / "runtime" / "hosts" / "ardupilot" / "examples" / "rover_all_sensors.toml"
SENSOR_TITLE = {"gnss": "GNSS", "magnetometer": "Magnetometer", "barometer": "Barometer"}
ACTUATOR_TITLE = {"drive": "Drive motor", "steering": "Steering servo",
                  "servos": "Control-surface servos"}
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


def devices(cc: dict, kind: str, levels: list[str]) -> list[dict]:
    """Real parts of one sensor kind (parts/KIND/*.toml) and, per level, the
    parameters the composer derives from each datasheet (`fire-compose part`)."""
    out = []
    for p in cc.get("parts", {}).get(kind, []):
        per_level = {}
        for level in levels:
            r = subprocess.run([str(compose_binary()), "part", kind, p["id"], "--level", level, "--json"],
                               capture_output=True, text=True)
            if r.returncode:
                continue
            rep = json.loads(r.stdout)
            per_level[level] = {
                "values": {k: (v["value"] if isinstance(v["value"], str) else literal(v["value"]))
                           for k, v in rep["parameters"].items()},
                "how": {k: v["how"] + (f" [{', '.join(v['from'])}]" if v["from"] else "")
                        for k, v in rep["parameters"].items()},
                "assumed": rep["assumed"],
                "not_represented": rep["not_represented"],
                "not_stated": rep["not_stated"],
            }
        out.append({**p, "levels": per_level})
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

            actuator_parameters = {name for a in spec.get("actuators", [])
                                   for level in a["levels"] for name in level["parameters"]}
            airframe = [field(["plant", p["name"]], p["name"]) for p in spec["plant_parameters"]
                        if p["name"] not in actuator_parameters]
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
        else:
            parts += [actuator_part(a, field) for a in spec.get("actuators", [])]
        parts.append(imu_part(cc, mounts.get("imu", [])))
        parts += [sensor_part(cc, sensor, mounts.get(sensor, []))
                  for sensor in ("magnetometer", "barometer", "gnss")]
        airframes[name] = {"vehicle": kind, "model_name": doc.get("model_name", ""),
                           "parts": parts}
    return {"airframes": airframes}


def actuator_part(actuator: dict, field) -> dict:
    """A rover/plane actuator: its levels and the plant parameters each adds."""
    levels = [{"id": l["level"], "label": f"{l['level']} {l['model'].replace('_', ' ')}",
               "description": l["description"] + ".",
               "select": {f"models.{actuator['key']}": l["model"]},
               "fields": [field(["plant", name], name) for name in l["parameters"]]}
              for l in actuator["levels"]]
    return {"id": actuator["key"], "title": ACTUATOR_TITLE.get(actuator["key"], actuator["key"]),
            "levels": levels}


def sensor_part(cc: dict, sensor: str, mounts: list[dict]) -> dict:
    """Magnetometer, barometer or GNSS: levels from the composer, example values
    of the sampled levels from SENSOR_EXAMPLE."""
    example = tomllib.loads(SENSOR_EXAMPLE.read_text())["physics"].get(sensor, {})
    levels = []
    for spec in cc["models"][sensor]:
        sampled = spec["model"] != cc["models"][sensor][0]["model"]
        values = example if sampled else {}
        fields = [{"path": [sensor, k], "value": literal(values[k]) if k in values else "",
                   "default": "library default", "description": ""} for k in spec["keys"]]
        levels.append({"id": spec["level"], "label": f"{spec['level']} {spec['model']}",
                       "description": spec["description"] + ".",
                       "select": {f"models.{sensor}.model": spec["model"]}, "fields": fields})
    return {"id": sensor, "title": SENSOR_TITLE[sensor], "levels": levels, "fields": mounts,
            "devices": devices(cc, sensor, [lv["id"] for lv in levels])}


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
    return {"id": "imu", "title": "IMU", "levels": levels, "fields": mounts,
            "devices": devices(cc, "imu", [lv["id"] for lv in levels])}


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
              "devices": {part: device id}, "values": {"plant.mass": "4.0", ...}, "sitl": {...}}

    A part with a device names it ([models.KIND] part, level): the composer
    fills the level's parameters from the datasheet, and values set here win.
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
        device = choice.get("devices", {}).get(part["id"])
        if device and any(d["id"] == device for d in part.get("devices", [])):
            put(physics, ["models", part["id"], "part"], device)
            put(physics, ["models", part["id"], "level"], level["id"])
        else:
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
    """configs airframe + one part at a level (the levels' example parameters).

    imu (0-3), propulsion (0-4, multirotor), magnetometer/barometer/gnss (0-1),
    drive/steering (0-1, rover), servos (0-1, fixed-wing).
    """
    doc = copy.deepcopy(doc)
    if part in SENSOR_TITLE and level:
        cc = compose_catalog()
        doc.setdefault("models", {})[part] = {"model": cc["models"][part][level]["model"]}
        doc[part] = tomllib.loads(SENSOR_EXAMPLE.read_text())["physics"][part]
        return doc
    if part in ACTUATOR_TITLE:
        cc = compose_catalog()
        actuator = next(a for a in cc["vehicles"][doc.get("vehicle", "multirotor")].get("actuators", [])
                        if a["key"] == part)
        doc.setdefault("models", {})[part] = actuator["levels"][level]["model"]
        return doc
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
