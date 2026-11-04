#!/usr/bin/env python3
"""Prepare the generic vehicle harness for one fire-compose configuration.

usage: vehicle.py prepare CONFIG.toml [--mission MISSION.toml] [--checks TEMPLATE] [--out DIR]
       vehicle.py describe CONFIG.toml        (print the vehicle description)
       vehicle.py metrics DIR                 (key flight errors of DIR/out/trace.csv)

prepare:
  1. fire-compose emit CONFIG --profile vio into <library>/build/fmi-vehicle/<name>/
     (the emitted <Model>.mo, <Model>.manifest.json and the staged
     sources/ModelicaCPSLibrary; files are only rewritten when they change, so
     the FMU is only re-exported when the composition or library changed);
  2. writes DIR (default tools/fmi/vehicle/out/<name>/), an ordinary harness
     directory built by ../../harness.mk:
       Makefile           MODEL/WRAPPER = the emitted model, LIBRARY_SOURCES = the staged copy
       world.toml         PWM inputs, sensor outputs, traced truth and diagnostics
       vehicle_config.h   VEHICLE (from the manifest), MISSION, sensor/PWM glue
       checks.toml        checks.template.toml filled for this vehicle and mission
       vehicle.json, mission.json   the description and resolved timeline (for metrics)
CONFIG is a path; relative paths are tried from the current directory, then
from the library root (so configs/quad.toml works from anywhere).
"""
import argparse, filecmp, json, math, os, shutil, string, subprocess, sys, tomllib
from pathlib import Path

VEHICLE_ROOT = Path(__file__).resolve().parent.parent
FMI_ROOT = VEHICLE_ROOT.parent
sys.path.insert(0, str(FMI_ROOT / "tools"))
import check_trace, fire_compile  # noqa: E402
from new_harness import toml_value  # noqa: E402

LIBRARY_ROOT = fire_compile.LIBRARY_ROOT
EMIT_ROOT = LIBRARY_ROOT / "build" / "fmi-vehicle"
COMPOSE = LIBRARY_ROOT / "tools" / "fire-compose" / "Cargo.toml"
GRAVITY = 9.80665


# ---- files -------------------------------------------------------------------

def write_if_changed(path: Path, text: str) -> None:
    if not path.exists() or path.read_text() != text:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(text)


def sync_tree(src: Path, dst: Path) -> None:
    """Make dst a copy of src, rewriting only files whose content differs."""
    dst.mkdir(parents=True, exist_ok=True)
    for path in sorted(src.rglob("*")):
        target = dst / path.relative_to(src)
        if path.is_dir():
            target.mkdir(exist_ok=True)
        elif not target.exists() or not filecmp.cmp(path, target, shallow=False):
            shutil.copy2(path, target)
    for path in sorted(dst.rglob("*"), reverse=True):
        if not (src / path.relative_to(dst)).exists():
            shutil.rmtree(path) if path.is_dir() else path.unlink()


def resolve_config(text: str) -> Path:
    for base in (Path.cwd(), LIBRARY_ROOT):
        path = (base / text).resolve()
        if path.exists():
            return path
    raise SystemExit(f"vehicle config {text} not found (tried . and {LIBRARY_ROOT})")


def emit(config: Path) -> tuple[Path, dict]:
    """fire-compose emit (VirtualIO profile); returns (emit dir, manifest)."""
    name = config.stem
    staging, dest = EMIT_ROOT / f".{name}.staging", EMIT_ROOT / name
    shutil.rmtree(staging, ignore_errors=True)
    command = ["cargo", "run", "--quiet", "--locked", "--offline", "--manifest-path", str(COMPOSE), "--",
               "emit", str(config), "--profile", "vio", "--output-dir", str(staging),
               "--library-root", str(LIBRARY_ROOT)]
    result = subprocess.run(command, capture_output=True, text=True)
    if result.returncode != 0:
        sys.stderr.write(result.stdout + result.stderr)
        raise SystemExit(f"fire-compose emit {config} failed")
    sync_tree(staging, dest)
    shutil.rmtree(staging)
    manifest = json.loads(next(dest.glob("*.manifest.json")).read_text())
    return dest, manifest


# ---- vehicle description -------------------------------------------------------

def matmul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(len(b))) for j in range(len(b[0]))] for i in range(len(a))]


def transpose(a):
    return [list(r) for r in zip(*a)]


def solve(a, b):
    """Gaussian elimination with partial pivoting: a x = b (a square)."""
    n = len(a)
    m = [list(map(float, a[i])) + [float(b[i])] for i in range(n)]
    for c in range(n):
        p = max(range(c, n), key=lambda r: abs(m[r][c]))
        if abs(m[p][c]) < 1e-12:
            raise SystemExit("rotor effectiveness matrix is singular")
        m[c], m[p] = m[p], m[c]
        for r in range(n):
            if r != c:
                f = m[r][c] / m[c][c]
                m[r] = [x - f * y for x, y in zip(m[r], m[c])]
    return [m[i][n] / m[i][i] for i in range(n)]


def cross(a, b):
    return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]


def combine_mass(parts):
    """Physical.Mechanical.Chassis.Functions.combineMassProperties."""
    mass = sum(p["mass"] for p in parts)
    cg = [sum(p["mass"] * p["r_C"][k] for p in parts) / mass for k in range(3)]
    inertia = [[0.0] * 3 for _ in range(3)]
    for p in parts:
        rot = matmul(matmul(p["R_Cj"], p["inertia"]), transpose(p["R_Cj"]))
        d = [p["r_C"][k] - cg[k] for k in range(3)]
        dd = sum(x * x for x in d)
        for j in range(3):
            for k in range(3):
                inertia[j][k] += rot[j][k] + p["mass"] * ((dd if j == k else 0.0) - d[j] * d[k])
    return mass, cg, inertia


def per_rotor(table: dict, key: str, n: int, default=None):
    value = table.get(key, default)
    if value is None:
        raise SystemExit(f"manifest has no {key}")
    return list(value) if isinstance(value, list) else [value] * n


def describe(manifest: dict, config: Path) -> dict:
    """The vehicle_t contents (SI units, body FRD about the combined CG)."""
    cfg = manifest["config"]
    geo, n = cfg["geometry"], cfg["geometry"]["nRotors"]
    masses = cfg["mass"]
    if masses["mode"] == "aggregate":
        parts = [masses["aggregate"]]
    else:
        parts = [masses["core"], *masses.get("arms", []), *masses.get("payloads", []),
                 *masses.get("additional_parts", [])]
    mass, cg, inertia = combine_mass(parts)

    position = [[geo["rotorPosition_C"][i][k] - cg[k] for k in range(3)] for i in range(n)]
    thrust_dir = [[geo["R_br"][i][k][2] for k in range(3)] for i in range(n)]
    rotor, motor, power = cfg["rotor"], cfg.get("motor", {}), cfg.get("power", {})
    kT, kQ = per_rotor(rotor, "kT", n), per_rotor(rotor, "kQ", n)
    spin = geo["spinSign"]
    drag = [[geo["R_br"][i][k][2] * -spin[i] * kQ[i] / kT[i] for k in range(3)] for i in range(n)]

    # Allocation of the weight (level hover): minimum-norm rotor thrusts.
    B = [[-thrust_dir[i][2] for i in range(n)]]
    for a in range(3):
        B.append([cross(position[i], thrust_dir[i])[a] + drag[i][a] for i in range(n)])
    y = solve(matmul(B, transpose(B)), [mass * GRAVITY, 0.0, 0.0, 0.0])
    thrust = [sum(B[r][i] * y[r] for r in range(4)) for i in range(n)]
    hover_omega = [math.sqrt(max(t, 0.0) / kT[i]) for i, t in enumerate(thrust)]

    # Steady-state command for a rotor speed: command = c0 + c1 w + c2 w^2.
    fidelity = manifest["physics"]["propulsion"]
    c0, c1, c2, tau = [0.0] * n, [0.0] * n, [0.0] * n, [0.0] * n
    for i in range(n):
        wh = hover_omega[i]
        if fidelity == "P0":    # FirstOrderSpeed: w -> omegaMax * command, lag tau
            c1[i] = 1.0 / per_rotor(motor, "omegaMax", n)[i]
            tau[i] = per_rotor(motor, "tau", n)[i]
        elif fidelity == "P1":  # FirstOrderVoltageToSpeed: w -> Kv * command * Vdc
            c1[i] = 1.0 / (per_rotor(motor, "Kv", n)[i] * power["dcVoltage"])
            tau[i] = per_rotor(motor, "tauOmega", n)[i]
        elif fidelity in ("P2", "P3", "P4"):
            # DC-equivalent steady state: u V = R i + k w, k i = b w + kQ w^2.
            # Six-step three-phase: two phases conduct, k = 2 ke (trapezoidal;
            # sqrt(3) ke for sinusoidal back-EMF), R = 2 RPhase, and the
            # freewheel diode drop Vd: u (V + Vd) = Vd + R i + k w.
            b, J = per_rotor(motor, "b", n, 0.0)[i], per_rotor(motor, "J", n)[i] + per_rotor(rotor, "J", n, 0.0)[i]
            if fidelity == "P2":
                k, R, vd = per_rotor(motor, "k", n)[i], per_rotor(motor, "R", n)[i], 0.0
            else:
                trapezoidal = per_rotor(motor, "trapezoidal", n, True)[i]
                k = (2.0 if trapezoidal else math.sqrt(3.0)) * per_rotor(motor, "ke", n)[i]
                R = 2.0 * per_rotor(motor, "RPhase", n)[i]
                vd = per_rotor(cfg.get("esc", {}), "Vdiode", n, 0.0)[i]
            v = power["dcVoltage"] + vd
            c0[i], c1[i], c2[i] = vd / v, (k + R * b / k) / v, R * kQ[i] / k / v
            tau[i] = J / (k * k / R + b + 2 * kQ[i] * wh)
        else:
            raise SystemExit(f"unsupported propulsion fidelity {fidelity}")
    omega_max = [((-c1[i] + math.sqrt(c1[i] ** 2 + 4 * c2[i] * (1 - c0[i]))) / (2 * c2[i])) if c2[i] > 0
                 else (1 - c0[i]) / c1[i] for i in range(n)]
    hover_command = [c0[i] + c1[i] * w + c2[i] * w * w for i, w in enumerate(hover_omega)]

    iface = cfg["interface"]
    gear = cfg.get("landing_gear", {})
    return {
        "name": manifest["model"], "config": config.name, "propulsion": fidelity,
        "n_rotors": n, "n_actuators": geo["nActuators"], "mass": mass, "cg": cg, "inertia": inertia,
        "gravity": GRAVITY, "position": position, "thrust_dir": thrust_dir, "drag_moment": drag,
        "spin_sign": spin, "actuator": [a - 1 for a in geo["actuatorIndex"]], "kT": kT, "kQ": kQ,
        "cmd_c0": c0, "cmd_c1": c1, "cmd_c2": c2, "omega_max": omega_max, "hover_omega": hover_omega,
        "hover_command": hover_command, "motor_tau": max(tau), "pwm_min": iface["pwm_min"],
        "pwm_max": iface["pwm_max"], "lat0_deg": iface["lat0"], "lon0_deg": iface["lon0"],
        "ground_alt_m": iface["ground_alt_wgs84"], "earth_radius_m": iface["earth_radius_m"],
        # Default sensor mounts (Geometry gnssPosition_C/barometerPosition_C = 0) relative to the CG.
        "gnss_position": [-x for x in cg], "baro_position": [-x for x in cg],
        "n_legs": 4 if gear.get("enabled", True) else 0,
        "arm": max(math.hypot(p[0], p[1]) for p in position),
    }


# ---- mission -------------------------------------------------------------------

KINDS = {"idle": "PHASE_IDLE", "fly": "PHASE_FLY", "land": "PHASE_LAND"}


def load_mission(path: Path) -> dict:
    mission = tomllib.loads(path.read_text())
    t, target, yaw = 0.0, [0.0, 0.0, 0.0], 0.0
    for phase in mission["phase"]:
        if phase.get("kind", "fly") not in KINDS:
            raise SystemExit(f"mission phase {phase['name']}: kind must be one of {sorted(KINDS)}")
        target, yaw = phase.get("target", target), phase.get("yaw_deg", yaw)
        phase.update(kind=phase.get("kind", "fly"), start=t, target=list(map(float, target)), yaw_deg=float(yaw))
        t += phase["duration"]
    mission["stop"] = t
    armed = [p for p in mission["phase"] if p["kind"] != "idle"]
    mission["t_arm"] = armed[0]["start"] if armed else 0.0
    return mission


# ---- generated files ---------------------------------------------------------

def c_num(x) -> str:
    return repr(float(x))


def c_array(values) -> str:
    if isinstance(values, (list, tuple)):
        return "{" + ", ".join(c_array(v) for v in values) + "}"
    return str(values) if isinstance(values, int) and not isinstance(values, bool) else c_num(values)


def vehicle_header(v: dict, mission: dict) -> str:
    fields = ["mass", "gravity", "motor_tau", "pwm_min", "pwm_max", "lat0_deg", "lon0_deg", "ground_alt_m",
              "earth_radius_m"]
    arrays = ["inertia", "position", "thrust_dir", "drag_moment", "spin_sign", "actuator", "kT", "kQ", "cmd_c0",
              "cmd_c1", "cmd_c2", "omega_max", "hover_omega", "hover_command", "gnss_position", "baro_position"]
    out = [f"/* Generated by tools/fmi/vehicle/tools/vehicle.py from {v['config']}; do not edit. */",
           "#ifndef VEHICLE_CONFIG_H", "#define VEHICLE_CONFIG_H", "",
           '#include "controller.h"', '#include "fire_harness.h"', '#include "world_bindings.h"', "",
           "#include <stdio.h>", "",
           f"#if {v['n_rotors']} > VEHICLE_MAX_ROTORS || {v['n_actuators']} > VEHICLE_MAX_ACTUATORS",
           "#error vehicle exceeds VEHICLE_MAX_ROTORS/VEHICLE_MAX_ACTUATORS", "#endif", "",
           "static const vehicle_t VEHICLE = {",
           f'    .name = "{v["name"]}",', f'    .config = "{v["config"]}",',
           f'    .propulsion = "{v["propulsion"]}",',
           f"    .n_rotors = {v['n_rotors']},", f"    .n_actuators = {v['n_actuators']},"]
    out += [f"    .{f} = {c_num(v[f])}," for f in fields]
    out += [f"    .{a} = {c_array(v[a])}," for a in arrays]
    out += ["};", ""]
    phases = mission["phase"]
    if len(phases) > 32:
        raise SystemExit("mission has more than MISSION_MAX_PHASES phases")
    out += ["static const mission_t MISSION = {", f'    .name = "{mission.get("name", "mission")}",']
    for key in ("step_s", "speed_h", "speed_v", "accel", "yaw_rate_deg", "land_speed", "tilt_limit_deg"):
        out.append(f"    .{key} = {c_num(mission[key])},")
    out += [f"    .stop_s = {c_num(mission['stop'])},", f"    .phase_count = {len(phases)},", "    .phases = {"]
    for p in phases:
        out.append(f'        {{"{p["name"]}", {KINDS[p["kind"]]}, {c_num(p["start"])}, {c_num(p["duration"])}, '
                   f'{c_array(p["target"])}, {c_num(p["yaw_deg"])}}},')
    out += ["    },", "};", ""]
    out += ["/* World bindings <-> controller arrays (aliases from world.toml). */",
            "#define VEHICLE_READ(alias, dst)                                                   \\",
            "  if (get_##alias(dst) != WM_OK) {                                                 \\",
            '    fprintf(stderr, "sensor " #alias " read failed: %s\\n", wm_last_error());      \\',
            "    return 1;                                                                      \\",
            "  }", "",
            "static int read_sensors(fc_sensors_t *s) {"]
    for field in ("accel", "gyro", "mag", "gps", "vel_ned"):
        out += [f"  VEHICLE_READ({field}_{k}, &s->{field}[{k - 1}]);" for k in (1, 2, 3)]
    out += ["  VEHICLE_READ(yaw_deg, &s->yaw_deg);", "  VEHICLE_READ(baro_altitude_m, &s->baro_alt);",
            "  return 0;", "}", "",
            "static wm_status_t (*const PWM_SETTERS[])(double) = {" +
            ", ".join(f"set_pwm_{c + 1}" for c in range(v["n_actuators"])) + "};", "",
            "static int write_pwm(const double pwm[]) {",
            f"  for (int c = 0; c < {v['n_actuators']}; ++c)",
            "    if (PWM_SETTERS[c](pwm[c]) != WM_OK) {",
            '      fprintf(stderr, "pwm[%d] = %g failed: %s\\n", c + 1, pwm[c], wm_last_error());',
            "      return 1;", "    }", "  return 0;", "}", "",
            "static const fh_output_t PRINTED[] = {FH_OUTPUT(baro_altitude_m), FH_OUTPUT(vel_ned_3), "
            "FH_OUTPUT(yaw_deg)};", "", "#endif", ""]
    return "\n".join(out)


def world_toml(v: dict, mission: dict) -> str:
    inputs = [(f"pwm_{c}", f"m.pwm[{c}]") for c in range(1, v["n_actuators"] + 1)]
    outputs = [(f"{f}_{k}", f"m.{f}[{k}]") for f in ("accel", "gyro", "mag", "gps", "vel_ned") for k in (1, 2, 3)]
    outputs += [("yaw_deg", "m.yaw_deg"), ("baro_altitude_m", "m.baro_altitude_m")]
    trace = ["m.truth.p_w", "m.truth.v_w", "m.truth.R_wb", "m.truth.omega_b", "m.pwm",
             "m.commands.normalizedCommand", "m.rotorSpeed", "m.gps", "m.vel_ned", "m.yaw_deg",
             "m.baro_altitude_m", "m.plant.chassis.mass", "m.plant.chassis.cg_C", "m.plant.chassis.inertia"]
    if v["n_legs"]:
        trace += ["m.plant.gear.contact", "m.plant.gear.normalForce"]
    out = [f"# Generated by tools/fmi/vehicle/tools/vehicle.py from {v['config']}; do not edit.",
           "[World]", "start = 0", f"stop = {round(mission['stop'] * 1e9)}",
           f"step = {round(mission['step_s'] * 1e9)}", 'time_unit = "ns"', "",
           "[Models.m]", 'path = "out/model.fmu"', "", "[World.Inputs]"]
    out += [f'{a} = "{e}"' for a, e in inputs]
    out += ["", "[World.Outputs]"] + [f'{a} = "{e}"' for a, e in outputs]
    out += ["", "[Trace]", "every_step = true", "variables = " + toml_value(trace),
            'output = "out/trace.csv"', "", "[Logging]", 'output = "out/runtime.log"', "",
            "[Frontend]", "enabled = false", ""]
    return "\n".join(out)


def makefile(v: dict, emit_dir: Path, out_dir: Path) -> str:
    src = os.path.relpath(VEHICLE_ROOT / "src", out_dir)
    mk = os.path.relpath(FMI_ROOT / "harness.mk", out_dir)
    return "\n".join([
        f"# Generated by tools/fmi/vehicle/tools/vehicle.py from {v['config']}; do not edit.",
        f"MODEL := {v['name']}",
        f"WRAPPER := {emit_dir / (v['name'] + '.mo')}",
        f"WRAPPER_MODEL := {v['name']}",
        f"LIBRARY_SOURCES := {emit_dir / 'sources' / 'ModelicaCPSLibrary'}",
        f"FMU_DEPS := {emit_dir / (v['name'] + '.manifest.json')}",
        f"VEHICLE_SRC := {src}",
        "HARNESS_SOURCES := $(VEHICLE_SRC)/harness.c $(VEHICLE_SRC)/controller.c",
        "HARNESS_DEPS := $(VEHICLE_SRC)/controller.h $(VEHICLE_SRC)/vehicle.h vehicle_config.h",
        "EXTRA_INCLUDES := . $(VEHICLE_SRC)",
        f"include {mk}", ""])


# ---- checks ------------------------------------------------------------------

NUMERIC = ("near", "tol", "rtol", "range", "from", "to")


def fill(check: dict, values: dict) -> dict:
    def sub(x):
        if isinstance(x, str):
            return string.Template(x).substitute(values)
        if isinstance(x, list):
            return [sub(e) for e in x]
        return x

    out = {k: sub(x) for k, x in check.items()}
    for key in NUMERIC:
        if key in out:
            out[key] = [float(e) for e in out[key]] if isinstance(out[key], list) else float(out[key])
    return out


def fmt(x: float) -> str:
    return repr(round(float(x), 12))


def checks_toml(v: dict, mission: dict, template_path: Path) -> str:
    template = tomllib.loads(template_path.read_text())
    tolerances = {**template.get("tolerances", {}), **mission.get("tolerances", {})}
    scope = {"mass": v["mass"], "arm": v["arm"], "n_rotors": v["n_rotors"],
             "hover_command": sum(v["hover_command"]) / v["n_rotors"]}
    tol = {k: float(eval(str(x), {"__builtins__": {}}, scope)) for k, x in tolerances.items()}
    base = {k: fmt(x) for k, x in tol.items()}
    I = [x for row in v["inertia"] for x in row]
    base.update({f"I{k + 1}": fmt(I[k]) for k in range(9)})
    base.update(mass=fmt(v["mass"]), cg_x=fmt(v["cg"][0]), cg_y=fmt(v["cg"][1]), cg_z=fmt(v["cg"][2]),
                t_arm=fmt(mission["t_arm"]), cos_tilt=fmt(math.cos(math.radians(tol["tilt_deg"]))),
                cos_hold_tilt=fmt(math.cos(math.radians(tol["hold_tilt_deg"]))),
                heading_rad=fmt(math.radians(tol["heading_deg"])))
    checks = [fill(c, base) for c in template.get("vehicle", [])]
    channels = sorted(set(v["actuator"]))
    for p in mission["phase"]:
        end = p["start"] + p["duration"]
        values = dict(base, phase=p["name"], t_start=fmt(p["start"]), t_end=fmt(end),
                      t_check=fmt(p["start"] + p.get("reach_by_s", p["duration"])),
                      t_settle=fmt(p["start"] + p.get("settle_s", 0.0)),
                      north=fmt(p["target"][0]), east=fmt(p["target"][1]), up=fmt(p["target"][2]),
                      yaw_deg=fmt(p["yaw_deg"]), yaw_rad=fmt(math.radians(p["yaw_deg"])))
        for kind in p.get("checks", []):
            if kind not in ("reach", "hold", "landed"):
                raise SystemExit(f"mission phase {p['name']}: unknown check {kind}")
            checks += [fill(c, values) for c in template.get(kind, [])]
            if kind == "hold":
                for ch in channels:
                    checks += [fill(c, dict(values, ch=str(ch + 1))) for c in template.get("hold_channel", [])]
                for i in range(v["n_rotors"]):
                    checks += [fill(c, dict(values, rotor=str(i + 1), hover_omega=fmt(v["hover_omega"][i])))
                               for c in template.get("hold_rotor", [])]
            if kind == "landed":
                for leg in range(1, v["n_legs"] + 1):
                    checks += [fill(c, dict(values, leg=str(leg))) for c in template.get("landed_leg", [])]
    out = [f"# Generated by tools/fmi/vehicle/tools/vehicle.py from {template_path.name},",
           f"# {v['config']} and mission {mission.get('name', '')}; do not edit.",
           "# Tolerances: " + ", ".join(f"{k} = {x:.4g}" for k, x in tol.items()), ""]
    for c in checks:
        out.append("[[check]]")
        out += [f"{k} = {toml_value(x)}" for k, x in c.items()]
        out.append("")
    return "\n".join(out)


# ---- commands ----------------------------------------------------------------

def prepare(args) -> int:
    config = resolve_config(args.config)
    emit_dir, manifest = emit(config)
    vehicle = describe(manifest, config)
    mission = load_mission(Path(args.mission).resolve())
    out_dir = Path(args.out or VEHICLE_ROOT / "out" / config.stem).resolve()
    write_if_changed(out_dir / "Makefile", makefile(vehicle, emit_dir, out_dir))
    write_if_changed(out_dir / "world.toml", world_toml(vehicle, mission))
    write_if_changed(out_dir / "vehicle_config.h", vehicle_header(vehicle, mission))
    write_if_changed(out_dir / "checks.toml", checks_toml(vehicle, mission, Path(args.checks).resolve()))
    write_if_changed(out_dir / "vehicle.json", json.dumps(vehicle, indent=1) + "\n")
    write_if_changed(out_dir / "mission.json", json.dumps(mission, indent=1) + "\n")
    return 0


def metrics(out_dir: Path) -> dict:
    """Key truth errors of a flown mission: per checked phase, the position,
    altitude and heading errors (at the reach time; maxima over the hold
    window) and the largest hold command; the largest tilt; the final state."""
    mission = json.loads((out_dir / "mission.json").read_text())
    times, col = check_trace.load(out_dir / "out" / "trace.csv")
    at = lambda t: check_trace.index_at(times, t)  # noqa: E731
    p = [col[f"m.truth.p_w[{k}]"] for k in (1, 2, 3)]
    a = at(mission["t_arm"])
    heading = [math.atan2(r4, r1) for r4, r1 in zip(col["m.truth.R_wb[4]"], col["m.truth.R_wb[1]"])]

    def errors(i, phase):
        n, e, up = phase["target"]
        xy = math.hypot(p[0][i] - p[0][a] - n, p[1][i] - p[1][a] - e)
        z = (p[2][a] - p[2][i]) - up
        yaw = math.degrees(math.remainder(heading[i] - heading[a] - math.radians(phase["yaw_deg"]), 2 * math.pi))
        return xy, z, yaw

    commands = [c for c in col if c.startswith("m.commands.normalizedCommand[")]
    out = {}
    for phase in mission["phase"]:
        checks = phase.get("checks", [])
        start, end = phase["start"], phase["start"] + phase["duration"]
        if "reach" in checks:
            xy, z, yaw = errors(at(start + phase.get("reach_by_s", phase["duration"])), phase)
            out[phase["name"]] = {"xy_m": xy, "z_m": z, "yaw_deg": yaw}
        if "hold" in checks:
            window = [i for i, t in enumerate(times) if start + phase.get("settle_s", 0.0) <= t <= end]
            errs = [errors(i, phase) for i in window]
            out[phase["name"] + "_hold"] = {
                "max_xy_m": max(e[0] for e in errs), "max_abs_z_m": max(abs(e[1]) for e in errs),
                "max_command": max(col[c][i] for c in commands for i in window)}
        if "landed" in checks:
            out[phase["name"]] = {
                "speed_mps": math.sqrt(sum(col[f"m.truth.v_w[{k}]"][-1] ** 2 for k in (1, 2, 3))),
                "xy_m": math.hypot(p[0][-1] - p[0][a], p[1][-1] - p[1][a]), "z_m": p[2][a] - p[2][-1]}
    out["max_tilt_deg"] = max(math.degrees(math.acos(max(-1.0, min(1.0, r)))) for r in col["m.truth.R_wb[9]"])
    return out


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="command", required=True)
    p = sub.add_parser("prepare")
    p.add_argument("config")
    p.add_argument("--mission", default=str(VEHICLE_ROOT / "mission.toml"))
    p.add_argument("--checks", default=str(VEHICLE_ROOT / "checks.template.toml"))
    p.add_argument("--out")
    d = sub.add_parser("describe")
    d.add_argument("config")
    m = sub.add_parser("metrics")
    m.add_argument("dir", type=Path)
    args = parser.parse_args()
    if args.command == "prepare":
        return prepare(args)
    if args.command == "metrics":
        print(json.dumps(metrics(args.dir), indent=1))
        return 0
    config = resolve_config(args.config)
    _, manifest = emit(config)
    print(json.dumps(describe(manifest, config), indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main())
