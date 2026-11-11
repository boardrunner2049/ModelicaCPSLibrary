"""The analog power-module pipeline (milestone 4).

Models (Systems.Sensing.PowerModule): A1 static gain and offset per channel,
A2 first-order output response, A3 component level (divider, shunt with
temperature coefficient and self-heating, amplifier). Bench data: the true
voltage and current (an independent reference) and the two sense voltages as
a logger records them. The logger is simulated here (its noise and
resolution are in each dataset's description), so a synthetic dataset is
model output plus logger error, never presented as a device.
"""
from __future__ import annotations

import datetime as _dt
import json
import math
import subprocess
import tomllib
from pathlib import Path

import numpy as np
from scipy import stats

from . import dataset as dsmod
from . import fitting
from .fmu import LIBRARY_ROOT, Slave, Trace, export, file_sha256, toolchain, rel

A1 = "Systems.Sensing.PowerModule.AnalogSensor"
A2 = "Systems.Sensing.PowerModule.FirstOrderAnalogSensor"
A3 = "Systems.Sensing.PowerModule.ShuntAnalogSensor"
OUTPUTS = ["voltageSense", "currentSense"]
CHANNEL = {"voltage": ("voltageSense", "voltage_sense_v"), "current": ("currentSense", "current_sense_v")}
COMPOSE = LIBRARY_ROOT / "tools/fire-compose/target/debug/fire-compose"


class Models:
    def __init__(self):
        self._s = {}

    def __getitem__(self, cls) -> Slave:
        if cls not in self._s:
            self._s[cls] = Slave(export(cls))
        return self._s[cls]

    def close(self):
        for s in self._s.values():
            s.close()


def reference_report(part: str, calibration: str | None = None) -> dict:
    cmd = [str(COMPOSE), "part", "power_module", part, "--level", "W2", "--json"]
    if calibration:
        cmd += ["--calibration", calibration]
    r = subprocess.run(cmd, capture_output=True, text=True, cwd=LIBRARY_ROOT)
    if r.returncode:
        raise RuntimeError(r.stderr.strip())
    return json.loads(r.stdout)


def simulate(slave: Slave, trace: Trace, params: dict, t_obs: np.ndarray, step: float) -> dict:
    """The sense voltages at the logger's sample times (each a communication point)."""
    stop = float(t_obs[-1])
    # A model takes the parameters it has (A1, A2 and A3 share most names).
    params = {k: v for k, v in params.items() if k in slave.vars}
    grid_trace = Trace(np.unique(np.concatenate([trace.time, t_obs])),
                       {k: np.array([trace.at(t)[k] for t in np.unique(np.concatenate([trace.time, t_obs]))])
                        for k in trace.values})
    r = slave.run(grid_trace, params, OUTPUTS, stop + 1e-9, step)
    k = np.clip(np.searchsorted(r.time, t_obs), 1, len(r.time) - 1)
    k = np.where(np.abs(r.time[k - 1] - t_obs) < np.abs(r.time[k] - t_obs), k - 1, k)
    if np.max(np.abs(r.time[k] - t_obs)) > 1e-9:
        raise RuntimeError("logger samples are not communication points")
    return {o: r.outputs[o][k] for o in OUTPUTS}


# --- stimuli and the simulated logger --------------------------------------------

def stimulus(spec: dict) -> Trace:
    kind = spec["stimulus"]
    T0 = spec.get("temperature_c", 25.0) + 273.15
    if kind == "staircase":
        dwell = float(spec["dwell_s"])
        v = np.array(spec.get("voltage_v", [22.2]), float)
        i = np.array(spec.get("current_a", [1.0]), float)
        n = max(len(v), len(i))
        v = np.resize(v, n) if len(v) > 1 else np.full(n, v[0])
        i = np.resize(i, n) if len(i) > 1 else np.full(n, i[0])
        t = np.arange(n) * dwell
        return Trace(t, {"voltage": v, "current": i, "temperature": np.full(n, T0)})
    if kind == "profile":
        # A flight-like current profile (seeded) and the pack voltage sagging with it.
        rng = np.random.default_rng(spec["seed"] + 1000)
        dt = float(spec.get("segment_s", 0.5))
        n = int(spec["duration_s"] / dt)
        i = np.clip(np.convolve(rng.uniform(*spec["current_range_a"], n), np.ones(5) / 5, "same"), 0, None)
        v = spec["voltage_open_v"] - spec["pack_resistance_ohm"] * i - np.linspace(0, spec.get("voltage_drop_v", 0), n)
        return Trace(np.arange(n) * dt, {"voltage": v, "current": i, "temperature": np.full(n, T0)})
    if kind == "pulses":
        on, off = float(spec["on_s"]), float(spec["off_s"])
        t, v, i = [], [], []
        for k, a in enumerate(spec["current_a"]):
            t += [k * (on + off), k * (on + off) + on]
            i += [a, 0.0]
        n = len(t)
        return Trace(np.array(t), {"voltage": np.full(n, spec.get("voltage_v", 22.2)), "current": np.array(i),
                                   "temperature": np.full(n, T0)})
    raise ValueError(kind)


def logger(values: np.ndarray, spec: dict, rng) -> np.ndarray:
    lg = spec["logger"]
    y = values + lg["noise_v"] * rng.standard_normal(len(values))
    q = lg["range_v"] / 2 ** lg["bits"]
    return np.clip(np.round(y / q) * q, 0, lg["range_v"])


def generate(models: Models, cls: str, truth: dict, spec: dict, lg: dict, out: Path, meta: dict) -> dsmod.Dataset:
    trace = stimulus(spec)
    dur = float(spec.get("duration_s", trace.time[-1] + spec.get("dwell_s", spec.get("on_s", 1.0) + spec.get("off_s", 0))))
    rate = float(spec.get("logger_rate_hz", lg["rate_hz"]))
    # The logger's clock is not the stimulus's: samples sit half a period off
    # the step times, where the held input is unambiguous.
    t = (np.arange(int(dur * rate)) + 0.5) / rate
    clean = simulate(models[cls], trace, {**truth, **spec.get("parameters", {})}, t, min(1 / rate, spec.get("step_s", 1 / rate)))
    rng = np.random.default_rng(spec["seed"])
    full = {**lg, **spec.get("logger", {})}
    readings = {"time_s": t, "voltage_sense_v": logger(clean["voltageSense"], {"logger": full}, rng),
                "current_sense_v": logger(clean["currentSense"], {"logger": full}, rng)}
    reference = {"time_s": trace.time, "voltage_v": trace.values["voltage"], "current_a": trace.values["current"],
                 "temperature_k": trace.values["temperature"]}
    return dsmod.write(out / spec["name"], readings, reference, {
        **meta, "kind": "power_module", "synthetic": True, "generator": cls, "stimulus": spec, "truth": truth,
        "time_base": "simulation time; time_s is the logger's sample time",
        "reference": {"kind": "the generating true voltage and current (exact)", "clock_offset_s": 0.0},
        "logger": {**full, "rate_hz": rate, "simulated": "Gaussian noise then quantization, numpy seeded"},
        "conditions": {"temperature_c": spec.get("temperature_c", 25.0)}})


# --- calibration ------------------------------------------------------------------

def residual_fn(models, cls, datasets, base, channel, step=None):
    out, col = CHANNEL[channel]

    def r(p):
        res = []
        for d in datasets:
            rate = 1 / np.median(np.diff(d.readings["time_s"]))
            sim = simulate(models[cls], d.trace(), {**base, **p}, d.readings["time_s"], step or 1 / rate)
            res.append(sim[out] - d.readings[col])
        return np.concatenate(res)
    return r


def dwell_trend(models, cls, ds, params, channel) -> dict:
    """At constant input the static model predicts a constant output: a trend
    of the residual within dwells is behaviour the model does not represent."""
    out, col = CHANNEL[channel]
    t = ds.readings["time_s"]
    sim = simulate(models[cls], ds.trace(), params, t, 1 / np.median(np.diff(t)))
    e = ds.readings[col] - sim[out]
    edges = np.concatenate([ds.reference["time_s"], [np.inf]])
    slopes, ses = [], []
    for a, b in zip(edges[:-1], edges[1:]):
        m = (t > a + 1e-9) & (t < b - 1e-9)
        if m.sum() > 10:
            r = stats.linregress(t[m], e[m])
            slopes.append(r.slope)
            ses.append(r.stderr)
    slopes, ses = np.array(slopes), np.array(ses)
    zmax = float(np.max(np.abs(slopes / ses))) if len(slopes) else 0.0
    p = float(min(1.0, 2 * stats.norm.sf(zmax) * len(slopes))) if len(slopes) else 1.0  # Bonferroni
    return {"dwells": int(len(slopes)), "max_abs_slope_v_per_s": float(np.max(np.abs(slopes))) if len(slopes) else 0.0,
            "max_z": zmax, "p_value": p, "structured": p < 0.01}


def evaluate(models, ds, params, thresholds) -> dict:
    t = ds.readings["time_s"]
    sim = simulate(models[A1], ds.trace(), params, t, 1 / np.median(np.diff(t)))
    out = {}
    for ch, (o, col) in CHANNEL.items():
        e = ds.readings[col] - sim[o]
        m, rms = float(np.mean(e)), float(np.sqrt(np.mean(e ** 2)))
        out[ch] = {"mean_error_v": m, "rms_error_v": rms,
                   "pass": abs(m) <= thresholds["mean_error_v"] and rms <= thresholds["rms_error_v"]}
    out["pass"] = all(out[c]["pass"] for c in CHANNEL)
    return out


def run(experiment: Path, out: Path | None = None) -> dict:
    exp = tomllib.loads(Path(experiment).read_text())
    e = exp["experiment"]
    out = Path(out or LIBRARY_ROOT / "build/calibration" / e["name"])
    out.mkdir(parents=True, exist_ok=True)
    rep = {"experiment": e, "experiment_file": rel(experiment),
           "experiment_sha256": file_sha256(Path(experiment)),
           "created": _dt.datetime.now(_dt.timezone.utc).isoformat(timespec="seconds"), "versions": toolchain()}
    models = Models()
    try:
        rep["toolchain"] = toolchain_check(models)
        ref = reference_report(e["part"])
        rep["reference"] = ref
        nominal = {k: v["value"] for k, v in ref["parameters"].items()
                   if k in ("voltageGain", "voltageOffset", "currentGain", "currentOffset", "outputMax", "outputMin")
                   and isinstance(v["value"], (int, float))}
        truth = {**nominal, **exp.get("truth", {})}
        lg = exp["logger"]
        meta = {"description": e["description"], "unit": e["unit"], "part": e["part"]}
        data = {s["name"]: dsmod.obtain(s, Path(experiment).parent,
                                        lambda s=s: generate(models, s.get("generator", A1), {**truth, **s.get("truth", {})},
                                                             s, lg, out / "data", meta), "power_module")
                for s in exp["datasets"]}
        rep["datasets"] = {n: {**d.summary(), "role": s["role"],
                               "generator": "recorded" if "file" in s else s.get("generator", A1)}
                           for (n, d), s in zip(data.items(), exp["datasets"])}
        role = lambda r, tag=None: [data[s["name"]] for s in exp["datasets"]
                                    if s["role"] == r and (tag is None or tag in s.get("tags", []))]
        th_path = Path(experiment).parent / e["thresholds"]
        thresholds = tomllib.loads(th_path.read_text())
        rep["thresholds"] = {"file": rel(th_path), "sha256": file_sha256(th_path),
                             "values": thresholds, "read_before_holdout_evaluation": True}
        rep["logger_noise_v"] = lg["noise_v"]

        # Stage 1: static gain and offset per channel (A1).
        fits = {}
        for ch, names in (("voltage", ["voltageGain", "voltageOffset"]), ("current", ["currentGain", "currentOffset"])):
            ds = role("train", ch)
            fits[ch] = fitting.fit(residual_fn(models, A1, ds, nominal, ch), names,
                                   {n: nominal.get(n, 0.0) for n in names},
                                   {n: (0.5 * nominal[n], 1.5 * nominal[n]) if "Gain" in n else (-0.2, 0.2) for n in names},
                                   scale={n: nominal[n] if "Gain" in n else 0.01 for n in names})
        static = {**nominal, **fits["voltage"]["estimate"], **fits["current"]["estimate"]}
        # Stage 2: the output response from a fast step (A2), the static values
        # held, then all three jointly.
        steps = role("train", "step")
        tc = fitting.fit(residual_fn(models, A2, steps, {**static, "voltageTimeConstant": 1e-3}, "current", step=2.5e-5),
                         ["currentTimeConstant"], {"currentTimeConstant": 5e-4}, {"currentTimeConstant": (1e-5, 0.05)},
                         scale={"currentTimeConstant": 1e-3})
        joint = fitting.fit(residual_fn(models, A2, role("train", "current") + steps,
                                        {**static, "voltageTimeConstant": 1e-3}, "current", step=None),
                            ["currentGain", "currentOffset", "currentTimeConstant"],
                            {**fits["current"]["estimate"], **tc["estimate"]},
                            {"currentGain": (0.5 * nominal["currentGain"], 1.5 * nominal["currentGain"]),
                             "currentOffset": (-0.2, 0.2), "currentTimeConstant": (1e-5, 0.05)},
                            scale={"currentGain": nominal["currentGain"], "currentOffset": 0.01, "currentTimeConstant": 1e-3})
        rep["calibration"] = {"static": fits, "time_constant": tc, "joint_current": joint}
        tr = exp.get("truth", {})   # absent for recorded data: no recovery
        rec = []
        for name, f in (("voltageGain", fits["voltage"]), ("voltageOffset", fits["voltage"]),
                        ("currentGain", fits["current"]), ("currentOffset", fits["current"]),
                        ("currentTimeConstant", tc)):
            if name in tr:
                rec.append({"parameter": name, "truth": tr[name], "estimate": f["estimate"][name],
                            "standard_error": f["standard_error"].get(name),
                            **fitting.recovered(f["estimate"][name], tr[name], f["standard_error"].get(name))})
        rep["recovery"] = rec

        # Identifiability: amplifier gain and shunt resistance from terminal data.
        a3 = {"amplifierGain": 55.0, "shuntResistance": 5e-4, "shuntTempco": 0.0, "amplifierOffset": static["currentOffset"],
              "dividerTop": 17182.0, "dividerBottom": 1000.0, "voltageOffset": static["voltageOffset"]}
        idf = fitting.fit(residual_fn(models, A3, role("train", "current"), a3, "current"),
                          ["amplifierGain", "shuntResistance"], {"amplifierGain": 50.0, "shuntResistance": 6e-4},
                          {"amplifierGain": (1, 1000), "shuntResistance": (1e-5, 1e-2)},
                          scale={"amplifierGain": 55.0, "shuntResistance": 5e-4})
        prod = idf["estimate"]["amplifierGain"] * idf["estimate"]["shuntResistance"]
        rep["identifiability"] = {"shunt_and_gain": {
            "question": "amplifierGain and shuntResistance (A3) separately from terminal measurements?",
            "expected": "no: only their product, the current gain, is determined",
            "unidentifiable_directions": idf["unidentifiable_directions"],
            "relative_singular_values": idf["relative_singular_values"],
            "product_v_per_a": prod, "A1_current_gain": static["currentGain"],
            "flagged": bool(idf["unidentifiable_directions"])}}

        # Mismatch: a self-heating shunt fitted with the static model (a
        # design check on simulated data; optional for a recorded experiment).
        rep["mismatch"] = None
        m = exp.get("mismatch")
        if m:
            mt = {**a3, "amplifierOffset": tr.get("currentOffset", 0.0), **m["truth"]}
            mtrain = generate(models, A3, mt, m["train"], lg, out / "data", {**meta, "generator": A3})
            mhold = generate(models, A3, mt, m["holdout"], lg, out / "data", {**meta, "generator": A3})
            mf = fitting.fit(residual_fn(models, A1, [mtrain], nominal, "current"), ["currentGain", "currentOffset"],
                             {"currentGain": nominal["currentGain"], "currentOffset": 0.0},
                             {"currentGain": (0.5 * nominal["currentGain"], 1.5 * nominal["currentGain"]), "currentOffset": (-0.2, 0.2)},
                             scale={"currentGain": nominal["currentGain"], "currentOffset": 0.01})
            mp = {**static, **mf["estimate"]}
            rep["mismatch"] = {"generator": f"A3 with self-heating ({m['truth']})", "fitted": "A1",
                               "fit": mf["estimate"], "training_rms_v": mf["rms"],
                               "training_dwell_trend": dwell_trend(models, A1, mtrain, mp, "current"),
                               "holdout": evaluate(models, mhold, mp, thresholds["holdout"]),
                               "holdout_dwell_trend": dwell_trend(models, A1, mhold, mp, "current")}
            rep["mismatch"]["revealed"] = bool(rep["mismatch"]["training_dwell_trend"]["structured"]
                                               or not rep["mismatch"]["holdout"]["pass"])

        rep["holdout"] = [{"dataset": d.name, "calibrated": evaluate(models, d, static, thresholds["holdout"]),
                           "reference": evaluate(models, d, nominal, thresholds["holdout"])} for d in role("holdout")]

        prof = out / f"{e['name']}.profile.toml"
        write_profile(prof, exp, rep, fits)
        rep["profile"] = {"path": str(prof), "sha256": file_sha256(prof), "text": prof.read_text()}
        rep["profile_in_composer"] = reference_report(e["part"], str(prof))
    finally:
        models.close()
    (out / "report.json").write_text(json.dumps(rep, indent=1, default=str) + "\n")
    (out / "report.md").write_text(markdown(rep))
    return rep


def toolchain_check(models: Models) -> dict:
    checks = []
    c = lambda name, ok, detail: checks.append({"check": name, "pass": bool(ok), "detail": detail})
    tr = Trace(np.array([0.0, 0.01]), {"voltage": np.array([22.2, 22.2]), "current": np.array([0.0, 40.0]),
                                      "temperature": np.array([298.15, 298.15])})
    t = np.arange(1, 401) * 1e-4
    base = {"voltageGain": 1 / 18.182, "currentGain": 1 / 36.364, "currentOffset": 0.01}
    a1 = simulate(models[A1], tr, base, t, 1e-4)
    ok = abs(a1["currentSense"][-1] - (40 / 36.364 + 0.01)) < 1e-12 and abs(a1["voltageSense"][0] - 22.2 / 18.182) < 1e-12
    c("A1 static transfer", ok, f"current sense at 40 A {a1['currentSense'][-1]:.9f} V (expected {40/36.364+0.01:.9f})")
    clip = simulate(models[A1], Trace(np.array([0.0]), {"voltage": np.array([80.0]), "current": np.array([200.0]),
                                                      "temperature": np.array([298.15])}), base, t[:5], 1e-4)
    c("A1 clips at the rails", clip["currentSense"][0] == 3.3 and clip["voltageSense"][0] == 3.3,
      f"200 A -> {clip['currentSense'][0]} V, 80 V -> {clip['voltageSense'][0]} V (outputMax 3.3 V)")
    tau = 1e-3
    for h in (1e-4, 2.5e-5):
        a2 = simulate(models[A2], tr, {**base, "currentTimeConstant": tau}, t, h)
        after = t > 0.01
        exact = 0.01 + 40 / 36.364 * (1 - np.exp(-(t[after] - 0.01) / tau))
        err = float(np.max(np.abs(a2["currentSense"][after] - exact)))
        c(f"A2 first-order step against 1 - exp(-t/tau), step {h} s", err < 1e-3 * 40 / 36.364,
          f"max error {err:.2e} V on a {40/36.364:.3f} V step")
    s = models[A1]
    excite = Trace(np.array([0.0, 0.5]), {"voltage": np.array([12.0, 24.0]), "current": np.array([5.0, 50.0]),
                                         "temperature": np.array([280.0, 320.0])})
    tt = np.arange(1, 101) * 0.01
    ref = simulate(s, excite, {"currentGainTempco": 1e-3}, tt, 0.01)
    inert = []
    for k in [v.name for v in s.md.modelVariables if v.causality == "parameter"]:
        if k == "currentGainTempco":
            continue
        bump = 1.0 if "Max" in k else 0.5 if "Min" in k else 0.01 if "Offset" in k else 300.0 if "Temperature" in k else 0.03
        p = {"currentGainTempco": 1e-3, k: bump}
        d = simulate(s, excite, p, tt, 0.01)
        if all(np.array_equal(d[o], ref[o]) for o in OUTPUTS):
            inert.append(k)
    d = simulate(s, excite, {"currentGainTempco": 2e-3}, tt, 0.01)
    if np.array_equal(d["currentSense"], ref["currentSense"]):
        inert.append("currentGainTempco")
    c("every A1 parameter changes the output", not inert, "inert: " + ", ".join(inert) if inert else "all change it")
    again = simulate(s, excite, {"currentGainTempco": 1e-3}, tt, 0.01)
    c("reset: repeated runs agree exactly", all(np.array_equal(again[o], ref[o]) for o in OUTPUTS), "A1")
    for cls in (A2, A3):
        x = simulate(models[cls], excite, {}, tt, 0.01)
        y = simulate(models[cls], excite, {}, tt, 0.01)
        c(f"reset: {cls.rsplit('.', 1)[1]} repeats exactly", all(np.array_equal(x[o], y[o]) for o in OUTPUTS), "")
    a3 = simulate(models[A3], tr, {}, t, 1e-4)
    a1d = simulate(models[A1], tr, {"currentGain": 55 * 5e-4, "voltageGain": 1000 / 18182}, t, 1e-4)
    c("A3 at tempco 0 equals A1 with gain = amplifierGain * shuntResistance",
      np.max(np.abs(a3["currentSense"] - a1d["currentSense"])) < 1e-9 and np.max(np.abs(a3["voltageSense"] - a1d["voltageSense"])) < 1e-6,
      f"max difference {np.max(np.abs(a3['currentSense'] - a1d['currentSense'])):.1e} V")
    return {"checks": checks, "pass": all(x["pass"] for x in checks)}


def write_profile(path: Path, exp, rep, fits):
    e = exp["experiment"]
    L = ["# Calibration profile written by tools/calibration (cp_calibration.power).",
         "# Overlay it on the part: [peripherals.power_module] part = ..., level = \"W2\", calibration = \"<this file>\".", "",
         "[profile]", 'kind = "power_module"', f'part = "{e["part"]}"', 'level = "W2"', f'unit = "{e["unit"]}"',
         f'label = "{e["label"]}"', f'synthetic = {str(bool(e.get("synthetic"))).lower()}', f'created = "{rep["created"]}"',
         f'experiment = "{rep["experiment_file"]}"', f'experiment_sha256 = "{rep["experiment_sha256"]}"',
         f'library_revision = "{rep["versions"]["library_revision"]}"', ""]
    for n, d in rep["datasets"].items():
        L += [f"[datasets.{n}]", f'role = "{d["role"]}"', f'readings_sha256 = "{d["sha256"]}"',
              f'reference_sha256 = "{d["reference_sha256"]}"', f"synthetic = {str(d['synthetic']).lower()}", ""]
    L.append("[parameters]")
    for ch, f in fits.items():
        dsn = "+".join(n for n, d in rep["datasets"].items() if d["role"] == "train" and ch in
                       next(s for s in exp["datasets"] if s["name"] == n).get("tags", []))
        for name, v in f["estimate"].items():
            L.append(f'{name} = {{ value = {float(v)!r}, class = "measured", dataset = "{dsn}", '
                     f'how = "least squares through the A1 FMU", standard_error = {float(f["standard_error_conservative"][name])!r} }}')
    L += ["", "[not_representable_at_W2]",
          f'currentTimeConstant = "measured {rep["calibration"]["time_constant"]["estimate"]["currentTimeConstant"]!r} s (A2); W2 is static"', ""]
    path.write_text("\n".join(L))


def markdown(r: dict) -> str:
    L = []
    w = L.append
    e = r["experiment"]
    f = lambda x, n=4: "—" if x is None else (f"{x:.{n}g}" if isinstance(x, float) else str(x))
    w(f"# Calibration report: {e['name']}\n\n{e['description']}. Created {r['created']}; experiment "
      f"`{r['experiment_file']}`.\n")
    if e.get("synthetic"):
        w("> **Synthetic.** The data is model output plus a simulated logger. It tests the implementation and the "
          "experiment design, not a real power module. The reference configuration is the Holybro PM02 V3 as its "
          "setup scales describe it, not the lab's module.\n")
    v = r["versions"]
    w(f"Rumoca `{v['rumoca']}`, FMPy {v['fmpy']}, SciPy {v['scipy']}; library {v['library_revision'][:10]}.\n")
    w("## Toolchain\n\n| check | result | detail |\n|---|---|---|")
    for c in r["toolchain"]["checks"]:
        w(f"| {c['check']} | {'pass' if c['pass'] else '**FAIL**'} | {c['detail']} |")
    ref = r["reference"]
    w(f"\n## Reference profile ({ref['name']}, W2)\n\n| parameter | value | evidence | how |\n|---|---|---|---|")
    for k, p in ref["parameters"].items():
        if isinstance(p["value"], (int, float)):
            w(f"| `{k}` | {f(p['value'])} | {p.get('class', '')} | {p['how']} |")
    w("\nAssumed: " + "; ".join(ref["assumed"]) + ".\n")
    w("## Datasets\n\n| dataset | role | generator | rows | logger |\n|---|---|---|---|---|")
    for n, d in r["datasets"].items():
        w(f"| {n} | {d['role']} | {d['generator'].rsplit('.', 1)[-1]} | {d['rows']} | `{d['sha256'][:10]}…` |")
    th = r["thresholds"]
    w(f"\nThresholds `{th['file']}` (sha256 `{th['sha256'][:12]}…`), read before held-out evaluation: {th['values']['holdout']}.\n")
    c = r["calibration"]
    w("## Calibration\n\n**Stage 1, static (A1), least squares through the FMU.** Standard errors assuming independent "
      "residuals, and Newey-West (HAC) ones that allow for autocorrelation; the profile carries the larger.\n\n"
      "| parameter | estimate | se (independent) | se (HAC) | residual lag-1 |\n|---|---|---|---|---|")
    for ch in ("voltage", "current"):
        st = c["static"][ch]
        for k, x in st["estimate"].items():
            w(f"| `{k}` | {f(x, 7)} | {f(st['standard_error'].get(k), 3)} | {f(st.get('standard_error_hac', {}).get(k), 3)} | "
              f"{f(st.get('residual_lag1'), 2)} |")
    w(f"\nResidual rms: voltage {f(c['static']['voltage']['rms'], 3)} V, current {f(c['static']['current']['rms'], 3)} V "
      f"(simulated logger noise {f(r['logger_noise_v'], 3)} V).\n")
    tc = c["time_constant"]
    w(f"**Stage 2, output response (A2):** currentTimeConstant {f(tc['estimate']['currentTimeConstant'])} s "
      f"± {f(tc['standard_error'].get('currentTimeConstant'), 3)} from a 20 kHz step. Jointly with gain and offset: "
      + ", ".join(f"{k} {f(x, 6)}" for k, x in c["joint_current"]["estimate"].items()) + ".\n")
    w("## Same-model recovery (|z| ≤ 3)\n")
    if not r["recovery"]:
        w("Not applicable: the experiment states no generating truth (recorded data).\n")
    else:
        w("| parameter | truth | estimate | se | z | recovered |\n|---|---|---|---|---|---|")
    for x in r["recovery"]:
        w(f"| `{x['parameter']}` | {f(x['truth'], 6)} | {f(x['estimate'], 6)} | {f(x['standard_error'], 3)} | "
          f"{'' if x['z'] is None else format(x['z'], '+.2f')} | {'yes' if x['recovered'] else 'no'} |")
    i = r["identifiability"]["shunt_and_gain"]
    w(f"\n## Identifiability\n\n- {i['question']} Expected: {i['expected']}. "
      f"{'Flagged' if i['flagged'] else '**Not flagged**'}: {i['unidentifiable_directions']}. The product is determined: "
      f"{f(i['product_v_per_a'], 6)} V/A (A1 current gain {f(i['A1_current_gain'], 6)}).\n")
    m = r["mismatch"]
    if m is None:
        w("## Model mismatch\n\nNot run: the experiment has no `[mismatch]` design check.\n")
    else:
      tt, ht = m["training_dwell_trend"], m["holdout_dwell_trend"]
      w(f"## Model mismatch\n\nGenerator {m['generator']}; fitted {m['fitted']}. Training: residual trend within constant-current "
      f"dwells, max |z| {f(tt['max_z'], 3)}, p {f(tt['p_value'], 3)} → {'**structured**' if tt['structured'] else 'none found'}. "
      f"Held out (sustained current): current mean error {f(m['holdout']['current']['mean_error_v'], 3)} V, rms "
      f"{f(m['holdout']['current']['rms_error_v'], 3)} V → {'pass' if m['holdout']['pass'] else '**fails the thresholds**'}; "
      f"dwell trend p {f(ht['p_value'], 3)}. Unexplained behaviour revealed: {'yes' if m['revealed'] else 'no'}.\n")
    w("## Held-out comparison\n\n| dataset | profile | voltage mean / rms (mV) | current mean / rms (mV) | pass |\n|---|---|---|---|---|")
    for h in r["holdout"]:
        for p in ("calibrated", "reference"):
            x = h[p]
            w(f"| {h['dataset']} | {p} | {x['voltage']['mean_error_v']*1e3:+.3f} / {x['voltage']['rms_error_v']*1e3:.3f} | "
              f"{x['current']['mean_error_v']*1e3:+.3f} / {x['current']['rms_error_v']*1e3:.3f} | {'pass' if x['pass'] else 'fail'} |")
    w(f"\n## Profile\n\nThe composer accepts it: evidence **{r['profile_in_composer']['evidence']}**.\n\n```toml\n{r['profile']['text'].strip()}\n```\n")
    w("## Limitations\n")
    for x in (["All data is synthetic; no property of a real module is measured or validated here."] if e.get("synthetic")
              else ["Recorded data: the results hold for this unit, logger and conditions only."]) + [
              "The zero-current offset is not stated by the reference's documentation: the reference profile assumes 0.",
              "The logger is simulated as white Gaussian noise and quantization; a real logger's bandwidth, offset and "
              "clock must be characterized and enter the thresholds.",
              "W2 is static: a measured time constant is reported but not representable in the vehicle level.",
              "Separate shunt resistance and amplifier gain need a component measurement; terminal data gives only "
              "their product.",
              "Self-heating and temperature effects are behaviours of A3 used for the mismatch test; their parameters "
              "(tempco, thermal resistance and capacitance) are assumed, not from a datasheet."]:
        w(f"- {x}")
    return "\n".join(L) + "\n"
