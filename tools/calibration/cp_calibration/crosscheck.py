"""Cross-check the new sensor models between the two compilers.

Every calibration result in this harness comes from Rumoca-exported FMUs. This
module simulates the same self-contained test models (stimulus inside the
model, no inputs) with OpenModelica and compares the outputs, so a result does
not rest on one compiler's code generation alone. It covers the power-module
levels A1-A3, the pulse RPM sensor E1 (count, edge times including the seeded
jitter draws, rpm) and the barometer timing contract.

    build/venv-calib/bin/python -m cp_calibration crosscheck [--out DIR]

OpenModelica (omc on PATH) loads a copy of the library's Modelica sources in
a directory named ModelicaCPSLibrary.
"""
from __future__ import annotations

import csv
import json
import os
import shutil
import subprocess
import tempfile
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from .fmu import LIBRARY_ROOT, Slave, Trace, export, toolchain

L = "ModelicaCPSLibrary"


@dataclass
class Case:
    name: str
    source: str                     # a model with no inputs
    stop: float
    step: float                     # FMU communication step and OMC output interval
    continuous: list[str]           # compared at every grid point (after events there)
    discrete: list[str] = field(default_factory=list)   # compared exactly
    arrays: dict[str, int] = field(default_factory=dict)  # compared at the final time
    rtol: float = 1e-6
    atol: float = 1e-9
    note: str = ""


CASES = [
    Case("power_a1_static", f"""
model XcPowerA1
  output Real voltageSense; output Real currentSense;
  {L}.Systems.Sensing.PowerModule.AnalogSensor s(voltageGain=0.055, voltageOffset=0.004,
    currentGain=0.0275, currentOffset=0.012);
equation
  s.voltage = 14 + 2.8*time;
  s.current = if time < 0.5 then 2 else 45 + 30*sin(6*time);
  voltageSense = s.voltageSense; currentSense = s.currentSense;
end XcPowerA1;""", 2.0, 0.01, ["voltageSense", "currentSense"]),
    Case("power_a2_first_order", f"""
model XcPowerA2
  output Real voltageSense; output Real currentSense;
  {L}.Systems.Sensing.PowerModule.FirstOrderAnalogSensor s(voltageGain=0.055, currentGain=0.0275,
    currentOffset=0.012, voltageTimeConstant=0.004, currentTimeConstant=0.012);
equation
  s.voltage = if time < 0.05 then 15 else 16.4;
  s.current = if time < 0.08 then 3 else 60;
  voltageSense = s.voltageSense; currentSense = s.currentSense;
end XcPowerA2;""", 0.2, 0.001, ["voltageSense", "currentSense"], rtol=1e-5, atol=1e-7,
         note="step inputs at 0.05 s and 0.08 s, both communication points"),
    Case("power_a3_self_heating", f"""
model XcPowerA3
  output Real voltageSense; output Real currentSense; output Real shuntTemperature;
  {L}.Systems.Sensing.PowerModule.ShuntAnalogSensor s(shuntTempco=0.004, thermalResistance=25,
    thermalCapacitance=1.5, amplifierOffset=0.01, temperature=288.15);
equation
  s.voltage = 16.8 - 0.02*time;
  s.current = if time < 5 then 1 else 80;
  voltageSense = s.voltageSense; currentSense = s.currentSense;
  shuntTemperature = s.shuntTemperature;
end XcPowerA3;""", 120.0, 0.5, ["voltageSense", "currentSense", "shuntTemperature"], rtol=1e-5, atol=1e-7),
    Case("rpm_e1_ramp_jitter", f"""
model XcRpm
  output Integer count; output Real lastEdgeTime; output Real rpm; output Real edgeTimes[16];
  {L}.Systems.Sensing.Rpm.PulseSensor s(bufferSize=16, pulsesPerRevolution=12, phaseOffset=0.3,
    edgeDelay=2e-5, edgeJitterStd=3e-6, seed=7);
equation
  s.speed = 200 + 150*time;
  count = s.count; lastEdgeTime = s.lastEdgeTime; rpm = s.rpm; edgeTimes = s.edgeTimes;
end XcRpm;""", 1.0, 0.01, ["lastEdgeTime", "rpm"], ["count"], {"edgeTimes": 16}, rtol=1e-6, atol=1e-8,
         note="edge times are state events; agreement is bounded by the two compilers' event location"),
    Case("barometer_timing", f"""
model XcBaro
  output Real pressure; output Real sampleTime; output Real deliveryTime; output Integer sequence;
  {L}.Systems.Sensing.Barometer.SampledSensor s(seed=3, pressureNoiseStd=2.5, pressureDriftStd=4,
    pressureDriftTau=20, temperatureNoiseStd=0.05, pressureQuantum=0.06);
equation
  s.ambientPressure = 101325 - 120*time; s.ambientTemperature = 293.15;
  s.altitude_w = 10*time; s.climbRate_w = 10;
  pressure = s.pressure; sampleTime = s.sampleTime; deliveryTime = s.deliveryTime; sequence = s.sequence;
end XcBaro;""", 2.0, 0.004, ["pressure", "sampleTime", "deliveryTime"], ["sequence"], rtol=1e-9, atol=1e-6,
         note="white noise, drift and quantization on: the seeded draws must agree"),
    Case("sampler_noise_drift", f"""
model XcSampler
  output Real y1; output Real y2; output Integer sequence;
  {L}.Systems.Peripherals.Sampler s(n=2, samplePeriod=0.05, seed=11, noiseStd={{0.2, 0.01}},
    noiseProportional={{0.01, 0}}, driftStd={{0.5, 0.02}}, driftTau={{3, 10}}, bias={{0.1, 0}}, scale={{0.02, 0}});
equation
  s.u = {{20*sin(time), 3 + time}};
  y1 = s.y[1]; y2 = s.y[2]; sequence = s.sequence;
end XcSampler;""", 3.0, 0.01, ["y1", "y2"], ["sequence"], rtol=1e-9, atol=1e-9,
         note="the generic peripheral chain (airspeed, rangefinder, temperature and others)"),
    Case("magnetometer_noise_drift", f"""
model XcMag
  output Real m1; output Real m2; output Real m3; output Integer sequence;
  {L}.Systems.Sensing.Magnetometer.SampledSensor s(seed=5, noiseStd=fill(3e-7, 3), biasStd=fill(5e-7, 3),
    biasTau=fill(4, 3), bias={{1e-6, -2e-6, 0}});
equation
  s.R_wb = identity(3);
  s.magneticField_w = {{2e-5 + 1e-6*time, 1e-6, 4.5e-5}};
  m1 = s.magneticField[1]; m2 = s.magneticField[2]; m3 = s.magneticField[3]; sequence = s.sequence;
end XcMag;""", 2.0, 0.01, ["m1", "m2", "m3"], ["sequence"], rtol=1e-9, atol=1e-12),
]


def _omc(case: Case, work: Path) -> dict[str, np.ndarray]:
    lib = work.parent / L
    if not lib.exists():
        # OMC resolves symbolic links and requires the directory to carry the
        # package name, so it loads a copy of the Modelica sources.
        shutil.copytree(LIBRARY_ROOT, lib, symlinks=True,
                        ignore=lambda d, names: [n for n in names if Path(d) == LIBRARY_ROOT
                                                 and n in ("build", "tools", "third_party", ".git", "runtime")])
    model = case.source.split()[1]
    (work / f"{model}.mo").write_text(case.source.strip() + "\n")
    names = case.continuous + case.discrete + [f"{a}\\[{i}\\]" for a, n in case.arrays.items() for i in range(1, n + 1)]
    n = int(round(case.stop / case.step))
    mos = (f'loadModel(Modelica,{{"4.0.0"}}); loadFile("../{L}/package.mo"); loadFile("{model}.mo");\n'
           f'res := simulate({model}, stopTime={case.stop}, numberOfIntervals={n}, tolerance=1e-10, '
           f'outputFormat="csv", variableFilter="time|{"|".join(names)}"); print(res.messages); getErrorString();\n')
    (work / "run.mos").write_text(mos)
    env = dict(os.environ)
    env["PATH"] = ":".join(p for p in env.get("PATH", "").split(":") if p != str(Path.home() / "bin"))
    r = subprocess.run(["omc", "run.mos"], cwd=work, capture_output=True, text=True, env=env, timeout=1800)
    out = work / f"{model}_res.csv"
    if not out.exists():
        raise RuntimeError(f"OpenModelica failed for {case.name}:\n{r.stdout[-3000:]}{r.stderr[-2000:]}")
    with open(out) as f:
        rows = list(csv.reader(f))
    head = [h.strip('"') for h in rows[0]]
    data = np.array([[float(x) for x in row] for row in rows[1:]])
    return {h: data[:, j] for j, h in enumerate(head)}


def _at_grid(cols: dict, grid: np.ndarray) -> dict[str, np.ndarray]:
    """OMC writes event points twice (before and after); take the last row at
    or before each grid time, which is the post-event value the FMU reports."""
    t = cols["time"]
    k = np.searchsorted(t, grid + 1e-12, side="right") - 1
    return {n: v[k] for n, v in cols.items()}


def _fmu(case: Case) -> dict[str, np.ndarray]:
    model = case.source.split()[1]
    with tempfile.TemporaryDirectory() as tmp:
        p = Path(tmp) / f"{model}.mo"
        p.write_text(case.source.strip() + "\n")
        s = Slave(export("x", wrapper=p, wrapper_model=model))
    try:
        outs = case.continuous + case.discrete + list(case.arrays)
        r = s.run(Trace(np.array([0.0]), {}), {}, outs, case.stop, case.step)
        return {"time": r.time, **{n: np.asarray(r.outputs[n]) for n in outs}}
    finally:
        s.close()


def compare(case: Case, work: Path) -> dict:
    f = _fmu(case)
    o = _at_grid(_omc(case, work), f["time"])
    rep = {"case": case.name, "points": int(len(f["time"])), "stop_s": case.stop, "step_s": case.step,
           "rtol": case.rtol, "atol": case.atol, "note": case.note, "channels": {}}
    ok = True
    for n in case.continuous:
        d = np.abs(f[n] - o[n])
        lim = case.atol + case.rtol * np.abs(o[n])
        good = bool(np.all(d <= lim))
        rep["channels"][n] = {"max_abs_diff": float(d.max()), "max_rel_diff": float(np.max(d / np.maximum(np.abs(o[n]), 1e-300))),
                              "pass": good}
        ok &= good
    for n in case.discrete:
        mism = int(np.sum(f[n] != o[n]))
        rep["channels"][n] = {"mismatched_points": mism, "final_fmu": float(f[n][-1]), "final_omc": float(o[n][-1]),
                              "pass": mism == 0}
        ok &= mism == 0
    for a, size in case.arrays.items():
        fa = np.asarray(f[a][-1], float)
        oa = np.array([o[f"{a}[{i}]"][-1] for i in range(1, size + 1)])
        d = np.abs(fa - oa)
        good = bool(np.all(d <= case.atol + case.rtol * np.abs(oa)))
        rep["channels"][f"{a} (final)"] = {"max_abs_diff": float(d.max()), "pass": good}
        ok &= good
    rep["pass"] = bool(ok)
    return rep


def run(out: Path | None = None) -> dict:
    out = out or LIBRARY_ROOT / "build/calibration/crosscheck"
    out.mkdir(parents=True, exist_ok=True)
    ver = subprocess.run(["omc", "--version"], capture_output=True, text=True).stdout.strip()
    reports = []
    with tempfile.TemporaryDirectory() as tmp:
        for case in CASES:
            work = Path(tmp) / case.name
            work.mkdir()
            reports.append(compare(case, work))
    rep = {"toolchain": {**toolchain(), "openmodelica": ver}, "cases": reports,
           "pass": all(r["pass"] for r in reports)}
    (out / "crosscheck.json").write_text(json.dumps(rep, indent=1))
    (out / "crosscheck.md").write_text(markdown(rep))
    return rep


def markdown(rep: dict) -> str:
    lines = ["# Rumoca / OpenModelica cross-check", "",
             f"Rumoca {rep['toolchain'].get('rumoca', '?')}, {rep['toolchain']['openmodelica']}.", "",
             "| case | channel | result | detail |", "|---|---|---|---|"]
    for c in rep["cases"]:
        for n, ch in c["channels"].items():
            detail = (f"max |Δ| {ch['max_abs_diff']:.3g}" if "max_abs_diff" in ch
                      else f"{ch['mismatched_points']} mismatched of {c['points']}; final {ch['final_fmu']:g}/{ch['final_omc']:g}")
            lines.append(f"| {c['case']} | {n} | {'pass' if ch['pass'] else 'FAIL'} | {detail} |")
    lines += ["", f"Overall: {'pass' if rep['pass'] else 'FAIL'}.", ""]
    return "\n".join(lines)
