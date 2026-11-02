#!/usr/bin/env python3
"""Run native OpenModelica physics assertions and numerical integration regressions.

Requires omc and Modelica 4.0.0. Artifacts and a machine-readable report go to
build/verification. This is not a Rumoca or FMU test; use probe_export.py separately.
"""
import argparse
import csv
import json
import math
import re
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def quoted(value):
    return json.dumps(str(value))


def experiment(path):
    text = path.read_text()
    def number(name, default):
        match = re.search(rf"\b{name}\s*=\s*([0-9.eE+-]+)", text)
        return float(match.group(1)) if match else default
    stop = number("StopTime", 1)
    return stop, number("Tolerance", 1e-8), max(50, math.ceil(stop/number("Interval", 0.002)))


def final_row(path):
    with path.open() as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise AssertionError(f"Empty result: {path}")
    return rows[-1]


def run(omc, out):
    out.mkdir(parents=True, exist_ok=True)
    (out/"report.json").unlink(missing_ok=True)
    cases = []
    for path in sorted((ROOT/"Tests").glob("*.mo")):
        if path.stem in {"package", "ExportOcto", "ChassisRejectedConfigurations"}:
            continue
        cases.append((f"ModelicaCPSLibrary.Tests.{path.stem}", *experiment(path)))
    for name in ("QuadHover", "QuadImuOnly", "HexaHover", "OctoHover", "CoaxialHover"):
        stop = 1
        cases.append((f"ModelicaCPSLibrary.Examples.{name}", stop, 1e-8, int(stop/0.002)))
    script = ['loadModel(Modelica,{"4.0.0"});', f'loadFile({quoted(ROOT/"package.mo")});',
              'getErrorString();']
    for name, stop, tol, intervals in cases:
        short = name.split(".")[-1]
        script.append(f'simulate({name},stopTime={stop},tolerance={tol},numberOfIntervals={intervals},'
                      f'outputFormat="csv",fileNamePrefix="{short}");')
        script.append('getErrorString();')
    script_path = out/"verify.mos"
    script_path.write_text("\n".join(script)+"\n")
    # Avoid accepting stale results if a later compiler run fails.
    for name, *_ in cases:
        (out/(name.split(".")[-1]+"_res.csv")).unlink(missing_ok=True)
    result = subprocess.run([omc, str(script_path)], cwd=out, text=True,
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=300)
    (out/"native.log").write_text(result.stdout)
    successes = result.stdout.count("The simulation finished successfully.")
    if result.returncode or "Error:" in result.stdout or successes != len(cases):
        raise AssertionError(f"Native run: {successes}/{len(cases)} passed; see {out/'native.log'}")
    metrics = {}
    for name in ("QuadHover", "HexaHover", "OctoHover", "CoaxialHover"):
        row = final_row(out/(name+"_res.csv"))
        altitude = float(row["vehicle.body.p_w[3]"])
        acc = float(row["vehicle.measurements.acceleration[3]"])
        assert abs(altitude+1)<1e-7 and abs(acc-9.80665)<1e-7, name
        metrics[name] = {"altitude_m": -altitude, "specific_force_up_mps2": acc}
    row = final_row(out/"ContactSettlingTakeoff_res.csv")
    metrics["landing_gear"] = {
        name: {"settled_altitude_m": float(row[name+".altitude"]),
               "normal_load_N": float(row[name+".normalLoad"]),
               "contact_count": int(float(row[name+".contactCount"])),
               "acceleration_frd_z_mps2": float(row[name+".acceleration_frd[3]"])}
        for name in ("quad", "hexa")
    }
    rejected = {
        "ImproperRotation": "right-handed rotation",
        "IndefiniteInertia": "physical mass distribution",
        "NonphysicalPrincipalMoments": "physical mass distribution",
        "MasslessInertia": "zero-mass part cannot carry nonzero mass inertia",
    }
    for name, diagnostic in rejected.items():
        mos = out/("reject_"+name+".mos")
        mos.write_text('loadModel(Modelica,{"4.0.0"});\n'
                       f'loadFile({quoted(ROOT/"package.mo")});\n'
                       f'simulate(ModelicaCPSLibrary.Tests.ChassisRejectedConfigurations.{name},'
                       f'stopTime=0.01,fileNamePrefix="reject_{name}");\ngetErrorString();\n')
        proc = subprocess.run([omc, str(mos)], cwd=out, text=True,
                              stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=60)
        (out/("reject_"+name+".log")).write_text(proc.stdout)
        if diagnostic not in proc.stdout or "The simulation finished successfully." in proc.stdout:
            raise AssertionError(f"Expected invalid configuration rejection: {name}")
    version = subprocess.check_output([omc,"--version"], text=True).strip()
    report = {"compiler": version, "target": "OpenModelica native DASSL",
              "passed_models": [c[0] for c in cases], "expected_rejections": list(rejected),
              "core_contract": "ned_frd_continuous_v1", "metrics": metrics,
              "warnings": [line for line in result.stdout.splitlines() if "Warning:" in line]}
    (out/"report.json").write_text(json.dumps(report, indent=2)+"\n")
    print(f"PASS: {len(cases)} simulations, {len(rejected)} invalid configurations rejected")
    print(out/"report.json")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--omc", default="omc")
    parser.add_argument("--output", type=Path, default=ROOT/"build/verification")
    args = parser.parse_args()
    run(args.omc, args.output.resolve())
