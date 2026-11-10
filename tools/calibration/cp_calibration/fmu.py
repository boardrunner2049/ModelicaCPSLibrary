"""Export library models to FMI 3 FMUs and run them as co-simulation slaves.

Structural parameters (those that size arrays, e.g. the barometer's
`latencySamples`) are evaluated by Rumoca at compile time and cannot be set in
the FMU, so each structural variant is its own export: a two-line wrapper
`extends <class>(<structural modifiers>)`. Exports are cached under
build/calibration/fmus by a key over the library sources, the wrapper and the
compiler binary.
"""
from __future__ import annotations

import hashlib
import json
import os
import shutil
import subprocess
import sys
import tempfile
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

LIBRARY_ROOT = Path(__file__).resolve().parents[3]
MAKE_FMU = LIBRARY_ROOT / "tools/fmi/tools/make_fmu.py"
CACHE = LIBRARY_ROOT / "build/calibration/fmus"
SOURCE_DIRS = ("Systems", "Interfaces", "Utilities", "Worlds")


def _rumoca() -> Path:
    pinned = LIBRARY_ROOT / "tools/fmi/bin/rumoca"
    if os.environ.get("RUMOCA"):
        return Path(os.environ["RUMOCA"])
    if pinned.exists():
        return pinned
    found = shutil.which("rumoca")
    if not found:
        raise RuntimeError("no Rumoca: put it at tools/fmi/bin/rumoca, on PATH, or in RUMOCA")
    return Path(found)


def file_sha256(path: Path) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def sources_digest() -> str:
    """Digest of every library Modelica source the sensor models can depend on."""
    h = hashlib.sha256()
    for d in SOURCE_DIRS:
        for p in sorted((LIBRARY_ROOT / d).rglob("*.mo")):
            h.update(str(p.relative_to(LIBRARY_ROOT)).encode())
            h.update(p.read_bytes())
    h.update((LIBRARY_ROOT / "package.mo").read_bytes())
    return h.hexdigest()


def toolchain() -> dict:
    """Versions of everything that determines an export and a run."""
    import fmpy
    import scipy

    rumoca = _rumoca()
    version = subprocess.run([str(rumoca), "--version"], capture_output=True, text=True).stdout.strip()
    rev = subprocess.run(["git", "-C", str(LIBRARY_ROOT), "rev-parse", "HEAD"],
                         capture_output=True, text=True).stdout.strip()
    dirty = bool(subprocess.run(["git", "-C", str(LIBRARY_ROOT), "status", "--porcelain", "--",
                                 *SOURCE_DIRS, "tools/calibration", "parts"],
                                capture_output=True, text=True).stdout.strip())
    return {
        "rumoca": version, "rumoca_sha256": file_sha256(rumoca), "rumoca_target": "fmi3 (Co-Simulation)",
        "fmpy": fmpy.__version__, "scipy": scipy.__version__, "numpy": np.__version__,
        "python": sys.version.split()[0], "library_revision": rev, "library_dirty": dirty,
        "library_sources_sha256": sources_digest(),
    }


def wrapper_source(name: str, cls: str, structural: dict | None) -> str:
    mods = ",".join(f"{k}={_literal(v)}" for k, v in sorted((structural or {}).items()))
    extends = f"ModelicaCPSLibrary.{cls}" + (f"({mods})" if mods else "")
    return f"model {name}\n  extends {extends};\nend {name};\n"


def _literal(v) -> str:
    if isinstance(v, bool):
        return "true" if v else "false"
    return repr(v)


def export(cls: str, structural: dict | None = None, *, wrapper: Path | None = None,
           wrapper_model: str | None = None) -> Path:
    """Export `cls` (without the ModelicaCPSLibrary prefix) with structural
    modifiers, or a wrapper model file; returns the cached FMU path."""
    if wrapper is not None:
        text = wrapper.read_text()
        model = wrapper_model or wrapper.stem
    else:
        model = "Calibrated_" + cls.replace(".", "_")
        text = wrapper_source(model, cls, structural)
    key = hashlib.sha256("\0".join([text, model, sources_digest(), file_sha256(_rumoca())]).encode()).hexdigest()[:16]
    out = CACHE / f"{model}-{key}.fmu"
    if out.exists():
        return out
    CACHE.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory() as tmp:
        src = Path(tmp) / f"{model}.mo"
        src.write_text(text)
        env = dict(os.environ)
        env["PATH"] = ":".join(p for p in env.get("PATH", "").split(":") if p != str(Path.home() / "bin"))
        run = subprocess.run([sys.executable, str(MAKE_FMU), "unused", str(out), "--wrapper", str(src),
                              "--wrapper-model", model], capture_output=True, text=True, env=env)
        if run.returncode or not out.exists():
            raise RuntimeError(f"Rumoca export of {model} failed:\n{run.stderr}{run.stdout}")
    (out.with_suffix(".json")).write_text(json.dumps({"model": model, "source": text, **toolchain()}, indent=1))
    return out


@dataclass
class Trace:
    """Piecewise-constant inputs: value `values[name][k]` holds from `time[k]`
    until `time[k+1]` (a step at t is the row starting at t)."""
    time: np.ndarray
    values: dict[str, np.ndarray]

    def at(self, t: float) -> dict[str, float]:
        k = max(int(np.searchsorted(self.time, t + 1e-12, side="right")) - 1, 0)
        return {n: float(v[k]) for n, v in self.values.items()}

    @property
    def breakpoints(self) -> np.ndarray:
        return self.time


@dataclass
class Result:
    time: np.ndarray                       # communication points
    outputs: dict[str, np.ndarray]          # every requested output at each point
    settings: dict = field(default_factory=dict)

    def deliveries(self, sequence: str = "sequence") -> dict[str, np.ndarray]:
        """The rows at which `sequence` increments: one per delivered reading
        when the communication step is no longer than the sample period."""
        seq = self.outputs[sequence]
        idx = np.flatnonzero(np.diff(np.concatenate([[0], seq])) > 0)
        jumps = np.diff(np.concatenate([[0], seq]))[idx]
        out = {n: v[idx] for n, v in self.outputs.items()}
        out["time"] = self.time[idx]
        out["missed"] = jumps - 1
        return out


class Slave:
    """One FMU instance reused across runs: reset, set parameters, initialize,
    step with zero-order-hold inputs (FMI 3 Co-Simulation)."""

    def __init__(self, fmu: Path):
        from fmpy import extract, read_model_description
        from fmpy.fmi3 import FMU3Slave

        self.path = Path(fmu)
        self.md = read_model_description(str(fmu))
        self.unzip = extract(str(fmu))
        self.vars = {v.name: v for v in self.md.modelVariables}
        self.slave = FMU3Slave(guid=self.md.guid, unzipDirectory=self.unzip,
                               modelIdentifier=self.md.coSimulation.modelIdentifier, instanceName="calibration")
        self.slave.instantiate()
        self.fresh = True

    def tunable(self) -> list[str]:
        return [v.name for v in self.md.modelVariables
                if v.causality == "parameter" and v.variability in ("tunable", "fixed")]

    def fixed(self) -> list[str]:
        return [v.name for v in self.md.modelVariables if v.causality == "calculatedParameter"]

    def _set(self, name: str, value):
        v = self.vars.get(name)
        if v is None:
            raise KeyError(f"{name} is not a variable of {self.md.modelName}")
        if v.causality not in ("parameter", "input"):
            raise KeyError(f"{name} is {v.causality} ({v.variability}): it cannot be set"
                           + (" — structural, export a variant instead" if v.causality == "calculatedParameter" else ""))
        if v.type == "Float64":
            self.slave.setFloat64([v.valueReference], [float(value)])
        elif v.type == "Int32":
            self.slave.setInt32([v.valueReference], [int(value)])
        elif v.type == "Boolean":
            self.slave.setBoolean([v.valueReference], [bool(value)])
        else:
            raise TypeError(f"{name}: {v.type}")

    def _get(self, names: list[str]) -> list[float]:
        types = {self.vars[n].type for n in names}
        if types <= {"Float64", "Int32", "Boolean"} and len(names) > 1:
            got = {}
            for t, fn in (("Float64", self.slave.getFloat64), ("Int32", self.slave.getInt32),
                          ("Boolean", self.slave.getBoolean)):
                group = [n for n in names if self.vars[n].type == t]
                if group:
                    got.update(zip(group, fn([self.vars[n].valueReference for n in group])))
            return [float(got[n]) for n in names]
        out = []
        for n in names:
            v = self.vars[n]
            if v.type == "Float64":
                out.append(self.slave.getFloat64([v.valueReference])[0])
            elif v.type == "Int32":
                out.append(float(self.slave.getInt32([v.valueReference])[0]))
            elif v.type == "Boolean":
                out.append(float(self.slave.getBoolean([v.valueReference])[0]))
        return out

    def run(self, trace: Trace, parameters: dict, outputs: list[str], stop: float, step: float,
            start_inputs: dict | None = None) -> Result:
        """Simulate from 0 to `stop`. Communication points are the multiples of
        `step` plus every trace breakpoint, so a step input changes exactly at
        its time; an acquisition at a communication point uses the inputs held
        over the step before it."""
        if not self.fresh:
            self.slave.reset()
        self.fresh = False
        for n, v in parameters.items():
            self._set(n, v)
        first = trace.at(0.0)
        for n, v in {**first, **(start_inputs or {})}.items():
            self._set(n, v)
        self.slave.enterInitializationMode(startTime=0.0, stopTime=stop)
        self.slave.exitInitializationMode()
        grid = np.arange(0.0, stop + step / 2, step)
        points = np.unique(np.round(np.concatenate([grid, trace.breakpoints[(trace.breakpoints > 0)
                                                                            & (trace.breakpoints < stop)]]), 12))
        rows = [self._get(outputs)]
        for t0, t1 in zip(points[:-1], points[1:]):
            for n, v in trace.at(t0).items():
                self._set(n, v)
            self.slave.doStep(currentCommunicationPoint=t0, communicationStepSize=t1 - t0)
            rows.append(self._get(outputs))
        data = np.array(rows)
        return Result(points, {n: data[:, j] for j, n in enumerate(outputs)},
                      {"stop": stop, "step": step, "communication_points": len(points),
                       "parameters": dict(parameters), "fmu": self.path.name})

    def close(self):
        try:
            self.slave.terminate()
        finally:
            self.slave.freeInstance()
