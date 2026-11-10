"""The B1 barometer (Systems.Sensing.Barometer.SampledSensor) in the harness.

Deterministic parameters are fitted by least squares through the FMU itself,
with every stochastic effect off; the integer, structural `latencySamples` by
comparing the fitted cost over exported variants. Stochastic parameters are
fitted to the residuals of the deterministic fit (stochastic.py), and the
offset re-estimated jointly with them.
"""
from __future__ import annotations

import json
import subprocess
import tempfile
from pathlib import Path

import numpy as np
from scipy import optimize

from . import stochastic
from .dataset import Dataset
from .fmu import LIBRARY_ROOT, Slave, Trace, export

CLASS = "Systems.Sensing.Barometer.SampledSensor"
OUTPUTS = ["pressure", "temperature", "altitude", "sampleTime", "deliveryTime", "sequence", "valid", "pressureDrift"]
# The deterministic stage: noise, drift and quantization off. Quantization is
# deterministic, but it makes the cost a step function of the offset; the
# stochastic stage carries it as white noise of variance q^2/12 instead.
STOCHASTIC_OFF = {"pressureNoiseStd": 0.0, "pressureDriftStd": 0.0, "temperatureNoiseStd": 0.0,
                  "pressureQuantum": 0.0, "temperatureQuantum": 0.0}
STRUCTURAL = ("latencySamples",)
COMPOSE = LIBRARY_ROOT / "tools/fire-compose/target/debug/fire-compose"


def datasheet_report(part: str, mode: str | None = None, drift: str | None = None,
                     calibration: str | None = None, extra: dict | None = None) -> dict:
    """The composer's own conversion of a part (`fire-compose part --json`):
    the datasheet-derived parameters, each with its evidence class."""
    if not COMPOSE.exists():
        subprocess.run(["cargo", "build", "-q", "--manifest-path", str(LIBRARY_ROOT / "tools/fire-compose/Cargo.toml")],
                       check=True)
    cmd = [str(COMPOSE), "part", "barometer", part, "--json"]
    for flag, v in (("--mode", mode), ("--drift", drift), ("--calibration", calibration)):
        if v:
            cmd += [flag, v]
    if extra:
        cmd += ["--set", "\n".join(f"{k} = {json.dumps(v)}" for k, v in extra.items())]
    run = subprocess.run(cmd, capture_output=True, text=True, cwd=LIBRARY_ROOT)
    if run.returncode:
        raise RuntimeError(run.stderr.strip())
    return json.loads(run.stdout)


def parameters_of(report: dict) -> dict:
    return {k: v["value"] for k, v in report["parameters"].items() if isinstance(v["value"], (int, float))}


class Model:
    """The barometer FMU, one export per structural variant."""

    def __init__(self, cls: str = CLASS, wrapper: Path | None = None, wrapper_model: str | None = None):
        self.cls, self.wrapper, self.wrapper_model = cls, wrapper, wrapper_model
        self._slaves: dict[tuple, Slave] = {}

    def slave(self, structural: dict) -> Slave:
        key = tuple(sorted(structural.items()))
        if key not in self._slaves:
            if self.wrapper:
                text = self.wrapper.read_text()
                for k, v in structural.items():
                    text = text.replace(f"{k}=0", f"{k}={v}")
                # Outside the library tree: Rumoca reads every .mo file under it.
                with tempfile.TemporaryDirectory() as d:
                    tmp = Path(d) / f"{self.wrapper_model}.mo"
                    tmp.write_text(text)
                    fmu = export(self.cls, wrapper=tmp, wrapper_model=self.wrapper_model)
            else:
                fmu = export(self.cls, structural)
            self._slaves[key] = Slave(fmu)
        return self._slaves[key]

    def simulate(self, trace: Trace, params: dict, stop: float, step: float | None = None) -> dict:
        structural = {k: int(params.get(k, 0)) for k in STRUCTURAL}
        tunable = {k: v for k, v in params.items() if k not in STRUCTURAL}
        period = float(params.get("samplePeriod", 0.02))
        s = self.slave(structural)
        step = step or period / 2
        outs = [o for o in OUTPUTS if o in s.vars]
        r = s.run(trace, tunable, outs, stop, step)
        d = r.deliveries()
        d["settings"] = {**r.settings, "structural": structural}
        return d

    def close(self):
        for s in self._slaves.values():
            s.close()


def held(deliveries: dict, t_obs: np.ndarray, name: str) -> np.ndarray:
    """What a consumer reading at `t_obs` sees: the latest reading delivered at
    or before that time (NaN before the first)."""
    k = np.searchsorted(deliveries["deliveryTime"], t_obs + 1e-9, side="right") - 1
    out = np.full(len(t_obs), np.nan)
    ok = k >= 0
    out[ok] = deliveries[name][k[ok]]
    return out


def residuals(model: Model, ds: Dataset, params: dict, channel: str = "pressure") -> np.ndarray:
    col = {"pressure": "pressure_pa", "temperature": "temperature_k", "altitude": "altitude_m"}[channel]
    t = ds.readings["time_s"]
    d = model.simulate(ds.trace(), params, float(t[-1]) + 1e-9)
    return held(d, t, channel) - ds.readings[col]


def fit_deterministic(model: Model, datasets: list[Dataset], base: dict, names: list[str],
                      x0: dict, bounds: dict, channels: list[str] = ("pressure",),
                      scale: dict | None = None) -> dict:
    """Bounded least squares over `names` with stochastic effects off, through
    the FMU. Reports the estimate, the Jacobian's singular values and, when
    they show a direction the data cannot resolve, that combination."""
    p0 = {**base, **STOCHASTIC_OFF}
    names = list(names)
    sc = np.array([(scale or {}).get(n, 1.0) for n in names])

    def resid(z):
        p = {**p0, **dict(zip(names, z * sc))}
        r = np.concatenate([residuals(model, ds, p, ch) for ds in datasets for ch in channels])
        return np.nan_to_num(r[np.isfinite(r)], nan=0.0)

    z0 = np.array([x0[n] for n in names]) / sc
    lb = np.array([bounds[n][0] for n in names]) / sc
    ub = np.array([bounds[n][1] for n in names]) / sc
    r = optimize.least_squares(resid, z0, bounds=(lb, ub), x_scale="jac", diff_step=1e-6)
    J = r.jac * 1.0
    sv = np.linalg.svd(J, compute_uv=False) if J.size else np.array([])
    _, _, vt = np.linalg.svd(J) if J.size else (None, None, np.zeros((0, 0)))
    rel = sv / sv[0] if len(sv) and sv[0] > 0 else sv
    # A direction is structurally unidentifiable when the Jacobian is singular
    # to rounding (1e-6), practically when ill-conditioned (1e-3: a parameter
    # error a thousand times larger than another moves the fit equally).
    unidentifiable = []
    for i, v in enumerate(rel):
        if v < 1e-3:
            direction = vt[i] / sc
            direction = direction / np.max(np.abs(direction))
            unidentifiable.append({"kind": "structural" if v < 1e-6 else "practical (ill-conditioned)",
                                   "relative_singular_value": float(v),
                                   "direction": {n: float(c) for n, c in zip(names, direction) if abs(c) > 1e-9}})
    n = len(r.fun)
    dof = max(n - len(names), 1)
    s2 = float(r.fun @ r.fun) / dof
    cov = None
    if not unidentifiable:
        cov = np.linalg.inv(J.T @ J) * s2
    est = {nm: float(v) for nm, v in zip(names, r.x * sc)}
    se = {nm: float(np.sqrt(cov[i, i]) * sc[i]) for i, nm in enumerate(names)} if cov is not None else {}
    return {"estimate": est, "standard_error_white": se, "cost": float(r.cost), "rms": float(np.sqrt(s2)),
            "n": n, "singular_values": sv.tolist(), "relative_singular_values": rel.tolist(),
            "unidentifiable_directions": unidentifiable, "success": bool(r.success), "nfev": int(r.nfev),
            "channels": list(channels)}


def fit_latency(model: Model, datasets: list[Dataset], base: dict, candidates: list[int],
                names: list[str], x0: dict, bounds: dict) -> dict:
    """The structural latency by profile: refit the deterministic parameters at
    each candidate and compare costs. Equal costs (no stimulus that moves in
    time) leave it unidentified."""
    rows = []
    for L in candidates:
        f = fit_deterministic(model, datasets, {**base, "latencySamples": L}, names, x0, bounds)
        rows.append({"latencySamples": L, "cost": f["cost"], "rms": f["rms"], "fit": f})
    costs = np.array([r["cost"] for r in rows])
    best = int(np.argmin(costs))
    n = rows[best]["fit"]["n"]
    s2 = 2 * costs[best] / max(n - len(names), 1)
    # Gaussian log-likelihood difference of each candidate from the best.
    dll = (costs - costs[best]) / max(s2, 1e-300)
    plausible = [rows[i]["latencySamples"] for i in range(len(rows)) if dll[i] <= stochastic.CHI2_95 / 2]
    return {"candidates": [{"latencySamples": r["latencySamples"], "cost": r["cost"], "rms": r["rms"],
                            "delta_loglik": float(-dll[i])} for i, r in enumerate(rows)],
            "best": rows[best]["latencySamples"], "plausible": plausible,
            "identifiable": len(plausible) == 1, "fit": rows[best]["fit"]}
