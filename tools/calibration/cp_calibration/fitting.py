"""Bounded least squares with an identifiability check, for any sensor model.

`fit(resid, names, x0, bounds)` minimizes |resid(params)|^2 over the named
parameters (SciPy, trust-region reflective, scaled), then reads the Jacobian
at the solution:

- standard errors from (J^T J)^-1 s^2, with s^2 the residual variance (or a
  given measurement sigma): valid for independent residuals;
- singular values relative to the largest: below 1e-6 a direction is
  structurally unidentifiable (only a combination is determined), below 1e-3
  practically (ill-conditioned). Such fits get no standard errors, and the
  direction is reported.
"""
from __future__ import annotations

import numpy as np
from scipy import optimize

STRUCTURAL = 1e-6
PRACTICAL = 1e-3


def fit(resid, names: list[str], x0: dict, bounds: dict, scale: dict | None = None,
        sigma: float | None = None, diff_step: float = 1e-6) -> dict:
    names = list(names)
    sc = np.array([(scale or {}).get(n, max(abs(x0[n]), 1e-12)) for n in names], float)

    def r(z):
        out = np.asarray(resid(dict(zip(names, z * sc))), float)
        return out[np.isfinite(out)]

    z0 = np.array([x0[n] for n in names]) / sc
    lb = np.array([bounds[n][0] for n in names]) / sc
    ub = np.array([bounds[n][1] for n in names]) / sc
    res = optimize.least_squares(r, np.clip(z0, lb, ub), bounds=(lb, ub), x_scale="jac", diff_step=diff_step)
    J = res.jac
    _, sv, vt = np.linalg.svd(J, full_matrices=False)
    rel = sv / sv[0] if len(sv) and sv[0] > 0 else sv
    directions = []
    for i, v in enumerate(rel):
        if v < PRACTICAL:
            d = vt[i] / sc
            d = d / np.max(np.abs(d))
            directions.append({"kind": "structural" if v < STRUCTURAL else "practical (ill-conditioned)",
                               "relative_singular_value": float(v),
                               "direction": {n: float(c) for n, c in zip(names, d) if abs(c) > 1e-9}})
    n = len(res.fun)
    dof = max(n - len(names), 1)
    s2 = sigma ** 2 if sigma else float(res.fun @ res.fun) / dof
    est = {nm: float(v) for nm, v in zip(names, res.x * sc)}
    se, corr = {}, None
    if not directions:
        cov = np.linalg.inv(J.T @ J) * s2
        se = {nm: float(np.sqrt(cov[i, i]) * sc[i]) for i, nm in enumerate(names)}
        d = np.sqrt(np.diag(cov))
        corr = (cov / np.outer(d, d)).tolist()
    return {"estimate": est, "standard_error": se, "correlation": corr, "rms": float(np.sqrt(np.mean(res.fun ** 2))),
            "residual_sigma": float(np.sqrt(s2)), "n": n, "cost": float(res.cost), "success": bool(res.success),
            "nfev": int(res.nfev), "relative_singular_values": rel.tolist(), "unidentifiable_directions": directions}


def recovered(estimate: float, truth: float, se: float | None, k: float = 3.0) -> dict:
    """Same-model recovery: within k standard errors."""
    if not se:
        return {"z": None, "recovered": None}
    z = (estimate - truth) / se
    return {"z": float(z), "recovered": bool(abs(z) <= k)}
