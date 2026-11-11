"""Bounded least squares with an identifiability check, for any sensor model.

`fit(resid, names, x0, bounds)` minimizes |resid(params)|^2 over the named
parameters (SciPy, trust-region reflective, scaled), then reads the Jacobian
at the solution:

- standard errors from (J^T J)^-1 s^2, with s^2 the residual variance (or a
  given measurement sigma): valid for independent residuals;
- beside them, Newey-West (HAC, Bartlett kernel) standard errors, which stay
  valid when the residuals are autocorrelated (a logger's filtering, slow
  drift, model error), and the residuals' lag-1 autocorrelation. A profile
  should carry the larger of the two (`standard_error_conservative`);
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
        hac = newey_west(J, res.fun)
        se_hac = {nm: float(np.sqrt(hac[i, i]) * sc[i]) for i, nm in enumerate(names)}
    else:
        se_hac = {}
    e = res.fun - np.mean(res.fun)
    lag1 = float(e[:-1] @ e[1:] / (e @ e)) if n > 2 and e @ e > 0 else 0.0
    return {"estimate": est, "standard_error": se, "standard_error_hac": se_hac,
            "standard_error_conservative": {k: max(v, se_hac.get(k, 0.0)) for k, v in se.items()},
            "residual_lag1": lag1, "correlation": corr, "rms": float(np.sqrt(np.mean(res.fun ** 2))),
            "residual_sigma": float(np.sqrt(s2)), "n": n, "cost": float(res.cost), "success": bool(res.success),
            "nfev": int(res.nfev), "relative_singular_values": rel.tolist(), "unidentifiable_directions": directions}


def newey_west(J: np.ndarray, e: np.ndarray, lags: int | None = None) -> np.ndarray:
    """HAC covariance (J'J)^-1 S (J'J)^-1, S = sum_l w_l (G_l + G_l'), Bartlett
    weights, G_l = sum_t g_t g_{t-l}', g_t = J_t e_t; lag 4(n/100)^(2/9)."""
    n = len(e)
    L = int(np.floor(4 * (n / 100) ** (2 / 9))) if lags is None else lags
    g = J * e[:, None]
    S = g.T @ g
    for l in range(1, min(L, n - 1) + 1):
        G = g[l:].T @ g[:-l]
        S += (1 - l / (L + 1)) * (G + G.T)
    B = np.linalg.inv(J.T @ J)
    return B @ S @ B * n / max(n - J.shape[1], 1)


def recovered(estimate: float, truth: float, se: float | None, k: float = 3.0) -> dict:
    """Same-model recovery: within k standard errors."""
    if not se:
        return {"z": None, "recovered": None}
    z = (estimate - truth) / se
    return {"z": float(z), "recovered": bool(abs(z) <= k)}
