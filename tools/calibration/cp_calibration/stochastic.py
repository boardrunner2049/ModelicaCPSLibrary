"""Stochastic error models for sampled sensors, and their identification.

The B1 barometer's pressure error at acquisition k, after the deterministic
part (true pressure plus offset), is

    e_k = d_k + w_k (+ quantization),   d_k = phi d_{k-1} + sigma_d sqrt(1 - phi^2) n_k,
    phi = exp(-T / tau),                w_k ~ N(0, sigma_w^2)

a stationary first-order Gauss-Markov drift plus white noise (Sabatini and
Genovese 2013). `loglik` is its exact Gaussian log-likelihood by a scalar
Kalman filter, with the offset b as the mean. Identifiability is checked with
profile likelihood: a parameter whose likelihood-ratio interval reaches the
edge of its search range is reported as not identifiable from the record
instead of being given a finite interval.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
from scipy import optimize, stats

CHI2_95 = stats.chi2.ppf(0.95, 1)


def loglik(y: np.ndarray, T: float, b: float, sigma_w: float, sigma_d: float, tau: float,
           quantum: float = 0.0, innovations: bool = False):
    """Exact log-likelihood of e = y - b under drift + white noise. Quantization
    is treated as extra white noise of variance q^2/12."""
    e = (np.asarray(y, float) - b).tolist()
    r = sigma_w ** 2 + quantum ** 2 / 12.0
    phi = math.exp(-T / tau) if tau > 0 else 0.0
    qd = sigma_d ** 2 * (1 - phi ** 2)
    x, p = 0.0, sigma_d ** 2
    ll = 0.0
    nu = np.empty(len(e)) if innovations else None
    for k, ek in enumerate(e):
        s = p + r
        v = ek - x
        if s <= 0:
            return -np.inf
        ll -= 0.5 * (math.log(2 * math.pi * s) + v * v / s)
        if innovations:
            nu[k] = v / math.sqrt(s)
        g = p / s
        x, p = x + g * v, (1 - g) * p
        x, p = phi * x, phi * phi * p + qd
    return (ll, nu) if innovations else ll


@dataclass
class Bounds:
    sigma_w: tuple[float, float]
    sigma_d: tuple[float, float]
    tau: tuple[float, float]


def _unpack(z, fixed):
    names = ["b", "sigma_w", "sigma_d", "tau"]
    vals, i = {}, 0
    for n in names:
        if n in fixed:
            vals[n] = fixed[n]
        else:
            vals[n] = z[i] if n == "b" else math.exp(z[i])
            i += 1
    return vals


def fit(y: np.ndarray, T: float, bounds: Bounds, quantum: float = 0.0, fixed: dict | None = None,
        start: dict | None = None, drift: bool = True) -> dict:
    """Maximum-likelihood b, sigma_w, sigma_d, tau (those not in `fixed`).
    With drift=False the model is white noise only (sigma_d = 0)."""
    fixed = dict(fixed or {})
    if not drift:
        fixed.update(sigma_d=0.0, tau=1.0)
    names = [n for n in ["b", "sigma_w", "sigma_d", "tau"] if n not in fixed]
    sd = float(np.std(y))
    guess = {"b": float(np.mean(y)), "sigma_w": max(sd / 2, 1e-6), "sigma_d": max(sd / 2, 1e-6),
             "tau": math.sqrt(bounds.tau[0] * bounds.tau[1]), **(start or {})}
    z0 = [guess[n] if n == "b" else math.log(min(max(guess[n], getattr(bounds, n)[0] * 1.01),
                                                    getattr(bounds, n)[1] / 1.01)) for n in names]
    lims = [(None, None) if n == "b" else tuple(map(math.log, getattr(bounds, n))) for n in names]

    def nll(z):
        v = _unpack(z, fixed)
        return -loglik(y, T, v["b"], v["sigma_w"], v["sigma_d"], v["tau"], quantum)

    best = None
    starts = [z0]
    if "tau" in names and not (start or {}).get("_warm"):  # tau is multimodal: try several
        j = names.index("tau")
        for t in np.geomspace(bounds.tau[0] * 1.5, bounds.tau[1] / 1.5, 4):
            z = list(z0)
            z[j] = math.log(t)
            starts.append(z)
    for z in starts:
        r = optimize.minimize(nll, z, method="L-BFGS-B", bounds=lims)
        if best is None or r.fun < best.fun:
            best = r
    v = _unpack(best.x, fixed)
    v["loglik"] = -best.fun
    v["n"] = len(y)
    v["converged"] = bool(best.success)
    return v


def profile(y: np.ndarray, T: float, bounds: Bounds, name: str, grid: np.ndarray, quantum: float = 0.0,
            mle: dict | None = None) -> dict:
    """Profile likelihood of one parameter over `grid` (with the estimate
    added), and its 95 % likelihood-ratio interval, the ends interpolated
    (log-linearly for scale parameters). `identifiable` is False when the
    interval reaches an end of the grid: the record does not bound the
    parameter there, and no finite interval is reported for that side."""
    mle = mle or fit(y, T, bounds, quantum)
    grid = np.unique(np.concatenate([np.asarray(grid, float), [mle[name]]]))
    prof = []
    for side in (1, -1):                      # warm-start outward from the estimate
        order = [i for i in np.argsort(grid) if (grid[i] >= mle[name]) == (side == 1)]
        warm = {**mle, "_warm": True}
        for i in (order if side == 1 else order[::-1]):
            r = fit(y, T, bounds, quantum, fixed={name: float(grid[i])}, start=warm)
            warm = {**r, "_warm": True}
            prof.append((i, r["loglik"]))
    prof = np.array([v for _, v in sorted(prof)])
    top = max(prof.max(), mle["loglik"])
    dev = 2 * (top - prof) - CHI2_95          # <= 0 inside the interval
    log = name != "b"
    x = np.log(grid) if log else grid
    inside = np.flatnonzero(dev <= 0)
    lo_i, hi_i = inside[0], inside[-1]

    def deviance(xv):
        g = math.exp(xv) if log else xv
        return 2 * (top - fit(y, T, bounds, quantum, fixed={name: float(g)}, start={**mle, "_warm": True})["loglik"]) - CHI2_95

    def cross(i, j):                          # dev[i] > 0 >= dev[j]: the end by root finding
        v = optimize.brentq(deviance, x[i], x[j], xtol=1e-6 * max(1.0, abs(x[j])), maxiter=60)
        return float(math.exp(v) if log else v)

    lower_bounded = lo_i > 0
    upper_bounded = hi_i < len(grid) - 1
    lo = cross(lo_i - 1, lo_i) if lower_bounded else None
    hi = cross(hi_i + 1, hi_i) if upper_bounded else None
    weak = bool(log and lo and hi and hi / lo > 10)
    return {"parameter": name, "grid": grid.tolist(), "profile_loglik": prof.tolist(),
            "interval95": [lo, hi], "lower_bounded": bool(lower_bounded), "upper_bounded": bool(upper_bounded),
            "identifiable": bool(lower_bounded and upper_bounded),
            "weakly_identified": weak, "mle": float(mle[name])}


def hessian_se(y: np.ndarray, T: float, est: dict, name: str = "b", quantum: float = 0.0, h: float | None = None):
    """Standard error of one parameter from the curvature of the log-likelihood
    with the others at their estimates (conditional, so a lower bound)."""
    h = h or max(abs(est[name]) * 1e-4, 1e-3)
    def ll(x):
        v = {**est, name: x}
        return loglik(y, T, v["b"], v["sigma_w"], v["sigma_d"], v["tau"], quantum)
    c = (ll(est[name] + h) - 2 * ll(est[name]) + ll(est[name] - h)) / h ** 2
    return math.sqrt(-1 / c) if c < 0 else math.inf


def allan_deviation(x: np.ndarray, T: float, taus: int = 24) -> dict:
    """Overlapping Allan deviation of a sampled series (cluster times m*T)."""
    n = len(x)
    ms = np.unique(np.floor(np.geomspace(1, max(n // 9, 1), taus)).astype(int))
    c = np.concatenate([[0.0], np.cumsum(x)])
    out_t, out_a = [], []
    for m in ms:
        if 2 * m >= n:
            break
        avg = (c[m:] - c[:-m]) / m
        d = avg[m:] - avg[:-m]
        out_t.append(m * T)
        out_a.append(float(np.sqrt(0.5 * np.mean(d ** 2))))
    return {"tau_s": out_t, "adev": out_a}


def ljung_box(nu: np.ndarray, lags: int = 20) -> dict:
    """Portmanteau test that standardized innovations are white."""
    nu = nu - nu.mean()
    n = len(nu)
    acf = np.array([np.sum(nu[k:] * nu[:-k]) for k in range(1, lags + 1)]) / np.sum(nu * nu)
    q = n * (n + 2) * np.sum(acf ** 2 / (n - np.arange(1, lags + 1)))
    return {"lags": lags, "Q": float(q), "p_value": float(stats.chi2.sf(q, lags)),
            "acf": acf.tolist()}
