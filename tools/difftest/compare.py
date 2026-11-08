"""Compare two traces: align them in time, sample both on one grid, measure
every channel they share, and check the measures against tolerances.

Settings (the [compare] table of a spec, or the CLI flags):

    align = "start"     # time zero of each trace: start | takeoff | motion |
                        # command (the first driven servo output that changes) |
                        # arm | disarm | none (the times as they are);
                        # arm/disarm come from a trace's events (DataFlash logs)
    offset = 0.0        # s added to B's time after alignment
    xcorr = "pos_d"     # refine the offset: the shift of B that best matches
    max_shift = 2.0     #   this channel, within +-max_shift s
    start = 0.0         # compared window, s after time zero
    end = 60.0          #   (default: as long as both traces last)
    rate = 50.0         # grid, Hz
    channels = ["pos_*", "attitude"]   # what to report (default: everything shared)

    [[check]]           # a tolerance; channel may be a glob or a derived measure
    channel = "pos_*"
    rmse = 0.5          # root mean square of B - A
    max = 2.0           # largest |B - A|
    bias = 0.2          # |mean of B - A|
    nrmse = 0.1         # rmse / range of A
    tol = 1.0           # B - A stays within tol ...
    hold = 5.0          # ... for at least hold s from the window start

Derived measures, from channels both traces have:
    position  |B - A| of (pos_n, pos_e, pos_d), m
    attitude  angle of the rotation from A's attitude to B's, rad
    path      how far each run is from the other's track, whatever the time:
              at each instant the farther of A from B's whole track and B from
              A's, m. For closed-loop runs whose timing differs.
"""
from __future__ import annotations

from dataclasses import dataclass, field
import fnmatch
import math

import numpy as np

import traces
from traces import Trace

DERIVED = ("position", "attitude", "path")
DEFAULTS = {"align": "start", "offset": 0.0, "xcorr": None, "max_shift": 2.0, "start": 0.0,
            "end": None, "rate": 50.0, "channels": None}
CRITERIA = ("rmse", "max", "bias", "nrmse", "hold")


@dataclass
class Measure:
    channel: str
    unit: str
    n: int
    rmse: float
    mae: float
    max: float
    bias: float
    nrmse: float
    corr: float
    a_range: tuple[float, float]
    exceeds_at: float | None = None      # first time |B - A| > tol (window time)
    checks: list[tuple[str, float, float, bool]] = field(default_factory=list)

    @property
    def ok(self) -> bool:
        return all(c[3] for c in self.checks)


@dataclass
class Result:
    a: Trace
    b: Trace
    zero_a: float
    zero_b: float
    shift: float
    grid: np.ndarray
    measures: list[Measure]
    settings: dict

    @property
    def checked(self) -> bool:
        return any(m.checks for m in self.measures)

    @property
    def ok(self) -> bool:
        return all(m.ok for m in self.measures)

    def series(self, channel: str):
        """(A, B) of a channel on the grid (window time)."""
        if channel in DERIVED:
            return None
        return (self.a.at(channel, self.grid + self.zero_a),
                self.b.at(channel, self.grid + self.zero_b + self.shift))


def event_time(trace: Trace, name: str) -> float:
    t = trace.t
    if name in ("start", "none"):
        return t[0] if name == "start" else 0.0
    if f"event.{name}" in trace.meta:
        return float(trace.meta[f"event.{name}"])
    if name == "takeoff" and trace.has("pos_d"):
        d = trace.channels["pos_d"]
        ok = np.isfinite(d)
        base = np.median(d[ok][: max(1, np.searchsorted(t[ok], t[ok][0] + 1.0))])
        up = np.nonzero(ok & (d < base - 0.5))[0]
        if len(up):
            return t[up[0]]
    if name == "motion" and all(trace.has(c) for c in ("vel_n", "vel_e", "vel_d")):
        v = np.sqrt(sum(np.nan_to_num(trace.channels[c]) ** 2 for c in ("vel_n", "vel_e", "vel_d")))
        moving = np.nonzero(v > 0.5)[0]
        if len(moving):
            return t[moving[0]]
    if name == "command":
        first = None
        for c in trace.names():
            if not c.startswith("pwm"):
                continue
            v = trace.channels[c]
            ok = np.nonzero(np.isfinite(v) & (v != 0))[0]  # 0: not driven
            if not len(ok):
                continue
            changed = ok[np.abs(v[ok] - v[ok[0]]) > 20]
            if len(changed) and (first is None or t[changed[0]] < first):
                first = t[changed[0]]
        if first is not None:
            return first
    raise ValueError(f"{trace.name}: no '{name}' to align on")


def _errors(channel: str, a: np.ndarray, b: np.ndarray) -> np.ndarray:
    e = b - a
    return traces.wrap(e) if traces.kind(channel) == "angle" else e


def _derived(name: str, a: Trace, b: Trace, ga: np.ndarray, gb: np.ndarray):
    """(A magnitude or None, B - A error) of a derived measure on the grid, or None."""
    pos = ("pos_n", "pos_e", "pos_d")
    if name == "position" and all(a.has(c) and b.has(c) for c in pos):
        d = np.stack([b.at(c, gb) - a.at(c, ga) for c in pos])
        return None, np.sqrt((d ** 2).sum(axis=0))
    if name == "attitude":
        qa, qb = traces.attitude(a, ga), traces.attitude(b, gb)
        if qa is None or qb is None:
            return None
        # The rotation from A to B: q_a^-1 q_b, its angle (stable near zero).
        w, x, y, z = traces.quaternion_multiply((qa[0], -qa[1], -qa[2], -qa[3]), qb)
        return None, 2 * np.arctan2(np.sqrt(x * x + y * y + z * z), np.abs(w))
    if name == "path" and all(a.has(c) and b.has(c) for c in pos):
        # At each instant, the farther of: A from B's whole track, B from A's.
        pa = np.stack([a.at(c, ga) for c in pos], axis=1)
        pb = np.stack([b.at(c, gb) for c in pos], axis=1)
        track_a, track_b = _track(a, pos), _track(b, pos)
        if len(track_a) < 2 or len(track_b) < 2:
            return None
        return None, np.fmax(_to_polyline(pa, track_b), _to_polyline(pb, track_a))
    return None


def _track(trace: Trace, pos, points: int = 4000) -> np.ndarray:
    """A trace's whole track, thinned to about `points` points."""
    p = np.stack([trace.channels[c] for c in pos], axis=1)
    p = p[np.all(np.isfinite(p), axis=1)]
    return p[::max(1, len(p) // points)] if len(p) else p


def _to_polyline(points: np.ndarray, line: np.ndarray, chunk: int = 256) -> np.ndarray:
    """Distance from each point to the polyline through `line`'s points."""
    p0, p1 = line[:-1], line[1:]
    seg = p1 - p0
    length2 = np.maximum((seg ** 2).sum(axis=1), 1e-12)
    out = np.full(len(points), np.nan)
    for s in range(0, len(points), chunk):
        p = points[s:s + chunk]
        ok = np.all(np.isfinite(p), axis=1)
        if not ok.any():
            continue
        q = p[ok][:, None, :]
        u = np.clip(((q - p0) * seg).sum(axis=2) / length2, 0, 1)
        nearest = p0 + u[..., None] * seg
        out[s:s + chunk][ok] = np.sqrt(((q - nearest) ** 2).sum(axis=2)).min(axis=1)
    return out


def _measure(channel: str, a: np.ndarray | None, e: np.ndarray, grid: np.ndarray,
             unit: str) -> Measure | None:
    ok = np.isfinite(e) if a is None else np.isfinite(e) & np.isfinite(a)
    if ok.sum() < 2:
        return None
    ev = e[ok]
    rmse = float(np.sqrt(np.mean(ev ** 2)))
    if a is not None:
        av = a[ok]
        lo, hi = float(np.min(av)), float(np.max(av))
        span = hi - lo
        bv = av + ev
        corr = (float(np.corrcoef(av, bv)[0, 1])
                if np.std(av) > 1e-12 and np.std(bv) > 1e-12 else math.nan)
    else:
        lo = hi = span = corr = math.nan
    return Measure(channel, unit, int(ok.sum()), rmse, float(np.mean(np.abs(ev))),
                   float(np.max(np.abs(ev))), float(np.mean(ev)),
                   rmse / span if span and span > 1e-12 else math.nan, corr, (lo, hi))


def _xcorr_shift(a: Trace, b: Trace, channel: str, za: float, zb: float, grid: np.ndarray,
                 max_shift: float, rate: float) -> float:
    va = a.at(channel, grid + za)
    best, best_err = 0.0, math.inf
    for s in np.arange(-max_shift, max_shift + 0.5 / rate, 1.0 / rate):
        e = _errors(channel, va, b.at(channel, grid + zb + s))
        ok = np.isfinite(e)
        if ok.sum() < len(grid) * 0.5:
            continue
        err = float(np.mean(e[ok] ** 2))
        if err < best_err:
            best, best_err = float(s), err
    return best


def compare(a: Trace, b: Trace, settings: dict | None = None,
            checks: list[dict] | None = None) -> Result:
    cfg = dict(DEFAULTS)
    cfg.update({k: v for k, v in (settings or {}).items() if v is not None})
    unknown = set(cfg) - set(DEFAULTS)
    if unknown:
        raise ValueError(f"unknown compare settings: {', '.join(sorted(unknown))}")
    za = event_time(a, cfg["align"])
    zb = event_time(b, cfg["align"]) - float(cfg["offset"])
    rate = float(cfg["rate"])
    end = min(a.t[-1] - za, b.t[-1] - zb)
    if cfg["end"] is not None:
        end = min(end, float(cfg["end"]))
    start = max(float(cfg["start"]), a.t[0] - za, b.t[0] - zb)
    if end <= start:
        raise ValueError(f"the traces do not overlap after alignment ({cfg['align']}): "
                         f"window {start:.2f}..{end:.2f} s")
    grid = np.arange(start, end, 1.0 / rate)
    shift = 0.0
    if cfg["xcorr"]:
        if not (a.has(cfg["xcorr"]) and b.has(cfg["xcorr"])):
            raise ValueError(f"xcorr channel {cfg['xcorr']} is not in both traces")
        shift = _xcorr_shift(a, b, cfg["xcorr"], za, zb, grid, float(cfg["max_shift"]), rate)
    ga, gb = grid + za, grid + zb + shift

    shared = [c for c in a.names() if b.has(c)]
    wanted = cfg["channels"]
    names = shared + [d for d in DERIVED]
    if wanted:
        names = [n for n in names if any(fnmatch.fnmatchcase(n, w) for w in wanted)]

    measures: list[Measure] = []
    for name in names:
        if name in DERIVED:
            got = _derived(name, a, b, ga, gb)
            if got is None:
                continue
            m = _measure(name, got[0], got[1], grid, "rad" if name == "attitude" else "m")
        else:
            va, vb = a.at(name, ga), b.at(name, gb)
            m = _measure(name, va, _errors(name, va, vb), grid, traces.unit(name))
        if m is not None:
            measures.append(m)

    result = Result(a, b, za, zb, shift, grid, measures, cfg)
    for check in checks or []:
        _apply(result, check)
    return result


def _error_series(result: Result, name: str) -> np.ndarray:
    ga, gb = result.grid + result.zero_a, result.grid + result.zero_b + result.shift
    if name in DERIVED:
        return _derived(name, result.a, result.b, ga, gb)[1]
    va, vb = result.a.at(name, ga), result.b.at(name, gb)
    return _errors(name, va, vb)


def _apply(result: Result, check: dict) -> None:
    pattern = check.get("channel")
    if not pattern:
        raise ValueError(f"check without a channel: {check}")
    unknown = set(check) - {"channel", "tol", *CRITERIA}
    if unknown:
        raise ValueError(f"check {pattern}: unknown keys {', '.join(sorted(unknown))}")
    if "hold" in check and "tol" not in check:
        raise ValueError(f"check {pattern}: hold needs tol")
    hits = [m for m in result.measures if fnmatch.fnmatchcase(m.channel, pattern)]
    if not hits:
        # A check on something neither trace has is a failure, not a pass.
        missing = Measure(pattern, "", 0, *(math.nan,) * 6, (math.nan, math.nan))
        missing.checks.append(("present", math.nan, math.nan, False))
        result.measures.append(missing)
        return
    for m in hits:
        if "tol" in check:
            e = np.abs(_error_series(result, m.channel))
            over = np.nonzero(np.isfinite(e) & (e > check["tol"]))[0]
            m.exceeds_at = float(result.grid[over[0]]) if len(over) else None
        for crit in CRITERIA:
            if crit not in check:
                continue
            limit = float(check[crit])
            if crit == "hold":
                held = (m.exceeds_at if m.exceeds_at is not None
                        else result.grid[-1]) - result.grid[0]
                m.checks.append((f"within {check['tol']:g} for", limit, held,
                                 m.exceeds_at is None or held >= limit))
                continue
            value = abs(m.bias) if crit == "bias" else getattr(m, crit)
            m.checks.append((crit, limit, value, bool(np.isfinite(value) and value <= limit)))


def table(result: Result) -> str:
    """The measures as text."""
    a, b = result.a.name, result.b.name
    lines = [f"A: {a}", f"B: {b}",
             f"aligned on {result.settings['align']}: A t0 {result.zero_a:.3f} s, "
             f"B t0 {result.zero_b:.3f} s"
             + (f", B shifted {result.shift:+.3f} s ({result.settings['xcorr']})"
                if result.settings["xcorr"] else ""),
             f"window {result.grid[0]:.2f}..{result.grid[-1]:.2f} s at "
             f"{result.settings['rate']:g} Hz", ""]
    head = f"{'channel':<12} {'unit':<6} {'rmse':>10} {'max':>10} {'bias':>10} {'nrmse':>7} {'corr':>6}"
    lines += [head, "-" * len(head)]
    for m in result.measures:
        lines.append(f"{m.channel:<12} {m.unit:<6} {_g(m.rmse):>10} {_g(m.max):>10} "
                     f"{_g(m.bias):>10} {_g(m.nrmse, 3):>7} {_g(m.corr, 3):>6}"
                     + (f"  exceeds tol at {m.exceeds_at:.2f} s" if m.exceeds_at is not None else ""))
        for crit, limit, value, ok in m.checks:
            lines.append(f"    {'ok  ' if ok else 'FAIL'} {crit} {_g(value)} (limit {_g(limit)})")
    if result.checked:
        failed = sum(not m.ok for m in result.measures)
        lines += ["", "PASSED" if result.ok else f"FAILED: {failed} channel(s) out of tolerance"]
    return "\n".join(lines)


def _g(x: float, digits: int = 4) -> str:
    return "-" if x is None or not math.isfinite(x) else f"{x:.{digits}g}"


def to_json(result: Result) -> dict:
    def clean(x):
        return None if isinstance(x, float) and not math.isfinite(x) else x
    return {
        "a": result.a.meta, "b": result.b.meta,
        "settings": result.settings, "zero_a": result.zero_a, "zero_b": result.zero_b,
        "shift": result.shift, "window": [float(result.grid[0]), float(result.grid[-1])],
        "ok": result.ok if result.checked else None,
        "measures": [{
            "channel": m.channel, "unit": m.unit, "n": m.n, "rmse": clean(m.rmse),
            "mae": clean(m.mae), "max": clean(m.max), "bias": clean(m.bias),
            "nrmse": clean(m.nrmse), "corr": clean(m.corr), "exceeds_at": m.exceeds_at,
            "checks": [{"criterion": c, "limit": clean(l), "value": clean(v), "ok": ok}
                       for c, l, v, ok in m.checks],
        } for m in result.measures],
    }
