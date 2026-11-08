"""Traces: one simulator run as named channels sampled over time.

A trace is a CSV file: optional `# key: value` metadata lines, a header row,
then one row per sample. Column `t` is time in seconds; every other column is
a channel. Channels with the names below have a fixed meaning, frame and unit,
so traces from different simulators compare by name; other columns are kept
and can be compared too. An empty field is a missing sample.

Frames: NED world (north, east, down) relative to home or the origin, FRD
body (forward, right, down), as ArduPilot and CP Models use.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
import math

import numpy as np

# name: (unit, kind, description). kind: linear | angle (wraps at +-pi) | hold
# (piecewise constant: commands).
CHANNELS: dict[str, tuple[str, str, str]] = {}


def _define(names, unit, kind, description):
    for n in names.split():
        CHANNELS[n] = (unit, kind, description)


_define("pos_n pos_e pos_d", "m", "linear", "position NED")
_define("vel_n vel_e vel_d", "m/s", "linear", "velocity NED")
_define("qw qx qy qz", "", "linear", "attitude quaternion, body FRD to NED, scalar first")
_define("roll pitch yaw", "rad", "angle", "Euler angles (ZYX) of the attitude")
_define("rate_x rate_y rate_z", "rad/s", "linear", "true angular velocity, body FRD")
_define("accel_x accel_y accel_z", "m/s^2", "linear", "specific force, body FRD (IMU frame)")
_define("gyro_x gyro_y gyro_z", "rad/s", "linear", "gyroscope, body FRD")
_define("mag_x mag_y mag_z", "Gauss", "linear", "magnetometer, body FRD")
_define("baro_press", "Pa", "linear", "barometric pressure")
_define("baro_alt", "m", "linear", "barometric altitude")
_define("gps_lat gps_lon", "deg", "linear", "GNSS position")
_define("gps_alt", "m", "linear", "GNSS altitude")
_define("gps_vn gps_ve gps_vd", "m/s", "linear", "GNSS velocity NED")
for _i in range(1, 33):
    CHANNELS[f"pwm{_i}"] = ("us", "hold", f"servo output {_i}")

# Report and check groups: name -> channels, in order.
GROUPS = {
    "position": ["pos_n", "pos_e", "pos_d"],
    "velocity": ["vel_n", "vel_e", "vel_d"],
    "attitude": ["roll", "pitch", "yaw"],
    "body rates": ["rate_x", "rate_y", "rate_z"],
    "accelerometer": ["accel_x", "accel_y", "accel_z"],
    "gyroscope": ["gyro_x", "gyro_y", "gyro_z"],
    "magnetometer": ["mag_x", "mag_y", "mag_z"],
    "barometer": ["baro_press", "baro_alt"],
    "GNSS": ["gps_lat", "gps_lon", "gps_alt", "gps_vn", "gps_ve", "gps_vd"],
    "commands": [f"pwm{i}" for i in range(1, 33)],
}


def kind(name: str) -> str:
    return CHANNELS.get(name, ("", "linear", ""))[1]


def unit(name: str) -> str:
    return CHANNELS.get(name, ("", "linear", ""))[0]


@dataclass
class Trace:
    t: np.ndarray
    channels: dict[str, np.ndarray]
    meta: dict[str, str] = field(default_factory=dict)

    def __post_init__(self):
        order = np.argsort(self.t, kind="stable")
        if np.any(order != np.arange(len(order))):
            self.t = self.t[order]
            self.channels = {k: v[order] for k, v in self.channels.items()}

    @property
    def name(self) -> str:
        return self.meta.get("name") or self.meta.get("source", "trace")

    def has(self, name: str) -> bool:
        return name in self.channels and np.any(np.isfinite(self.channels[name]))

    def names(self) -> list[str]:
        return [n for n in self.channels if self.has(n)]

    def shifted(self, dt: float) -> "Trace":
        """The same trace with every time plus dt."""
        return Trace(self.t + dt, dict(self.channels), dict(self.meta))

    def between(self, start: float, end: float) -> "Trace":
        keep = (self.t >= start) & (self.t <= end)
        return Trace(self.t[keep], {k: v[keep] for k, v in self.channels.items()}, dict(self.meta))

    def at(self, name: str, grid: np.ndarray) -> np.ndarray:
        """Channel `name` at the times `grid`: linear interpolation (angles
        unwrapped first, wrapped after), commands held from their sample."""
        v = self.channels[name]
        ok = np.isfinite(v)
        t, v = self.t[ok], v[ok]
        out = np.full(len(grid), np.nan)
        if len(t) == 0:
            return out
        inside = (grid >= t[0]) & (grid <= t[-1])
        k = kind(name)
        if k == "hold":
            idx = np.searchsorted(t, grid[inside], side="right") - 1
            out[inside] = v[np.clip(idx, 0, len(v) - 1)]
        elif k == "angle":
            out[inside] = wrap(np.interp(grid[inside], t, np.unwrap(v)))
        else:
            out[inside] = np.interp(grid[inside], t, v)
        return out

    @classmethod
    def read(cls, path: str | Path) -> "Trace":
        meta: dict[str, str] = {}
        header: list[str] | None = None
        rows: list[list[float]] = []
        with open(path) as f:
            for line in f:
                line = line.rstrip("\r\n")
                if not line:
                    continue
                if line.startswith("#"):
                    key, _, value = line[1:].partition(":")
                    if value:
                        meta[key.strip()] = value.strip()
                    continue
                if header is None:
                    header = [h.strip() for h in line.split(",")]
                    continue
                rows.append([_number(x) for x in line.split(",")])
        if header is None or "t" not in header:
            raise ValueError(f"{path}: not a trace (no header row with a t column)")
        width = len(header)
        data = np.full((len(rows), width), np.nan)
        for i, r in enumerate(rows):
            data[i, :min(width, len(r))] = r[:width]
        ti = header.index("t")
        ok = np.isfinite(data[:, ti])
        channels = {h: data[ok, j] for j, h in enumerate(header) if j != ti}
        meta.setdefault("path", str(path))
        return cls(data[ok, ti], channels, meta)

    def write(self, path: str | Path) -> None:
        names = list(self.channels)
        with open(path, "w") as f:
            for k, v in self.meta.items():
                if k != "path":
                    f.write(f"# {k}: {v}\n")
            f.write(",".join(["t"] + names) + "\n")
            cols = [self.channels[n] for n in names]
            for i, t in enumerate(self.t):
                f.write(",".join([f"{t:.6f}"] + [_fmt(c[i]) for c in cols]) + "\n")


def _number(x: str) -> float:
    x = x.strip()
    if not x:
        return math.nan
    try:
        return float(x)
    except ValueError:
        return math.nan


def _fmt(x: float) -> str:
    return "" if not math.isfinite(x) else f"{x:.9g}"


def wrap(a):
    return (np.asarray(a) + np.pi) % (2 * np.pi) - np.pi


def euler_from_quaternion(w, x, y, z):
    roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = np.arcsin(np.clip(2 * (w * y - z * x), -1, 1))
    yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


def quaternion_from_euler(roll, pitch, yaw):
    cr, sr = np.cos(roll / 2), np.sin(roll / 2)
    cp, sp = np.cos(pitch / 2), np.sin(pitch / 2)
    cy, sy = np.cos(yaw / 2), np.sin(yaw / 2)
    return (cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy)


def quaternion_multiply(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (aw * bw - ax * bx - ay * by - az * bz, aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx, aw * bz + ax * by - ay * bx + az * bw)


def attitude(trace: Trace, grid: np.ndarray):
    """Body-to-NED quaternion at `grid`, from qw..qz or else roll/pitch/yaw; None if neither."""
    if all(trace.has(c) for c in ("qw", "qx", "qy", "qz")):
        q = [trace.at(c, grid) for c in ("qw", "qx", "qy", "qz")]
        n = np.sqrt(sum(c * c for c in q))
        return tuple(c / n for c in q)
    if all(trace.has(c) for c in ("roll", "pitch", "yaw")):
        return quaternion_from_euler(*(trace.at(c, grid) for c in ("roll", "pitch", "yaw")))
    return None
