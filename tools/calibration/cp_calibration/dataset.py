"""Measurement datasets: immutable CSVs plus a JSON description.

    <name>.csv            the sensor's readings, one row each, as logged
    <name>.reference.csv  the independent reference (or known stimulus), on its
                          own time base
    <name>.json           what the columns are, units, what each timestamp
                          means, the reference instrument and its uncertainty,
                          conditions, the sensor unit, and (synthetic data
                          only) the generating truth

Columns (SI units): readings `time_s, pressure_pa, temperature_k`; reference
`time_s, pressure_pa, temperature_k`. The reference is a zero-order hold: each
row holds until the next, so a step is a row at the step time. Clock offsets
between the two time bases are corrected in the description
(`reference.clock_offset_s`, added to reference times), never by aligning away
the latency being estimated.

The harness never rewrites a dataset. It records the readings' and the
reference's SHA-256 in every profile and report and refuses a dataset whose
hash differs from the one an experiment names.
"""
from __future__ import annotations

import csv
import json
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from .fmu import Trace, file_sha256

READINGS = ["time_s", "pressure_pa", "temperature_k"]
REFERENCE = ["time_s", "pressure_pa", "temperature_k"]


@dataclass
class Dataset:
    name: str
    path: Path
    meta: dict
    readings: dict[str, np.ndarray]
    reference: dict[str, np.ndarray]
    sha256: str
    reference_sha256: str

    def trace(self) -> Trace:
        """The reference as the model's input trace (ambient pressure and
        temperature), on the readings' time base."""
        offset = float(self.meta.get("reference", {}).get("clock_offset_s", 0.0))
        t = self.reference["time_s"] + offset
        return Trace(t,
                     {"ambientPressure": self.reference["pressure_pa"],
                      "ambientTemperature": self.reference["temperature_k"]})

    def summary(self) -> dict:
        t = self.readings["time_s"]
        return {"name": self.name, "readings": str(self.path), "sha256": self.sha256,
                "reference_sha256": self.reference_sha256, "rows": int(len(t)),
                "span_s": [float(t[0]), float(t[-1])], "synthetic": bool(self.meta.get("synthetic")),
                "description": self.meta.get("description", ""), "unit": self.meta.get("unit", ""),
                "time_base": self.meta.get("time_base", ""), "reference": self.meta.get("reference", {}),
                "conditions": self.meta.get("conditions", {})}


def _write_csv(path: Path, columns: list[str], data: dict[str, np.ndarray]):
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        columns = columns + [c for c in data if c not in columns]
        w.writerow(columns)
        for row in zip(*(data[c] for c in columns)):
            w.writerow([repr(float(v)) for v in row])


def _read_csv(path: Path, columns: list[str]) -> dict[str, np.ndarray]:
    with open(path) as f:
        rows = list(csv.DictReader(f))
    missing = [c for c in columns if not rows or c not in rows[0]]
    if missing:
        raise ValueError(f"{path}: missing columns {missing}")
    return {c: np.array([float(r[c]) for r in rows]) for c in rows[0]}


def write(path: Path, readings: dict, reference: dict, meta: dict) -> "Dataset":
    path = Path(path).with_suffix("")
    path.parent.mkdir(parents=True, exist_ok=True)
    _write_csv(path.with_suffix(".csv"), READINGS, readings)
    _write_csv(path.with_suffix(".reference.csv"), REFERENCE, reference)
    path.with_suffix(".json").write_text(json.dumps(meta, indent=1, default=float) + "\n")
    return read(path)


def read(path: Path, expect: dict | None = None) -> Dataset:
    base = Path(path)
    base = base.with_name(base.name.removesuffix(".csv").removesuffix(".json"))
    readings_path = base.with_name(base.name + ".csv")
    reference_path = base.with_name(base.name + ".reference.csv")
    sha, ref_sha = file_sha256(readings_path), file_sha256(reference_path)
    for what, got in (("sha256", sha), ("reference_sha256", ref_sha)):
        want = (expect or {}).get(what)
        if want and want != got:
            raise ValueError(f"{base}: {what} {got} differs from the experiment's {want}: raw datasets are "
                             "immutable; give a changed dataset a new name")
    meta_path = base.with_name(base.name + ".json")
    meta = json.loads(meta_path.read_text()) if meta_path.exists() else {}
    return Dataset(base.name, readings_path, meta, _read_csv(readings_path, READINGS),
                   _read_csv(reference_path, REFERENCE), sha, ref_sha)
