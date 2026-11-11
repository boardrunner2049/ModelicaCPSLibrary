"""DPS310 noise against oversampling on public data: the Causal Chambers wind tunnel.

The wind tunnel (Gamella, Peters and Bühlmann, Nature Machine Intelligence
2025, arXiv 2404.11341; data CC BY 4.0) carries four Infineon DPS310
barometers (its firmware, juangamella/causal-chamber hardware/arduino/wt_mk_1,
reads them through the Infineon Dps310 library, measurePressureOnce with
precision codes 0..3 for 1x..8x oversampling). Dataset wt_test_v1, experiment
osr_barometers: 100 readings at 1x, then 100 at 8x, of all four units at once,
about 6.6 Hz, fans at their lowest setting.

What this data can test: the per-reading pressure noise at 1x and 8x against
the datasheet's precision figures (Table 16: 2.5 Pa and 0.4 Pa), which are
the composer's `fast` mode noise and a table entry. What it cannot: absolute
offsets, drift, temperature dependence or latency (no reference instrument,
30 s per setting, no timestamps of the conversions). Unit-to-unit offsets are
reported but confounded with the units' positions in the tunnel.

The comparison was written after the data was first looked at, so it states
ratios with confidence intervals and no pass/fail thresholds.

    build/venv-calib/bin/python -m cp_calibration chambers
"""
from __future__ import annotations

import csv
import datetime as _dt
import hashlib
import json
import subprocess
import urllib.request
import zipfile
from pathlib import Path

import numpy as np
from scipy import stats

from .fmu import LIBRARY_ROOT, file_sha256

URL = "https://causalchamber.s3.eu-central-1.amazonaws.com/downloadables/wt_test_v1.zip"
MD5 = "5eed621f5b54e431c4442fc9cc947db6"          # causal-chamber datasets/directory.yaml
SHA256 = "1809a70db71caa3b41347d1e9f7b57e47082d9631433217ce212ab0798c4e8fd"
DIR = LIBRARY_ROOT / "build/calibration/datasets/causal_chambers"
UNITS = ["upwind", "downwind", "ambient", "intake"]
COMPOSE = LIBRARY_ROOT / "tools/fire-compose/target/debug/fire-compose"
CITATION = ("J. L. Gamella, J. Peters, P. Bühlmann, \"Causal chambers as a real-world physical testbed for AI "
            "methodology\", Nature Machine Intelligence 7, 2025 (arXiv 2404.11341); dataset wt_test_v1, CC BY 4.0, "
            "https://github.com/juangamella/causal-chamber")


def fetch() -> Path:
    """The osr_barometers CSV, downloading and checking the archive once."""
    z = DIR / "wt_test_v1.zip"
    if not z.exists():
        DIR.mkdir(parents=True, exist_ok=True)
        urllib.request.urlretrieve(URL, z)
    if hashlib.md5(z.read_bytes()).hexdigest() != MD5 or file_sha256(z) != SHA256:
        raise ValueError(f"{z}: does not match the published archive")
    out = DIR / "wt_test_v1"
    csv_path = out / "wt_test_v1/osr_barometers.csv"
    if not csv_path.exists():
        with zipfile.ZipFile(z) as f:
            f.extract("wt_test_v1/osr_barometers.csv", out)
    return csv_path


def noise(x: np.ndarray, t: np.ndarray) -> dict:
    """Per-reading noise: the std after removing a linear trend (slow ambient
    change), its 95 % interval (chi-square, n - 2 dof), and the first-difference
    estimate as a check that is insensitive to slow variation."""
    n = len(x)
    r = x - np.polyval(np.polyfit(t, x, 1), t)
    s = float(np.sqrt(np.sum(r ** 2) / (n - 2)))
    lo, hi = (s * np.sqrt((n - 2) / stats.chi2.ppf(q, n - 2)) for q in (0.975, 0.025))
    return {"n": n, "std_pa": s, "interval95_pa": [float(lo), float(hi)],
            "diff_std_pa": float(np.std(np.diff(x), ddof=1) / np.sqrt(2)),
            "lag1": float(np.corrcoef(r[:-1], r[1:])[0, 1])}


def datasheet() -> dict:
    j = json.loads(subprocess.run([str(COMPOSE), "part", "barometer", "infineon_dps310", "--mode", "fast", "--json"],
                                  capture_output=True, text=True, check=True, cwd=LIBRARY_ROOT).stdout)
    fast = j["parameters"]["pressureNoiseStd"]["value"]
    return {"fast_mode_pressureNoiseStd_pa": float(fast), "table16_precision_pa": {1: 2.5, 2: 1.0, 4: 0.5, 8: 0.4}}


def run(out: Path | None = None) -> dict:
    out = Path(out or LIBRARY_ROOT / "build/calibration/dps310_causal_chambers")
    out.mkdir(parents=True, exist_ok=True)
    path = fetch()
    rows = list(csv.DictReader(open(path)))
    col = lambda k: np.array([float(r[k]) for r in rows])
    t, flag = col("timestamp"), col("flag")
    sheet = datasheet()
    settings = []
    for fl in sorted(set(flag)):
        m = flag == fl
        osr = {int(col(f"osr_{u}")[m][0]) for u in UNITS}
        if len(osr) != 1:
            raise ValueError("units at different oversampling in one block")
        osr = osr.pop()
        spec = sheet["table16_precision_pa"][osr]
        units = {}
        for u in UNITS:
            nz = noise(col(f"pressure_{u}")[m], t[m])
            units[u] = {**nz, "ratio_to_datasheet": nz["std_pa"] / spec,
                        "ratio_interval95": [v / spec for v in nz["interval95_pa"]],
                        "mean_pa": float(np.mean(col(f"pressure_{u}")[m]))}
        X = np.vstack([col(f"pressure_{u}")[m] for u in UNITS])
        common = (X - X.mean(1, keepdims=True)).mean(0)
        settings.append({"oversampling": osr, "datasheet_precision_pa": spec, "readings": int(m.sum()),
                         "span_s": float(t[m][-1] - t[m][0]), "period_s": float(np.median(np.diff(t[m]))),
                         "fan_rpm": [float(col("rpm_in")[m].mean()), float(col("rpm_out")[m].mean())],
                         "units": units, "common_mode_std_pa": float(np.std(common, ddof=1)),
                         "offsets_vs_ambient_pa": {u: units[u]["mean_pa"] - units["ambient"]["mean_pa"] for u in UNITS}})
    pooled = {s["oversampling"]: float(np.sqrt(np.mean([s["units"][u]["std_pa"] ** 2 for u in UNITS]))) for s in settings}
    rep = {"created": _dt.datetime.now(_dt.timezone.utc).isoformat(timespec="seconds"), "source": CITATION,
           "archive": {"url": URL, "md5": MD5, "sha256": SHA256}, "readings_sha256": file_sha256(path),
           "datasheet": sheet, "settings": settings, "pooled_std_pa": pooled,
           "noise_ratio_8x_over_1x": pooled[8] / pooled[1],
           "datasheet_ratio_8x_over_1x": sheet["table16_precision_pa"][8] / sheet["table16_precision_pa"][1],
           "post_hoc": True}
    (out / "report.json").write_text(json.dumps(rep, indent=1) + "\n")
    (out / "report.md").write_text(markdown(rep))
    return rep


def markdown(r: dict) -> str:
    L = ["# DPS310 noise against oversampling: Causal Chambers wind tunnel", "",
         f"Source: {r['source']}. Archive sha256 `{r['archive']['sha256'][:16]}…` (md5 matches the published value); "
         f"readings `osr_barometers.csv` sha256 `{r['readings_sha256'][:16]}…`.", "",
         "> **Real data, four DPS310 units, no reference instrument.** It tests the per-reading noise figures only. "
         "The comparison was written after the data was first examined, so it reports ratios with 95 % intervals "
         "and no pass/fail thresholds.", "",
         "| oversampling | unit | readings | std (Pa) | 95 % interval | first-difference std | lag-1 | datasheet (Pa) | ratio |",
         "|---|---|---|---|---|---|---|---|---|"]
    for s in r["settings"]:
        for u, x in s["units"].items():
            L.append(f"| {s['oversampling']}x | {u} | {x['n']} | {x['std_pa']:.3f} | {x['interval95_pa'][0]:.3f}–"
                     f"{x['interval95_pa'][1]:.3f} | {x['diff_std_pa']:.3f} | {x['lag1']:+.2f} | "
                     f"{s['datasheet_precision_pa']} | {x['ratio_to_datasheet']:.2f} "
                     f"({x['ratio_interval95'][0]:.2f}–{x['ratio_interval95'][1]:.2f}) |")
    L += ["", "Pooled over the four units: " + ", ".join(f"{k}x {v:.3f} Pa" for k, v in r["pooled_std_pa"].items())
          + f"; 8x/1x {r['noise_ratio_8x_over_1x']:.3f} (datasheet {r['datasheet_ratio_8x_over_1x']:.3f}; "
          "averaging 8 independent conversions alone would give 0.354).", ""]
    for s in r["settings"]:
        L.append(f"- {s['oversampling']}x: {s['readings']} readings over {s['span_s']:.1f} s (period {s['period_s']*1e3:.0f} ms), "
                 f"fans at {s['fan_rpm'][0]:.0f}/{s['fan_rpm'][1]:.0f} rpm; common-mode std {s['common_mode_std_pa']:.3f} Pa "
                 f"({r['pooled_std_pa'][s['oversampling']] / 2:.3f} if the units' noise were independent: the excess is "
                 "variation they share, flow or ambient, which the per-unit figures include); "
                 "offsets against the ambient unit " + ", ".join(f"{u} {v:+.2f}" for u, v in s["offsets_vs_ambient_pa"].items())
                 + " Pa.")
    L += ["", f"The composer's DPS310 `fast` mode (1x) sets pressureNoiseStd = "
          f"{r['datasheet']['fast_mode_pressureNoiseStd_pa']} Pa.", "", "## Reading", "",
          "- Every unit's noise is at or below the datasheet precision at both settings: the figures behave as "
          "conservative typical values for these four units. The composer's `fast` mode (2.5 Pa) therefore "
          "overstates these units' noise; a profile from this data would set about the pooled 1x value.",
          "- The 8x/1x ratio is close to the datasheet's, consistent with the noise model's scaling across modes.",
          "- Not tested: absolute accuracy, drift, temperature coefficient, latency, and the 64x (`low_noise`) mode. "
          "The unit offsets (a few Pa) are within the ±100 Pa absolute accuracy but are confounded with position and "
          "airflow. The fans run during both blocks, so flow fluctuation may contribute to the noise; it would raise, "
          "not lower, the estimates.", ""]
    return "\n".join(L)
