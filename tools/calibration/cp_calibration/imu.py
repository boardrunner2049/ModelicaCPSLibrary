"""The IMU reuse case (milestone 6): the noise workflow on a public static
recording, and the fitted noise driving the existing COMPOSE IMU model.

Data: MILUV (Shalaby et al. 2025, CC BY 4.0), 16 h static recordings of a
Uvify IFO-S's Pixhawk 4 IMU (PX4 /mavros/imu/data_raw) and its RealSense D435i
IMU at 200 Hz. Neither chip is named by the dataset, and no COMPOSE part file
is established for them, so this exercises the workflow; it does not validate
a part. The PX4 stream is the autopilot's processed output, not raw samples.

Per channel: overlapping Allan deviation at the full rate; white noise plus a
first-order Gauss-Markov bias fitted on 1 s means of the training window
(exact Kalman likelihood, profile intervals); the held-out window's Allan
deviation against the band the fitted model predicts; the per-sample spread
and lag-1 correlation against white noise (filtering shows there). Then the
fitted values configure Systems.Sensing.IMU.StatisticalSensor and its output's
Allan deviation is compared with the fitted model's.
"""
from __future__ import annotations

import datetime as _dt
import json
import math
import struct
import tomllib
from pathlib import Path

import numpy as np
from scipy import signal

from . import stochastic as st
from .fmu import LIBRARY_ROOT, Slave, Trace, export, file_sha256, toolchain, rel

AXES = ["gyro_x", "gyro_y", "gyro_z", "accel_x", "accel_y", "accel_z"]


# --- the recording ------------------------------------------------------------------

def decode_imu(raw: bytes):
    """sensor_msgs/Imu (ROS 1): header, orientation and covariance, angular
    velocity and covariance, linear acceleration and covariance."""
    _seq, sec, nsec, n = struct.unpack_from("<IIII", raw, 0)
    o = 16 + n
    o += 8 * 4 + 8 * 9
    w = struct.unpack_from("<3d", raw, o)
    o += 24 + 72
    a = struct.unpack_from("<3d", raw, o)
    return sec + nsec * 1e-9, w, a


def extract(bag: Path, out: Path, topics: dict[str, str]) -> dict:
    """The IMU topics of a bag as arrays (immutable once written; hashed)."""
    from rosbags.rosbag1 import Reader
    res = {}
    with Reader(bag) as r:
        conns = {c.topic: c for c in r.connections}
        for name, topic in topics.items():
            c = conns[topic]
            n = c.msgcount
            t = np.empty(n); tb = np.empty(n); g = np.empty((n, 3)); a = np.empty((n, 3))
            k = 0
            for _, stamp, raw in r.messages(connections=[c]):
                t[k], g[k], a[k] = decode_imu(raw)
                tb[k] = stamp * 1e-9
                k += 1
            path = out / f"{name}.npz"
            np.savez(path, time_s=t[:k], bag_time_s=tb[:k], gyro=g[:k], accel=a[:k])
            res[name] = {"topic": topic, "messages": k, "npz": str(path), "sha256": file_sha256(path)}
    return res


def decode_pressure(raw: bytes):
    """sensor_msgs/FluidPressure (ROS 1): header, fluid_pressure (Pa), variance."""
    _seq, sec, nsec, n = struct.unpack_from("<IIII", raw, 0)
    p, var = struct.unpack_from("<2d", raw, 16 + n)
    return sec + nsec * 1e-9, p


def extract_pressure(bag: Path, out: Path, topic: str) -> dict:
    from rosbags.rosbag1 import Reader
    with Reader(bag) as r:
        c = next(c for c in r.connections if c.topic == topic)
        t = np.empty(c.msgcount); p = np.empty(c.msgcount)
        k = 0
        for _, _, raw in r.messages(connections=[c]):
            t[k], p[k] = decode_pressure(raw)
            k += 1
    path = out / "barometer.npz"
    np.savez(path, time_s=t[:k], pressure_pa=p[:k])
    return {"topic": topic, "messages": k, "npz": str(path), "sha256": file_sha256(path)}


def barometer_case(exp, data_dir: Path, bag: Path, bounds_cfg, th, rng) -> dict:
    """The bag's barometer topic, screened before any analysis: a static
    ambient reading must sit near atmospheric pressure and change between
    consecutive samples by about the part's per-reading noise."""
    b = exp["barometer"]
    path = data_dir / "barometer.npz"
    if not path.exists():
        extract_pressure(bag, data_dir, b["topic"])
    a = dict(np.load(path))
    t, x = a["time_s"], a["pressure_pa"]
    sheet = subprocess_part(b["part"])
    d = np.diff(x[:200000])
    level = float(np.median(x))
    step_rms = float(np.std(d) / math.sqrt(2))              # per-sample noise if consecutive samples were independent
    plausible_level = b["screen"]["level_pa"][0] <= level <= b["screen"]["level_pa"][1]
    plausible_noise = step_rms <= b["screen"]["max_noise_over_datasheet"] * sheet["fast"]
    res = {"topic": b["topic"], "part": b["part"], "part_basis": b["basis"], "samples": int(len(t)),
           "timing": timing(t, len(t) / (t[-1] - t[0])), "npz_sha256": file_sha256(path),
           "median_pa": level, "per_sample_noise_from_differences_pa": step_rms,
           "datasheet": {"pressureNoiseStd_low_noise": sheet["low_noise"], "pressureNoiseStd_fast": sheet["fast"]},
           "screen": b["screen"], "plausible_level": plausible_level, "plausible_noise": plausible_noise,
           "accepted": bool(plausible_level and plausible_noise)}
    if res["accepted"]:
        t0 = t[0]
        bounds = st.Bounds(*(tuple(v) for v in (bounds_cfg["sigma_w"], bounds_cfg["sigma_d"], bounds_cfg["tau"])))
        res["channel"] = analyse_channel(x, t, (t0 + exp["split"]["train_s"][0], t0 + exp["split"]["train_s"][1]),
                                         (t0 + exp["split"]["holdout_s"][0], t0 + exp["split"]["holdout_s"][1]), bounds, th, rng)
    return res


def subprocess_part(part: str) -> dict:
    import subprocess
    exe = LIBRARY_ROOT / "tools/fire-compose/target/debug/fire-compose"
    get = lambda *a: json.loads(subprocess.run([str(exe), "part", "barometer", part, "--json", *a],
                                               capture_output=True, text=True, cwd=LIBRARY_ROOT).stdout)
    lo, fa = get(), get("--mode", "fast")
    return {"low_noise": lo["parameters"]["pressureNoiseStd"]["value"], "fast": fa["parameters"]["pressureNoiseStd"]["value"],
            "quantum": lo["parameters"].get("pressureQuantum", {}).get("value")}


def timing(t: np.ndarray, rate: float) -> dict:
    dt = np.diff(t)
    return {"samples": int(len(t)), "span_s": float(t[-1] - t[0]), "median_dt_s": float(np.median(dt)),
            "gaps_over_1p5_periods": int(np.sum(dt > 1.5 / rate)), "non_increasing": int(np.sum(dt <= 0)),
            "max_gap_s": float(np.max(dt))}


# --- analysis -------------------------------------------------------------------------

def block_means(x: np.ndarray, m: int) -> np.ndarray:
    n = len(x) // m
    return x[: n * m].reshape(n, m).mean(axis=1)


def simulate_model(n: int, T: float, sigma_w: float, sigma_d: float, tau: float, rng) -> np.ndarray:
    phi = math.exp(-T / tau)
    d = signal.lfilter([sigma_d * math.sqrt(1 - phi ** 2)], [1, -phi], rng.standard_normal(n))
    d += sigma_d * rng.standard_normal() * phi ** np.arange(n)      # stationary start
    return d + sigma_w * rng.standard_normal(n)


def allan_on(x, T, taus):
    a = st.allan_deviation(x, T, taus=40)
    return np.interp(np.log(taus), np.log(a["tau_s"]), a["adev"], left=np.nan, right=np.nan)


def bin_means(t: np.ndarray, x: np.ndarray, width: float, t0: float, t1: float):
    """Means of x in consecutive time bins (the streams are not uniformly
    sampled); empty bins are dropped and counted."""
    m = (t >= t0) & (t < t1)
    k = np.floor((t[m] - t0) / width).astype(int)
    n = int(np.ceil((t1 - t0) / width))
    cnt = np.bincount(k, minlength=n)[:n]
    tot = np.bincount(k, weights=x[m], minlength=n)[:n]
    ok = cnt > 0
    return tot[ok] / cnt[ok], int(np.sum(~ok))


def analyse_channel(x: np.ndarray, t: np.ndarray, train: tuple, hold: tuple, bounds: st.Bounds, th: dict, rng) -> dict:
    T1 = 1.0
    m10 = (t >= train[0]) & (t < train[0] + 600)
    seg = x[m10] - np.mean(x[m10])
    raw_std = float(np.std(seg))
    lag1 = float(np.corrcoef(seg[:-1], seg[1:])[0, 1])
    rate = float(np.sum(m10) / 600.0)
    yt, empty_t = bin_means(t, x, T1, *train)
    yh, empty_h = bin_means(t, x, T1, *hold)
    est = st.fit(yt, T1, bounds)
    prof = {k: st.profile(yt, T1, bounds, k, g, mle=est) for k, g in
            (("tau", np.geomspace(bounds.tau[0] * 2, bounds.tau[1] / 2, 21)),
             ("sigma_d", np.geomspace(max(est["sigma_d"] / 30, bounds.sigma_d[0]), est["sigma_d"] * 30, 17)))}
    N = est["sigma_w"] * math.sqrt(T1)                      # white noise density (unit/sqrt(Hz))
    taus = np.geomspace(1, len(yh) / 10, 18)
    hold_adev = allan_on(yh, T1, taus)
    sims = np.array([allan_on(simulate_model(len(yh), T1, est["sigma_w"], est["sigma_d"], est["tau"], rng), T1, taus)
                     for _ in range(th["realizations"])])
    lo, hi = np.nanpercentile(sims, [5, 95], axis=0)
    inside = (hold_adev >= lo) & (hold_adev <= hi)
    ok = np.isfinite(hold_adev)
    frac = float(np.mean(inside[ok]))
    ref = st.fit(yh, T1, bounds)                            # refit: are the values stationary?
    fine, _ = bin_means(t, x, 0.05, *train)
    full = st.allan_deviation(fine - np.mean(fine), 0.05, taus=30)
    return {"noise_density": N, "rate_hz": rate, "per_sample_std_if_white": N * math.sqrt(rate),
            "per_sample_std_measured": raw_std, "lag1_autocorrelation": lag1,
            "empty_1s_bins": {"train": empty_t, "holdout": empty_h},
            "fit": {k: float(est[k]) for k in ("b", "sigma_w", "sigma_d", "tau")},
            "tau_interval95": prof["tau"]["interval95"], "tau_identifiable": prof["tau"]["identifiable"],
            "sigma_d_interval95": prof["sigma_d"]["interval95"],
            "holdout": {"taus_s": taus.tolist(), "measured": hold_adev.tolist(), "band5": lo.tolist(), "band95": hi.tolist(),
                        "fraction_inside": frac, "pass": frac >= th["fraction_inside_min"],
                        "ratio_to_median_first": float(hold_adev[0] / np.nanmedian(sims[:, 0])),
                        "ratio_to_median_last": float(hold_adev[ok][-1] / np.nanmedian(sims[:, np.flatnonzero(ok)[-1]]))},
            "holdout_refit": {k: float(ref[k]) for k in ("sigma_w", "sigma_d", "tau")},
            "allan_train_50ms_bins": {"tau_s": full["tau_s"], "adev": full["adev"]}}


def exercise_model(channels: dict, rate: float, duration: float, seed: int) -> dict:
    """The fitted values in Systems.Sensing.IMU.StatisticalSensor, static and level."""
    s = Slave(export("Systems.Sensing.IMU.StatisticalSensor"))
    try:
        order = ["accel_x", "accel_y", "accel_z", "gyro_x", "gyro_y", "gyro_z"]
        noise = [channels[c]["per_sample_std_if_white"] for c in order]
        bstd = [channels[c]["fit"]["sigma_d"] for c in order]
        btau = [channels[c]["fit"]["tau"] for c in order]
        rate = float(np.mean([channels[c]["rate_hz"] for c in order]))
        params = {"p.samplePeriod": 1 / rate, "p.noiseStd": noise, "p.biasStd": bstd, "p.biasTau": btau, "p.seed": seed}
        tr = Trace(np.array([0.0]), {"R_wb": np.array([[1, 0, 0, 0, 1, 0, 0, 0, 1]], float),
                                     "gravity_w": np.array([[0.0, 0.0, 9.80665]]), "a_w": np.zeros((1, 3)),
                                     "omega_b": np.zeros((1, 3)), "alpha_b": np.zeros((1, 3))})
        r = s.run(tr, params, ["acceleration", "gyro"], duration, 1 / rate)
        tt = r.time[1:]
        out = {}
        taus = np.geomspace(1, duration / 10, 10)
        for j, c in enumerate(order):
            y = (r.outputs["acceleration"] if c.startswith("accel") else r.outputs["gyro"])[1:, j % 3]
            y = y - np.mean(y)
            model = allan_on(bin_means(tt, y, 1.0, 0.0, duration)[0], 1.0, taus)
            ch = channels[c]["fit"]
            rng = np.random.default_rng(seed + j)
            ref = np.nanmedian([allan_on(simulate_model(len(y) // int(rate), 1.0, ch["sigma_w"], ch["sigma_d"], ch["tau"], rng),
                                         1.0, taus) for _ in range(20)], axis=0)
            out[c] = {"taus_s": taus.tolist(), "compose_model": model.tolist(), "fitted_model_median": ref.tolist(),
                      "ratio": (model / ref).tolist(), "per_sample_std": float(np.std(y)),
                      # A Gauss-Markov bias seen over a window T, mean removed:
                      # s^2 (1 - 2 tau/T + 2 (tau/T)^2 (1 - exp(-T/tau))).
                      "expected_per_sample_std": float(math.sqrt(noise[j] ** 2 + bstd[j] ** 2 * (
                          1 - 2 * btau[j] / duration + 2 * (btau[j] / duration) ** 2 * (1 - math.exp(-duration / btau[j])))))}
        return {"class": "Systems.Sensing.IMU.StatisticalSensor", "duration_s": duration, "parameters": params,
                "channels": out}
    finally:
        s.close()


def run(experiment: Path, out: Path | None = None) -> dict:
    exp = tomllib.loads(Path(experiment).read_text())
    e = exp["experiment"]
    out = Path(out or LIBRARY_ROOT / "build/calibration" / e["name"])
    out.mkdir(parents=True, exist_ok=True)
    src = exp["source"]
    data_dir = LIBRARY_ROOT / src["local_dir"]
    th_path = Path(experiment).parent / e["thresholds"]
    th = tomllib.loads(th_path.read_text())
    rep = {"experiment": e, "experiment_file": rel(experiment),
           "experiment_sha256": file_sha256(Path(experiment)), "source": src,
           "created": _dt.datetime.now(_dt.timezone.utc).isoformat(timespec="seconds"), "versions": toolchain(),
           "thresholds": {"file": rel(th_path), "sha256": file_sha256(th_path), "values": th,
                          "read_before_holdout_evaluation": True}}
    bag = data_dir / src["bag"]
    rep["bag_sha256"] = src.get("bag_sha256") or file_sha256(bag)
    arrays = {}
    for name in src["topics"]:
        p = data_dir / f"{name}.npz"
        if not p.exists():
            extract(bag, data_dir, src["topics"])
        arrays[name] = dict(np.load(p))
    rep["recordings"] = {}
    bounds = st.Bounds(*(tuple(v) for v in (exp["fit"]["sigma_w"], exp["fit"]["sigma_d"], exp["fit"]["tau"])))
    rng = np.random.default_rng(e["seed"])
    rate = float(src["rate_hz"])
    for name, a in arrays.items():
        t = a["time_s"]
        info = {"timing": timing(t, rate), "npz_sha256": file_sha256(data_dir / f"{name}.npz")}
        t0 = t[0]
        train = (t0 + exp["split"]["train_s"][0], t0 + exp["split"]["train_s"][1])
        hold = (t0 + exp["split"]["holdout_s"][0], t0 + exp["split"]["holdout_s"][1])
        x = {"gyro_x": a["gyro"][:, 0], "gyro_y": a["gyro"][:, 1], "gyro_z": a["gyro"][:, 2],
             "accel_x": a["accel"][:, 0], "accel_y": a["accel"][:, 1], "accel_z": a["accel"][:, 2]}
        warm = bin_means(t, x["gyro_z"], 600.0, t0, t0 + 7200)[0].tolist()
        info["warmup_gyro_z_10min_means"] = warm
        info["static_means"] = {c: float(np.mean(x[c][(t >= train[0]) & (t < train[1])])) for c in AXES}
        info["channels"] = {c: analyse_channel(x[c], t, train, hold, bounds, th["holdout"], rng) for c in AXES}
        info["compose_model"] = exercise_model(info["channels"], rate, exp["model"]["duration_s"], e["seed"])
        ref = exp["miluv_reference"].get(name, {})
        ref = {**ref, "per_axis": miluv_log(data_dir / {"px4": "mavros_output.log", "camera": "camera_output.log"}[name])}
        info["miluv_allan_variance_ros"] = ref
        rep["recordings"][name] = info
    if "barometer" in exp:
        rep["barometer"] = barometer_case(exp, data_dir, bag, exp["barometer"]["fit"], th["holdout"], rng)
    (out / "report.json").write_text(json.dumps(rep, indent=1, default=float) + "\n")
    (out / "report.md").write_text(markdown(rep))
    prof = out / f"{e['name']}.imu.toml"
    write_snippet(prof, rep)
    rep["profile"] = {"path": str(prof), "sha256": file_sha256(prof), "text": prof.read_text()}
    rep["toolchain"] = {"pass": all(ch["holdout"]["pass"] is not None for r in rep["recordings"].values()
                                    for ch in r["channels"].values())}
    return rep


def miluv_log(path: Path) -> dict:
    """allan_variance_ros's per-axis white noise from MILUV's output log, in SI
    (deg/sqrt(s) to rad/s/sqrt(Hz); m/s/sqrt(s) is m/s^2/sqrt(Hz))."""
    out = {}
    if not path.exists():
        return out
    for line in path.read_text().splitlines():
        parts = line.split()
        if "Angle Random Walk" in line:
            out[f"gyro_{parts[0].lower()}"] = float(parts[4]) * math.pi / 180
        elif "Velocity Random Walk" in line:
            out[f"accel_{parts[0].lower()}"] = float(parts[4])
    return out


def write_snippet(path: Path, rep: dict):
    L = ["# IMU noise fitted to the MILUV static recordings (tools/calibration, cp_calibration.imu).",
         "# Not an overlay of a COMPOSE part: no part file is established for these chips. Use it as explicit",
         "# [imu] values of an I1/I2 configuration (six channels: accel xyz, then gyro xyz), each value measured",
         f"# from {rep['source']['citation']}, CC BY 4.0.", ""]
    for name, r in rep["recordings"].items():
        order = ["accel_x", "accel_y", "accel_z", "gyro_x", "gyro_y", "gyro_z"]
        ch = r["channels"]
        L += [f"# {name}: {rep['source']['topics'][name]}",
              f"[{name}]", "samplePeriod = " + repr(1 / rep["source"]["rate_hz"]),
              "noiseStd = [" + ", ".join(f"{ch[c]['per_sample_std_if_white']:.6g}" for c in order) + "]   # per sample, from the white noise density",
              "biasStd = [" + ", ".join(f"{ch[c]['fit']['sigma_d']:.6g}" for c in order) + "]",
              "biasTau = [" + ", ".join(f"{ch[c]['fit']['tau']:.6g}" for c in order) + "]", ""]
    path.write_text("\n".join(L))


def markdown(r: dict) -> str:
    L = []
    w = L.append
    e, s = r["experiment"], r["source"]
    f = lambda x, n=3: "—" if x is None else (f"{x:.{n}g}" if isinstance(x, float) else str(x))
    w(f"# IMU noise report: {e['name']}\n\n{e['description']}. Created {r['created']}.\n")
    w(f"Data: {s['citation']} License {s['license']}. File `{s['member']}` from {s['url']} "
      f"(bag sha256 `{r['bag_sha256'][:16]}…`). {s['note']}\n")
    w(f"Split (declared before analysis): train {r['experiment']['split_note']}. Thresholds `{r['thresholds']['file']}`: "
      f"{r['thresholds']['values']['holdout']}.\n")
    for name, rec in r["recordings"].items():
        t = rec["timing"]
        w(f"## {name} ({s['topics'][name]})\n")
        w(f"{t['samples']} samples over {t['span_s']/3600:.2f} h; median period {t['median_dt_s']*1e3:.3f} ms; "
          f"{t['gaps_over_1p5_periods']} gaps over 1.5 periods (largest {t['max_gap_s']:.3f} s); "
          f"{t['non_increasing']} non-increasing stamps. Gyro z 10-min means over the first 2 h: "
          + ", ".join(f"{v:.2e}" for v in rec["warmup_gyro_z_10min_means"]) + " rad/s.\n")
        w("Static means over the training window: " + ", ".join(f"{c} {v:+.4g}" for c, v in rec["static_means"].items())
          + " (gyro rad/s, accel m/s²).\n")
        w("| channel | noise density | allan_variance_ros (MILUV) | per-sample std: white / measured | lag-1 corr | bias σ | bias τ (s) | τ 95 % | held-out Allan in 90 % band | held-out / predicted at shortest, longest τ | refit σ_w / σ_d / τ |\n|---|---|---|---|---|---|---|---|---|---|---|")
        for c, ch in rec["channels"].items():
            unit = "rad/s/√Hz" if c.startswith("gyro") else "m/s²/√Hz"
            iv = ch["tau_interval95"]
            ho = ch["holdout"]
            rf = ch["holdout_refit"]
            mil = rec["miluv_allan_variance_ros"].get("per_axis", {}).get(c)
            w(f"| {c} | {f(ch['noise_density'])} {unit} | {f(mil)} | {f(ch['per_sample_std_if_white'])} / {f(ch['per_sample_std_measured'])} | "
              f"{ch['lag1_autocorrelation']:.2f} | {f(ch['fit']['sigma_d'])} | {f(ch['fit']['tau'])} | [{f(iv[0])}, {f(iv[1])}]"
              f"{'' if ch['tau_identifiable'] else ' (not bounded)'} | {ho['fraction_inside']:.2f} {'pass' if ho['pass'] else '**fail**'} | "
              f"{ho['ratio_to_median_first']:.2f}, {ho['ratio_to_median_last']:.2f} | "
              f"{f(rf['sigma_w'])} / {f(rf['sigma_d'])} / {f(rf['tau'])} |")
        ref = rec["miluv_allan_variance_ros"]
        if ref:
            w(f"\nThe allan_variance_ros column is MILUV's own per-axis result (its output log), converted to SI; its yaml "
              f"gives gyroscope {ref['gyroscope_noise_density']:.3g} and accelerometer {ref['accelerometer_noise_density']:.3g} "
              "(the largest axis). The held-out check fails where the held-out window's values differ from the training "
              "window's (the refit column): a band from one window's fit is narrow at short cluster times.\n")
        cm = rec["compose_model"]
        w(f"**COMPOSE model:** `{cm['class']}` configured with these values, static and level, {cm['duration_s']:.0f} s. "
          "Allan deviation of its output over the fitted model's (median of 20 numpy realizations):\n")
        w("| channel | per-sample std model / expected | Allan ratio at τ = " + ", ".join(f"{t:.0f}" for t in list(cm['channels'].values())[0]['taus_s'][::3]) + " s |\n|---|---|---|")
        for c, ch in cm["channels"].items():
            w(f"| {c} | {f(ch['per_sample_std'])} / {f(ch['expected_per_sample_std'])} | " + ", ".join(f"{v:.2f}" for v in ch["ratio"][::3]) + " |")
        w("")
    if "barometer" in r:
        b = r["barometer"]
        d = b["datasheet"]
        w(f"## Barometer topic ({b['topic']}), screened\n")
        w(f"Candidate part: {b['part']} ({b['part_basis']}). {b['samples']} samples. Median {b['median_pa']:.0f} Pa; "
          f"per-sample noise from consecutive differences {f(b['per_sample_noise_from_differences_pa'])} Pa, against the "
          f"datasheet's {f(d['pressureNoiseStd_low_noise'])} Pa (OSR 4096) to {f(d['pressureNoiseStd_fast'])} Pa (OSR 256). "
          f"Screen (in the experiment; written after a first look had shown the level): level in {b['screen']['level_pa']} Pa, noise at most "
          f"{b['screen']['max_noise_over_datasheet']}x the OSR 256 figure.\n")
        if b["accepted"]:
            ch = b["channel"]
            w(f"Accepted. Noise density {f(ch['noise_density'])} Pa/√Hz; slow component {f(ch['fit']['sigma_d'])} Pa, "
              f"τ {f(ch['fit']['tau'])} s; held-out Allan in band {ch['holdout']['fraction_inside']:.2f}.\n")
        else:
            w("**Rejected.** " + ("The level is not a plausible ambient pressure. " if not b["plausible_level"] else "")
              + ("The sample-to-sample spread is far above any of the part's modes. " if not b["plausible_noise"] else "")
              + "Without knowing how PX4 and MAVROS produced this topic it cannot be attributed to the MS5611, so no "
              "barometer noise is reported from it.\n")
    w("## Limitations\n")
    for x in ["The IMU chips are not identified by the dataset, so no COMPOSE part's datasheet is compared or validated.",
              "The PX4 stream is the autopilot's processed (filtered, resampled) output; a lag-1 correlation well above 0 "
              "and a per-sample spread below the white-noise value show filtering, which the per-sample white model does "
              "not represent. The noise density and bias, from 1 s means, are not affected.",
              "Temperature was not recorded; slow variation in a static record may be environmental, not sensor drift.",
              "One Gauss-Markov bias per channel: where the Allan curve shows a rate random walk beyond the fitted τ, "
              "the model underpredicts long-τ deviation (visible as held-out points above the band).",
              "The bias is fitted on 1 s means; correlation times of a few seconds are approximated."]:
        w(f"- {x}")
    return "\n".join(L) + "\n"
