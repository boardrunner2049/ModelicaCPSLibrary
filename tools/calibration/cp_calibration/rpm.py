"""The pulse-output RPM sensor pipeline (milestone 5).

Model: Systems.Sensing.Rpm.PulseSensor (E1; R2 per shaft in a vehicle).
The event contract is what a consumer such as FastDyn sees at its
communication points: an exact rising-edge count, and the latest edge times
in a bounded buffer. `consume` is that consumer: it recovers every edge since
its previous read, or counts the ones whose times overflowed the buffer.

Calibration compares the reported edge times with an independent reference
(an encoder's true edge times). Synthetic references are the analytic edge
times of the generating shaft motion.
"""
from __future__ import annotations

import datetime as _dt
import json
import math
import subprocess
import tomllib
from pathlib import Path

import numpy as np
from scipy import stats

from . import dataset as dsmod
from . import fitting
from .fmu import LIBRARY_ROOT, Slave, Trace, export, file_sha256, toolchain, rel

E1 = "Systems.Sensing.Rpm.PulseSensor"
RAMP = Path(__file__).resolve().parents[1] / "models/RpmRamp.mo"
COMPOSE = LIBRARY_ROOT / "tools/fire-compose/target/debug/fire-compose"
TWO_PI = 2 * math.pi


def edge_times_constant(speed: float, ppr: float, phase: float, n: int) -> np.ndarray:
    """Rising edge k (1-based) of a shaft at constant speed: cycles = k."""
    f = abs(speed) * ppr / TWO_PI
    return (np.arange(1, n + 1) - phase) / f


def edge_times_ramp(omega0: float, alpha: float, ppr: float, phase: float, n: int) -> np.ndarray:
    """cycles(t) = phase + ppr/(2 pi) (omega0 t + alpha t^2 / 2) = k."""
    k = np.arange(1, n + 1)
    c = (k - phase) * TWO_PI / ppr
    if alpha == 0:
        return c / omega0
    return (-omega0 + np.sqrt(omega0 ** 2 + 2 * alpha * c)) / alpha


def consume(slave: Slave, trace: Trace, params: dict, stop: float, step: float, buffer: int) -> dict:
    """A consumer reading count and the edge buffer every `step` seconds."""
    names = ["count", "edgeTimes"]
    r = slave.run(trace, params, names, stop, step)
    counts = r.outputs["count"].astype(int)
    buf = r.outputs["edgeTimes"]
    edges, times, lost = [], [], 0
    prev = 0
    for c, b in zip(counts, buf):
        new = c - prev
        if new > buffer:
            lost += new - buffer
        got = min(new, buffer)
        for j in range(got):                  # oldest of the new first
            edges.append(c - got + 1 + j)
            times.append(b[got - 1 - j])
        prev = c
    return {"count": int(counts[-1]), "edges": np.array(edges), "times": np.array(times), "lost": lost,
            "max_new_per_step": int(np.max(np.diff(np.concatenate([[0], counts])))) if len(counts) else 0}


def contract_check(slave: Slave, ramp: Slave, buffer: int = 32) -> dict:
    checks, rows = [], []
    c = lambda name, ok, detail: checks.append({"check": name, "pass": bool(ok), "detail": detail})
    ppr, rpm0, phase = 12.0, 6000.0, 0.25
    w = rpm0 * TWO_PI / 60
    stop = 1.0 - 1e-4
    cases = [("constant 6000 rpm", slave, Trace(np.array([0.0]), {"speed": np.array([w])}),
              {"pulsesPerRevolution": ppr, "phaseOffset": phase}, edge_times_constant(w, ppr, phase, 3000)),
             ("accelerating 1000 -> 11000 rpm in 1 s", ramp, Trace(np.array([0.0]), {}),
              {"omega0": 1000 * TWO_PI / 60, "alpha": 10000 * TWO_PI / 60, "pulsesPerRevolution": ppr,
               "phaseOffset": phase}, edge_times_ramp(1000 * TWO_PI / 60, 10000 * TWO_PI / 60, ppr, phase, 4000))]
    for name, s, tr, p, exact in cases:
        n_exact = int(np.sum(exact <= stop))
        for h in (0.0005, 0.001, 0.005, 0.01, 0.02, 0.05):
            got = consume(s, tr, p, stop, h, buffer)
            ok_count = got["count"] == n_exact
            err = float(np.max(np.abs(got["times"] - exact[got["edges"] - 1]))) if len(got["times"]) else 0.0
            expect_loss = got["max_new_per_step"] > buffer
            ok = ok_count and err <= 1e-6 and (got["lost"] > 0) == expect_loss \
                and len(got["edges"]) + got["lost"] == n_exact
            rows.append({"case": name, "step_s": h, "count": got["count"], "expected": n_exact,
                         "recovered": len(got["edges"]), "lost_times": got["lost"],
                         "max_new_per_step": got["max_new_per_step"], "max_time_error_s": err, "pass": bool(ok)})
            c(f"{name}, consumer step {h * 1e3:g} ms", ok,
              f"count {got['count']}/{n_exact}, {len(got['edges'])} edges timed (max error {err:.1e} s)"
              + (f", {got['lost']} times beyond the {buffer}-edge buffer, detected" if got["lost"] else ""))
    r = slave.run(Trace(np.array([0.0]), {"speed": np.array([w])}), {"pulsesPerRevolution": ppr}, ["rpm", "level", "count"],
                  0.1, 1e-4)
    c("rpm from the latest two edges", abs(r.outputs["rpm"][-1] - rpm0) < 1e-6, f"{r.outputs['rpm'][-1]:.9f} rpm")
    duty = float(np.mean(r.outputs["level"][r.time > 0.002]))
    c("logic level high for the duty cycle", abs(duty - 0.5) < 0.02, f"high {duty:.3f} of the time (duty 0.5)")
    j = {"pulsesPerRevolution": ppr, "edgeJitterStd": 1e-5}
    a = consume(slave, Trace(np.array([0.0]), {"speed": np.array([w])}), j, 0.2, 0.005, buffer)
    b = consume(slave, Trace(np.array([0.0]), {"speed": np.array([w])}), j, 0.2, 0.005, buffer)
    d = consume(slave, Trace(np.array([0.0]), {"speed": np.array([w])}), {**j, "seed": 2}, 0.2, 0.005, buffer)
    c("jitter: same seed reproduces, another seed differs", np.array_equal(a["times"], b["times"])
      and not np.array_equal(a["times"], d["times"]), f"{len(a['times'])} edges")
    inert = []
    base = consume(slave, Trace(np.array([0.0]), {"speed": np.array([w])}), {"pulsesPerRevolution": ppr, "edgeJitterStd": 1e-6},
                   0.05, 0.001, buffer)
    for k, v in (("pulsesPerRevolution", 7), ("phaseOffset", 0.5), ("edgeDelay", 1e-4), ("edgeJitterStd", 2e-6),
                 ("seed", 3), ("streamId", 9)):
        x = consume(slave, Trace(np.array([0.0]), {"speed": np.array([w])}),
                    {"pulsesPerRevolution": ppr, "edgeJitterStd": 1e-6, k: v}, 0.05, 0.001, buffer)
        if x["count"] == base["count"] and np.array_equal(x["times"], base["times"]):
            inert.append(k)
    c("every timing parameter changes the edges", not inert, "inert: " + ", ".join(inert) if inert else "all change them")
    return {"checks": checks, "rows": rows, "pass": all(x["pass"] for x in checks)}


def reference_report(part: str, ppr: float, calibration: str | None = None) -> dict:
    cmd = [str(COMPOSE), "part", "rpm", part, "--level", "R2", "--json", "--set", f"pulsesPerRevolution = {ppr}"]
    if calibration:
        cmd += ["--calibration", calibration]
    r = subprocess.run(cmd, capture_output=True, text=True, cwd=LIBRARY_ROOT)
    if r.returncode:
        raise RuntimeError(r.stderr.strip())
    return json.loads(r.stdout)


def generate(slave, ppr, truth, spec, out, meta) -> dsmod.Dataset:
    w = spec["rpm"] * TWO_PI / 60
    dur = float(spec["duration_s"])
    tr = Trace(np.array([0.0]), {"speed": np.array([w])})
    p = {"pulsesPerRevolution": ppr, "phaseOffset": spec.get("sensor_phase", spec.get("phase", 0.0)),
         **{k: v for k, v in truth.items() if k in slave.vars}, "seed": spec["seed"]}
    got = consume(slave, tr, p, dur, spec.get("consumer_step_s", 0.01), 32)
    ref = edge_times_constant(w, ppr, spec.get("phase", 0.0), int(got["edges"].max()) + 1)[got["edges"] - 1]
    return dsmod.write(out / spec["name"], {"time_s": got["times"], "edge": got["edges"].astype(float),
                                           "reference_time_s": ref},
                       {"time_s": np.array([0.0]), "speed_rad_s": np.array([w])},
                       {**meta, "kind": "rpm", "synthetic": True, "stimulus": spec, "truth": p,
                        "time_base": "simulation time; time_s is each edge's reported time as the consumer recovered it",
                        "reference": {"kind": "analytic encoder edge times of the generating shaft (exact)",
                                      "clock_offset_s": 0.0},
                        "conditions": {"rpm": spec["rpm"]}, "lost_edge_times": got["lost"]})


def timing_residual(ds) -> np.ndarray:
    offset = float(ds.meta.get("reference", {}).get("clock_offset_s", 0.0))
    return ds.readings["time_s"] - (ds.readings["reference_time_s"] + offset)


def fit_timing(datasets) -> dict:
    """Edge delay (mean of reported minus reference) and jitter (its spread),
    with standard errors; the residual must be white for these to hold."""
    e = np.concatenate([timing_residual(d) for d in datasets])
    n = len(e)
    delay, jitter = float(np.mean(e)), float(np.std(e, ddof=1))
    f = fitting.fit(lambda p: e - p["edgeDelay"], ["edgeDelay"], {"edgeDelay": 0.0}, {"edgeDelay": (-1, 1)},
                    scale={"edgeDelay": 1e-5})
    lag1 = float(np.corrcoef(e[:-1], e[1:])[0, 1]) if n > 3 else 0.0
    return {"edgeDelay": delay, "edgeDelay_se": f["standard_error"]["edgeDelay"], "edgeJitterStd": jitter,
            "edgeJitterStd_se": jitter / math.sqrt(2 * (n - 1)), "n": n, "lag1_autocorrelation": lag1}


def evaluate(ds, delay: float, jitter: float, th: dict) -> dict:
    e = timing_residual(ds) - delay
    n = len(e)
    ratio = float(np.std(e, ddof=1) / jitter) if jitter > 0 else float("inf")
    res = {"n": n, "mean_error_s": float(np.mean(e)), "std_s": float(np.std(e, ddof=1)), "std_over_jitter": ratio}
    res["checks"] = {"mean_error": abs(res["mean_error_s"]) <= th["mean_error_s"],
                     "jitter": th["std_ratio"][0] <= ratio <= th["std_ratio"][1]}
    res["pass"] = all(res["checks"].values())
    return res


def run(experiment: Path, out: Path | None = None) -> dict:
    exp = tomllib.loads(Path(experiment).read_text())
    e = exp["experiment"]
    out = Path(out or LIBRARY_ROOT / "build/calibration" / e["name"])
    out.mkdir(parents=True, exist_ok=True)
    rep = {"experiment": e, "experiment_file": rel(experiment),
           "experiment_sha256": file_sha256(Path(experiment)),
           "created": _dt.datetime.now(_dt.timezone.utc).isoformat(timespec="seconds"), "versions": toolchain()}
    s = Slave(export(E1))
    ramp = Slave(export("x", wrapper=RAMP, wrapper_model="RpmRamp"))
    try:
        rep["toolchain"] = contract_check(s, ramp)
        ppr = float(e["pulses_per_revolution"])
        rep["reference"] = reference_report(e["part"], ppr)
        truth = exp.get("truth", {})   # absent for recorded data: no recovery
        meta = {"description": e["description"], "unit": e["unit"], "part": e["part"]}
        data = {sp["name"]: dsmod.obtain(sp, Path(experiment).parent,
                                         lambda sp=sp: generate(s, ppr, truth, sp, out / "data", meta), "rpm")
                for sp in exp["datasets"]}
        rep["datasets"] = {n: {**d.summary(), "role": sp["role"], "rpm": sp["rpm"]}
                           for (n, d), sp in zip(data.items(), exp["datasets"])}
        th_path = Path(experiment).parent / e["thresholds"]
        thresholds = tomllib.loads(th_path.read_text())
        rep["thresholds"] = {"file": rel(th_path), "sha256": file_sha256(th_path),
                             "values": thresholds, "read_before_holdout_evaluation": True}
        train = [data[sp["name"]] for sp in exp["datasets"] if sp["role"] == "train"]
        fit = fit_timing(train)
        rep["calibration"] = fit
        rep["recovery"] = [] if not truth else [
            {"parameter": "edgeDelay", "truth": truth["edgeDelay"], "estimate": fit["edgeDelay"],
             "standard_error": fit["edgeDelay_se"], **fitting.recovered(fit["edgeDelay"], truth["edgeDelay"], fit["edgeDelay_se"])},
            {"parameter": "edgeJitterStd", "truth": truth["edgeJitterStd"], "estimate": fit["edgeJitterStd"],
             "standard_error": fit["edgeJitterStd_se"],
             **fitting.recovered(fit["edgeJitterStd"], truth["edgeJitterStd"], fit["edgeJitterStd_se"])}]
        # Delay against an unknown offset between the sensor's and the encoder's clocks.
        e0 = timing_residual(train[0])
        idf = fitting.fit(lambda p: e0 - p["edgeDelay"] - p["clockOffset"], ["edgeDelay", "clockOffset"],
                          {"edgeDelay": 0.0, "clockOffset": 0.0}, {"edgeDelay": (-1, 1), "clockOffset": (-1, 1)},
                          scale={"edgeDelay": 1e-5, "clockOffset": 1e-5})
        rep["identifiability"] = {"delay_and_clock_offset": {
            "question": "the sensor's edge delay and the offset between the sensor's and the reference's clocks, together?",
            "expected": "no: only their sum is determined; a common clock (or a measured offset) is needed",
            "unidentifiable_directions": idf["unidentifiable_directions"], "flagged": bool(idf["unidentifiable_directions"])}}
        # Mismatch: a constant angular lag (a threshold in the field), fitted as
        # a constant time delay (a design check on simulated data; optional for
        # a recorded experiment, which uses the fitted jitter).
        jitter = truth.get("edgeJitterStd", fit["edgeJitterStd"])
        rep["mismatch"] = None
        m = exp.get("mismatch")
        if m:
            lag = m["angular_lag_pulses"]
            mtrain = [generate(s, ppr, {"edgeJitterStd": jitter},
                               {**sp, "sensor_phase": -lag % 1.0, "phase": 0.0, "name": f"mismatch_{sp['name']}"},
                               out / "data", meta) for sp in m["train"]]
            mhold = generate(s, ppr, {"edgeJitterStd": jitter},
                             {**m["holdout"], "sensor_phase": -lag % 1.0, "phase": 0.0}, out / "data", meta)
            # The generator's phase wraps the lag within a pulse: reported edge k is
            # matched to the reference edge it lags.
            def lagged(ds):
                e = timing_residual(ds)
                period = 60 / (ds.meta["conditions"]["rpm"] * ppr)
                return e - np.round((e - lag * period) / period) * period
            mtr = np.concatenate([lagged(d) for d in mtrain])
            speeds = np.concatenate([np.full(len(lagged(d)), d.meta["conditions"]["rpm"]) for d in mtrain])
            mdelay, mjit = float(np.mean(mtr)), float(np.std(mtr, ddof=1))
            means = [float(np.mean(lagged(d))) for d in mtrain]
            inv = [60 / (d.meta["conditions"]["rpm"] * ppr) for d in mtrain]
            reg = stats.linregress(inv, means) if len(mtrain) > 2 else None
            ho = lagged(mhold) - mdelay
            rep["mismatch"] = {
                "generator": f"PulseSensor with a constant angular lag of {lag} pulse periods (edge delay 0)",
                "fitted": "a constant time delay", "fit": {"edgeDelay": mdelay, "edgeJitterStd": mjit},
                "training_delay_by_speed": [{"rpm": d.meta["conditions"]["rpm"], "mean_delay_s": mm} for d, mm in zip(mtrain, means)],
                "training_trend": None if reg is None else {"slope_per_period": float(reg.slope), "p_value": float(reg.pvalue),
                                                            "structured": bool(reg.pvalue < 0.01)},
                "holdout": {"rpm": m["holdout"]["rpm"], "mean_error_s": float(np.mean(ho)),
                            "pass": abs(float(np.mean(ho))) <= thresholds["holdout"]["mean_error_s"]}}
            rep["mismatch"]["revealed"] = bool((reg is not None and reg.pvalue < 0.01) or not rep["mismatch"]["holdout"]["pass"])
        rep["holdout"] = [{"dataset": sp["name"], "rpm": sp["rpm"],
                           "calibrated": evaluate(data[sp["name"]], fit["edgeDelay"], fit["edgeJitterStd"], thresholds["holdout"]),
                           "reference": evaluate(data[sp["name"]], 0.0, max(jitter * 1e-3, 1e-12), thresholds["holdout"])}
                          for sp in exp["datasets"] if sp["role"] == "holdout"]
        prof = out / f"{e['name']}.profile.toml"
        L = ["# Calibration profile written by tools/calibration (cp_calibration.rpm).",
             "# Overlay it: [peripherals.rpm] part = ..., level = \"R2\", calibration = \"<this file>\".", "",
             "[profile]", 'kind = "rpm"', f'part = "{e["part"]}"', 'level = "R2"', f'unit = "{e["unit"]}"',
             f'label = "{e["label"]}"', f'synthetic = {str(bool(e.get("synthetic"))).lower()}', f'created = "{rep["created"]}"',
             f'experiment = "{rep["experiment_file"]}"', f'pulses_per_revolution = {ppr}', ""]
        for n, d in rep["datasets"].items():
            L += [f"[datasets.{n}]", f'role = "{d["role"]}"', f'readings_sha256 = "{d["sha256"]}"', f"rpm = {d['rpm']}", ""]
        dsn = "+".join(n for n, d in rep["datasets"].items() if d["role"] == "train")
        L += ["[parameters]",
              f'edgeDelay = {{ value = {fit["edgeDelay"]!r}, class = "measured", dataset = "{dsn}", how = "mean of reported minus reference edge times", standard_error = {fit["edgeDelay_se"]!r} }}',
              f'edgeJitterStd = {{ value = {fit["edgeJitterStd"]!r}, class = "measured", dataset = "{dsn}", how = "standard deviation of reported minus reference edge times", standard_error = {fit["edgeJitterStd_se"]!r} }}', ""]
        prof.write_text("\n".join(L))
        rep["profile"] = {"path": str(prof), "sha256": file_sha256(prof), "text": prof.read_text()}
        rep["profile_in_composer"] = reference_report(e["part"], ppr, str(prof))
    finally:
        s.close()
        ramp.close()
    (out / "report.json").write_text(json.dumps(rep, indent=1, default=str) + "\n")
    (out / "report.md").write_text(markdown(rep))
    return rep


def markdown(r: dict) -> str:
    L = []
    w = L.append
    e = r["experiment"]
    f = lambda x, n=4: "—" if x is None else (f"{x:.{n}g}" if isinstance(x, float) else str(x))
    w(f"# Calibration report: {e['name']}\n\n{e['description']}. Created {r['created']}; experiment `{r['experiment_file']}`.\n")
    if e.get("synthetic"):
        w("> **Synthetic.** Edge times come from the model and the reference from the generating motion. This tests "
          "the event contract, the implementation and the experiment design, not a real sensor. Sampled rpm cannot "
          "establish edge jitter or latency; edge-level reference data is needed for that.\n")
    v = r["versions"]
    w(f"Rumoca `{v['rumoca']}`, FMPy {v['fmpy']}; library {v['library_revision'][:10]}.\n")
    w("## Event contract and toolchain\n\nA consumer reads `count` and the 32-edge buffer every step and recovers the "
      "edges since its previous read.\n\n| check | result | detail |\n|---|---|---|")
    for c in r["toolchain"]["checks"]:
        w(f"| {c['check']} | {'pass' if c['pass'] else '**FAIL**'} | {c['detail']} |")
    ref = r["reference"]
    w(f"\n## Reference ({ref['name']}, R2)\n\nAssumed: " + "; ".join(ref["assumed"]) + ".\n")
    w("## Datasets\n\n| dataset | role | rpm | edges | sha256 |\n|---|---|---|---|---|")
    for n, d in r["datasets"].items():
        w(f"| {n} | {d['role']} | {d['rpm']} | {d['rows']} | `{d['sha256'][:10]}…` |")
    th = r["thresholds"]
    w(f"\nThresholds `{th['file']}` (sha256 `{th['sha256'][:12]}…`), read before held-out evaluation: {th['values']['holdout']}.\n")
    c = r["calibration"]
    w(f"## Calibration\n\nEdge delay {f(c['edgeDelay']*1e6)} µs ± {f(c['edgeDelay_se']*1e6, 2)}; jitter "
      f"{f(c['edgeJitterStd']*1e6)} µs ± {f(c['edgeJitterStd_se']*1e6, 2)} ({c['n']} edges); lag-1 autocorrelation of the "
      f"timing residual {f(c['lag1_autocorrelation'], 2)}.\n")
    w("## Same-model recovery (|z| ≤ 3)\n")
    w("Not applicable: the experiment states no generating truth (recorded data).\n" if not r["recovery"]
      else "| parameter | truth | estimate | se | z | recovered |\n|---|---|---|---|---|---|")
    for x in r["recovery"]:
        w(f"| `{x['parameter']}` | {f(x['truth'])} | {f(x['estimate'])} | {f(x['standard_error'], 2)} | "
          f"{format(x['z'], '+.2f')} | {'yes' if x['recovered'] else 'no'} |")
    i = r["identifiability"]["delay_and_clock_offset"]
    w(f"\n## Identifiability\n\n- {i['question']} Expected: {i['expected']}. {'Flagged' if i['flagged'] else '**Not flagged**'}: "
      f"{i['unidentifiable_directions']}.\n")
    m = r["mismatch"]
    tt = m and m["training_trend"]
    w("## Model mismatch\n\nNot run: the experiment has no `[mismatch]` design check.\n") if m is None else w(f"## Model mismatch\n\nGenerator: {m['generator']}; fitted: {m['fitted']}. Mean delay by training speed: "
      + ", ".join(f"{x['rpm']} rpm {x['mean_delay_s']*1e6:.2f} µs" for x in m["training_delay_by_speed"])
      + (f". Against the pulse period: slope {f(tt['slope_per_period'], 3)}, p {f(tt['p_value'], 2)} → "
         f"{'**structured: the delay scales with the period (an angle, not a time)**' if tt['structured'] else 'none'}" if tt else "")
      + f". Held out at {m['holdout']['rpm']} rpm: mean error {m['holdout']['mean_error_s']*1e6:+.2f} µs → "
      f"{'pass' if m['holdout']['pass'] else '**fails the threshold**'}. Revealed: {'yes' if m['revealed'] else 'no'}.\n")
    w("## Held-out comparison\n\n| dataset | rpm | profile | mean error (µs) | std (µs) | std / jitter | pass |\n|---|---|---|---|---|---|---|")
    for h in r["holdout"]:
        for p in ("calibrated", "reference"):
            x = h[p]
            w(f"| {h['dataset']} | {h['rpm']} | {p} | {x['mean_error_s']*1e6:+.3f} | {x['std_s']*1e6:.3f} | "
              f"{f(x['std_over_jitter'], 3)} | {'pass' if x['pass'] else 'fail'} |")
    w(f"\nThe reference profile states no delay or jitter (both 0); its ratio is against a negligible jitter.\n")
    w(f"## Profile\n\nThe composer accepts it: evidence **{r['profile_in_composer']['evidence']}**.\n\n```toml\n{r['profile']['text'].strip()}\n```\n")
    w("## Limitations\n")
    for x in (["All data is synthetic; no real sensor's delay or jitter is measured here."] if e.get("synthetic")
              else ["Recorded data: the results hold for this sensor, magnet ring, reference encoder and clock setup only."]) + [
              "Missed pulses, the 10 kHz switching limit and magnet spacing errors are not modelled.",
              "Delay and jitter are characterized only against edge-level reference timestamps on a common clock.",
              "The buffer is 32 edges per shaft by default; a consumer stepping longer than 32 pulse periods keeps an "
              "exact count but loses the older edge times (detected).",
              "The FMU has no co-simulation event mode: the contract relies on the count and buffer, not on "
              "the consumer seeing each event."]:
        w(f"- {x}")
    return "\n".join(L) + "\n"
