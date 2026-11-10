"""Sensor calibration harness (tools/calibration, docs/sensor-calibration.md).

The statistics, dataset and composer-profile tests always run (NumPy and SciPy).
FIRE_TEST_FMU=1 adds the toolchain tests through the Rumoca-exported FMU (FMPy;
build/venv-calib has it), and FIRE_TEST_CALIBRATION=1 the full DPS310 pilot.
"""
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools/calibration"))
from cp_calibration import dataset, stochastic  # noqa: E402

COMPOSE = ROOT / "tools/fire-compose/target/debug/fire-compose"
T = 0.108


def simulate(n, b, sw, sd, tau, seed):
    rng = np.random.default_rng(seed)
    phi = math.exp(-T / tau)
    d = np.empty(n)
    d[0] = sd * rng.standard_normal()
    for k in range(1, n):
        d[k] = phi * d[k - 1] + sd * math.sqrt(1 - phi * phi) * rng.standard_normal()
    return b + d + sw * rng.standard_normal(n)


BOUNDS = stochastic.Bounds((1e-3, 10.0), (1e-3, 1e3), (0.5, 1e5))


class Stochastic(unittest.TestCase):
    def test_recovers_drift_and_white_noise(self):
        y = simulate(12000, 12.0, 0.2, 3.0, 30.0, 5)
        est = stochastic.fit(y, T, BOUNDS)
        self.assertAlmostEqual(est["sigma_w"], 0.2, delta=0.01)
        for name, truth, grid in (("tau", 30.0, np.geomspace(3, 300, 21)),
                                  ("sigma_d", 3.0, np.geomspace(0.5, 20, 21)),
                                  ("b", 12.0, np.linspace(est["b"] - 6, est["b"] + 6, 21))):
            p = stochastic.profile(y, T, BOUNDS, name, grid, mle=est)
            self.assertTrue(p["identifiable"], name)
            lo, hi = p["interval95"]
            self.assertLessEqual(lo, truth, name)
            self.assertGreaterEqual(hi, truth, name)

    def test_short_record_does_not_resolve_the_drift_time(self):
        y = simulate(280, 12.0, 0.2, 3.0, 300.0, 6)   # 30 s record, tau 300 s
        est = stochastic.fit(y, T, BOUNDS)
        p = stochastic.profile(y, T, BOUNDS, "tau", np.geomspace(1, 5e4, 25), mle=est)
        self.assertTrue(not p["identifiable"] or p["weakly_identified"], p["interval95"])

    def test_white_noise_only_has_white_innovations(self):
        y = 3.0 + 0.5 * np.random.default_rng(7).standard_normal(4000)
        est = stochastic.fit(y, T, BOUNDS, drift=False)
        self.assertAlmostEqual(est["sigma_w"], 0.5, delta=0.02)
        _, nu = stochastic.loglik(y, T, est["b"], est["sigma_w"], 0.0, 1.0, innovations=True)
        self.assertGreater(stochastic.ljung_box(nu)["p_value"], 0.001)

    def test_allan_deviation_of_white_noise_falls_as_one_over_root_tau(self):
        a = stochastic.allan_deviation(np.random.default_rng(8).standard_normal(20000), 1.0)
        t, d = np.array(a["tau_s"]), np.array(a["adev"])
        slope = np.polyfit(np.log(t[:8]), np.log(d[:8]), 1)[0]
        self.assertAlmostEqual(slope, -0.5, delta=0.05)


class Datasets(unittest.TestCase):
    def test_changed_dataset_is_refused(self):
        with tempfile.TemporaryDirectory() as d:
            ds = dataset.write(Path(d) / "x", {"time_s": [0.1, 0.2], "pressure_pa": [1e5, 1e5], "temperature_k": [300, 300]},
                               {"time_s": [0.0], "pressure_pa": [1e5], "temperature_k": [300]}, {"synthetic": True})
            dataset.read(Path(d) / "x", {"sha256": ds.sha256})
            (Path(d) / "x.csv").write_text((Path(d) / "x.csv").read_text().replace("100000.0", "100001.0", 1))
            with self.assertRaises(ValueError):
                dataset.read(Path(d) / "x", {"sha256": ds.sha256})


def part(*args):
    return subprocess.run([str(COMPOSE), "part", "barometer", "infineon_dps310", "--json", *args],
                          capture_output=True, text=True, cwd=ROOT)


@unittest.skipUnless(COMPOSE.exists(), "build tools/fire-compose first")
class ComposerProfiles(unittest.TestCase):
    PROFILE = """[profile]
kind = "barometer"
part = "infineon_dps310"
level = "B1"
unit = "U1"
synthetic = {synthetic}
[parameters]
pressureBias = {{ value = 11.5, class = "measured", dataset = "bench1" }}
pressureDriftTau = {{ value = 40.0, class = "measured", dataset = "bench1" }}
"""

    def test_datasheet_profile_classifies_its_values(self):
        r = json.loads(part().stdout)
        p = r["parameters"]
        self.assertEqual(r["evidence"], "datasheet-derived")
        self.assertEqual(p["pressureNoiseStd"]["class"], "specified")
        self.assertEqual(p["samplePeriod"]["class"], "derived")
        self.assertAlmostEqual(p["samplePeriod"]["value"], 0.108)   # 104.4 ms pressure + 3.6 ms temperature
        self.assertEqual(p["pressureDriftStd"]["class"], "assumed")
        self.assertEqual(p["pressureDriftTau"]["class"], "assumed")
        self.assertTrue(any(u.startswith("pressure drift over time") for u in r["unknown"]))

    def test_no_drift_scenario(self):
        r = json.loads(part("--drift", "none").stdout)
        self.assertEqual(r["parameters"]["pressureDriftStd"]["value"], 0.0)
        self.assertNotIn("pressureDriftTau", r["parameters"])

    def test_period_shorter_than_the_measurement_time_is_refused(self):
        r = part("--set", "samplePeriod = 0.1044")
        self.assertNotEqual(r.returncode, 0)
        self.assertIn("pressure and temperature", r.stderr)

    def test_calibration_profile_overlays_the_part(self):
        with tempfile.TemporaryDirectory() as d:
            for synthetic, evidence in (("false", "measurement-calibrated"), ("true", "synthetic-calibrated (not a device)")):
                f = Path(d) / "p.toml"
                f.write_text(self.PROFILE.format(synthetic=synthetic))
                r = json.loads(part("--calibration", str(f)).stdout)
                self.assertEqual(r["evidence"], evidence)
                self.assertEqual(r["parameters"]["pressureBias"]["value"], 11.5)
                self.assertEqual(r["parameters"]["pressureBias"]["class"], "measured")
                self.assertEqual(r["parameters"]["pressureNoiseStd"]["class"], "specified")
                self.assertEqual(r["calibration"]["measured"], ["pressureBias", "pressureDriftTau"])

    def test_profile_for_another_part_or_without_a_dataset_is_refused(self):
        with tempfile.TemporaryDirectory() as d:
            f = Path(d) / "p.toml"
            f.write_text(self.PROFILE.format(synthetic="false").replace("infineon_dps310", "bosch_bmp388"))
            self.assertIn("fitted for part", part("--calibration", str(f)).stderr)
            f.write_text(self.PROFILE.format(synthetic="false").replace(', dataset = "bench1"', "", 1))
            self.assertIn("names no dataset", part("--calibration", str(f)).stderr)


def _fmpy():
    try:
        import fmpy  # noqa: F401
        return True
    except ImportError:
        return False


@unittest.skipUnless(os.environ.get("FIRE_TEST_FMU") == "1" and _fmpy(), "FIRE_TEST_FMU=1 with FMPy")
class Toolchain(unittest.TestCase):
    def test_exported_barometer(self):
        from cp_calibration import barometer, pipeline
        m = barometer.Model()
        try:
            r = pipeline.toolchain_check(m)
        finally:
            m.close()
        failed = [c for c in r["checks"] if not c["pass"]]
        self.assertFalse(failed, failed)


@unittest.skipUnless(os.environ.get("FIRE_TEST_CALIBRATION") == "1" and _fmpy(), "FIRE_TEST_CALIBRATION=1 with FMPy")
class Pilot(unittest.TestCase):
    def test_dps310_synthetic(self):
        from cp_calibration import pipeline
        r = pipeline.run(ROOT / "tools/calibration/experiments/dps310_synthetic.toml")
        self.assertTrue(r["toolchain"]["pass"])
        for x in r["recovery"]:
            if x["identifiable"]:
                self.assertTrue(x["recovered"], x)
        for case in r["identifiability"].values():
            self.assertTrue(case["flagged"], case)
        self.assertTrue(r["mismatch"]["revealed"])
        for ev in r["holdout"]:
            # The calibrated profile predicts every held-out set better than the
            # datasheet-derived one, and its timing is exact.
            self.assertGreater(ev["calibrated"]["loglik_per_reading"], ev["datasheet_derived"]["loglik_per_reading"])
            self.assertLess(ev["timing"]["max_spacing_error_s"], 1e-6)
            self.assertAlmostEqual(ev["timing"]["first_delivery_s"], ev["timing"]["expected_first_delivery_s"], places=6)
        # The 30 min static hold passes the predeclared thresholds. (The 120 s
        # step set fails the mean-error threshold, whose rationale assumed a
        # 30 min record: the report shows that failure; the threshold is not
        # changed after the fact.)
        static = next(ev for ev in r["holdout"] if ev["dataset"] == "holdout_static")
        self.assertTrue(static["calibrated"]["pass"], static)
        self.assertEqual(r["profile_in_composer"]["evidence"], "synthetic-calibrated (not a device)")


if __name__ == "__main__":
    unittest.main()
