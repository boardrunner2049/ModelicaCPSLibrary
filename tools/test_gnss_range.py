"""GNSS/range numerical evidence. FIRE_TEST_OMC=1 enables native Modelica tests."""
import csv
import json
import math
import os
from pathlib import Path
import statistics
import subprocess
import unittest

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "build/gnss-range-validation"


def samples(path, prefix):
    """Keep the last event iteration for each delivered packet, including invalids."""
    result = {}
    with path.open() as stream:
        for row in csv.DictReader(stream):
            sequence = int(float(row[prefix + ".sequence"]))
            if sequence:
                result[sequence] = row
    return result


@unittest.skipUnless(os.environ.get("FIRE_TEST_OMC") == "1", "native Modelica opt-in")
class GnssRangeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        OUT.mkdir(parents=True, exist_ok=True)
        (OUT / "report.json").unlink(missing_ok=True)
        cases = [("GnssStatistical", 4, 2000), ("RangePhysics", 1, 500),
                 ("GnssObservables", .7, 350), ("GnssEphemeris", .02, 20)]
        script = ['loadModel(Modelica,{"4.0.0"});',
                  f'loadFile({json.dumps(str(ROOT / "package.mo"))});', 'getErrorString();']
        for name, stop, intervals in cases:
            (OUT / f"{name}_res.csv").unlink(missing_ok=True)
            script += [f'simulate(fire_modelica_models.Tests.{name},stopTime={stop},'
                       f'numberOfIntervals={intervals},tolerance=1e-9,outputFormat="csv",'
                       f'fileNamePrefix="{name}");', 'getErrorString();']
        script += ['simulate(fire_modelica_models.Tests.GnssStatistical,stopTime=4,'
                   'numberOfIntervals=2000,tolerance=1e-10,outputFormat="csv",'
                   'fileNamePrefix="Refined",simflags="-maxStepSize=0.0007");',
                   'getErrorString();',
                   'simulate(fire_modelica_models.Examples.GnssRange,stopTime=1,'
                   'numberOfIntervals=500,tolerance=1e-9,outputFormat="csv",'
                   'fileNamePrefix="GnssRange");', 'getErrorString();']
        for name in ("Refined", "GnssRange"):
            (OUT / f"{name}_res.csv").unlink(missing_ok=True)
        (OUT / "native.mos").write_text("\n".join(script) + "\n")
        run = subprocess.run(["omc", "native.mos"], cwd=OUT, capture_output=True, text=True, timeout=180)
        log = run.stdout + run.stderr
        (OUT / "native.log").write_text(log)
        if (run.returncode or log.count("The simulation finished successfully.") != 6
                or any(x in log for x in ("Error:", "Warning:", "assertion has been violated"))):
            raise AssertionError(f"Native GNSS/range checks failed: {OUT / 'native.log'}")

    def test_correlated_errors_and_step_independent_replay(self):
        first = samples(OUT / "GnssStatistical_res.csv", "repeatA")
        refined = samples(OUT / "Refined_res.csv", "repeatA")
        self.assertEqual(list(first), list(refined))
        channels = [[] for _ in range(6)]
        bias = []
        for seq, row in first.items():
            for j in range(6):
                key = f'repeatA.{"position" if j < 3 else "velocity"}[{j % 3 + 1}]'
                self.assertEqual(row[key], refined[seq][key])
                channels[j].append(float(row[key]) - float(row[f"repeatA.biasState[{j+1}]"]))
            bias.append(float(row["repeatA.biasState[1]"]))
        self.assertGreater(len(first), 1900)
        covariance_ne = statistics.covariance(channels[0], channels[1])
        covariance_pv = statistics.covariance(channels[0], channels[3])
        lag = statistics.covariance(bias[:-1], bias[1:]) / statistics.variance(bias[:-1])
        self.assertLess(abs(statistics.mean(channels[0])), .08)
        self.assertLess(abs(statistics.pvariance(channels[0]) - 1), .12)
        self.assertLess(abs(covariance_ne - .5), .1)
        self.assertLess(abs(covariance_pv - .2), .04)
        self.assertLess(abs(statistics.pvariance(bias) - .01), .003)
        self.assertLess(abs(lag - math.exp(-.002/.02)), .05)
        self.report = {"samples": len(first), "white_variance": statistics.pvariance(channels[0]),
                       "north_east_covariance": covariance_ne, "position_velocity_covariance": covariance_pv,
                       "bias_lag_correlation": lag, "step_refinement_replay": "exact"}
        (OUT / "statistics.json").write_text(json.dumps(self.report, indent=2) + "\n")

    def test_pvt_clock_geometry_and_delivery(self):
        rows = samples(OUT / "GnssObservables_res.csv", "nominal")
        self.assertEqual(list(rows), list(range(1, 8)))
        max_position_error = 0
        for seq, row in rows.items():
            epoch = float(row["nominal.sampleTime"])
            self.assertAlmostEqual(epoch, (seq-1)*.1, places=10)
            self.assertAlmostEqual(float(row["nominal.deliveryTime"])-epoch, .1, places=10)
            valid = bool(float(row["nominal.valid"]))
            self.assertEqual(valid, seq not in (4, 5))
            if valid:
                for j, (p, v) in enumerate(zip([120, -75, -50], [3, -2, .5]), 1):
                    error = abs(float(row[f"nominal.position[{j}]"])-p-v*epoch)
                    max_position_error = max(error, max_position_error)
                    self.assertLess(error, 1e-4)
                    self.assertAlmostEqual(float(row[f"nominal.velocity[{j}]"]), v, places=6)
                self.assertAlmostEqual(float(row["nominal.clockBias"]), 1e-4+2e-9*epoch, places=12)
            self.assertFalse(float(row["rankDeficient.valid"]))
        (OUT / "navigation.json").write_text(json.dumps({"max_position_error_m": max_position_error,
            "geometry_rejection": "passed", "visibility_loss_and_recovery": "passed"}, indent=2) + "\n")

    def test_range_status_and_constellation_composition(self):
        ranges = samples(OUT / "RangePhysics_res.csv", "delayed")
        self.assertEqual([int(float(ranges[i]["delayed.status"])) for i in range(1, 8)],
                         [1, 1, 2, 2, 1, 5, 1])
        self.assertAlmostEqual(float(ranges[1]["delayed.distance"]), 10.2)
        with (OUT / "GnssRange_res.csv").open() as stream:
            rows = list(csv.DictReader(stream))
        for row in rows:
            if float(row["sequence"]) > 0:
                self.assertTrue(float(row["valid"]))
                epoch = float(row["sampleTime"])
                for j, expected in enumerate([2*epoch, epoch, -20], 1):
                    self.assertAlmostEqual(float(row[f"position[{j}]"]), expected, places=4)
                self.assertTrue(float(row["rangeValid"]))
        self.assertEqual(int(float(rows[-1]["sequence"])), 10)
        self.assertEqual(int(float(rows[-1]["rangeSequence"])), 50)
        (OUT / "report.json").write_text(json.dumps({"native_simulations": 6,
            "profiles": ["G0", "G1", "G2", "L0", "L1"], "broadcast_constellation": "passed",
            "range_status_delivery": "passed", "scope": "OpenModelica native; see statistics.json and navigation.json"}, indent=2) + "\n")


if __name__ == "__main__":
    unittest.main()
