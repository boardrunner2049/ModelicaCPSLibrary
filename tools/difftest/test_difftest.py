"""difftest without simulators: python3 -m unittest tools/difftest/test_difftest.py"""
from pathlib import Path
import math
import sys
import tempfile
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import compare  # noqa: E402
import report  # noqa: E402
import sources  # noqa: E402
from traces import Trace, euler_from_quaternion, quaternion_from_euler  # noqa: E402


def flight(t0=0.0, dt=0.01, seconds=20.0, yaw_rate=0.3, delay=0.0):
    """A vehicle sitting 2 s, climbing to 5 m, then circling while yawing."""
    t = np.arange(0, seconds, dt)
    tt = np.clip(t - 2.0 - delay, 0, None)
    alt = np.minimum(tt, 5.0)
    north, east = 10 * np.sin(0.2 * tt), 10 * (1 - np.cos(0.2 * tt))
    yaw = (yaw_rate * tt + math.pi) % (2 * math.pi) - math.pi
    roll = 0.05 * np.sin(tt)
    pitch = 0.03 * np.cos(tt) * (tt > 0)
    q = quaternion_from_euler(roll, pitch, yaw)
    ch = {"pos_n": north, "pos_e": east, "pos_d": -alt, "vel_n": np.gradient(north, dt),
          "vel_e": np.gradient(east, dt), "vel_d": -np.gradient(alt, dt),
          "qw": q[0], "qx": q[1], "qy": q[2], "qz": q[3], "roll": roll, "pitch": pitch, "yaw": yaw,
          "pwm1": np.where(tt > 0, 1500.0, 1000.0)}
    return Trace(t + t0, ch, {"name": "flight"})


class Traces(unittest.TestCase):
    def test_csv_round_trip_keeps_metadata_and_gaps(self):
        tr = flight()
        tr.channels["baro_alt"] = np.where(np.arange(len(tr.t)) % 3 == 0, -tr.channels["pos_d"], np.nan)
        tr.meta["event.arm"] = "1.5"
        with tempfile.TemporaryDirectory() as d:
            tr.write(Path(d) / "t.csv")
            back = Trace.read(Path(d) / "t.csv")
        self.assertEqual(back.meta["event.arm"], "1.5")
        self.assertTrue(np.allclose(back.channels["pos_n"], tr.channels["pos_n"], atol=1e-6))
        self.assertEqual(np.isnan(back.channels["baro_alt"]).sum(), np.isnan(tr.channels["baro_alt"]).sum())

    def test_angles_interpolate_across_the_wrap(self):
        tr = Trace(np.array([0.0, 1.0]), {"yaw": np.array([3.1, -3.1])})
        mid = tr.at("yaw", np.array([0.5]))[0]
        self.assertAlmostEqual(abs(mid), math.pi, places=2)

    def test_commands_hold_between_samples(self):
        tr = Trace(np.array([0.0, 1.0]), {"pwm1": np.array([1000.0, 2000.0])})
        self.assertEqual(tr.at("pwm1", np.array([0.99]))[0], 1000.0)


class Compare(unittest.TestCase):
    def test_identical_traces_measure_zero(self):
        a = flight()
        r = compare.compare(a, flight(), checks=[{"channel": "*", "max": 1e-9}])
        self.assertTrue(r.ok, compare.table(r))
        names = {m.channel for m in r.measures}
        self.assertTrue({"pos_n", "yaw", "position", "attitude", "path"} <= names)

    def test_yaw_error_wraps(self):
        a = flight()
        b = flight()
        b.channels["yaw"] = compare.traces.wrap(b.channels["yaw"] + 0.1)
        m = {m.channel: m for m in compare.compare(a, b).measures}
        self.assertAlmostEqual(m["yaw"].max, 0.1, places=6)

    def test_takeoff_alignment_and_xcorr_remove_a_delay(self):
        a, b = flight(), flight(t0=100.0, delay=0.73)
        r = compare.compare(a, b, {"align": "takeoff"})
        m = {m.channel: m for m in r.measures}
        self.assertLess(m["pos_d"].max, 0.02)
        r = compare.compare(a, b, {"align": "start", "xcorr": "pos_d", "max_shift": 2.0, "rate": 100})
        self.assertAlmostEqual(r.shift - 100.0 if r.shift > 50 else r.shift, 0.73, delta=0.011)

    def test_path_ignores_timing_position_does_not(self):
        a, b = flight(seconds=40), flight(seconds=40, delay=1.0)  # a closed circuit each
        m = {m.channel: m for m in compare.compare(a, b, {"align": "start"}).measures}
        self.assertGreater(m["position"].max, 1.0)
        self.assertLess(m["path"].max, 0.05)
        off = flight(seconds=40, delay=1.0)
        off.channels["pos_e"] = off.channels["pos_e"] + np.where(off.t > 30, 3.0, 0.0)
        m = {m.channel: m for m in compare.compare(a, off, {"align": "start"}).measures}
        self.assertGreater(m["path"].max, 2.5)  # B wandering off A's track counts

    def test_attitude_measure_is_the_rotation_angle(self):
        a, b = flight(), flight()
        q = quaternion_from_euler(b.channels["roll"] + 0.2, b.channels["pitch"], b.channels["yaw"])
        b.channels.update(qw=q[0], qx=q[1], qy=q[2], qz=q[3])
        m = {m.channel: m for m in compare.compare(a, b).measures}
        self.assertAlmostEqual(m["attitude"].max, 0.2, places=3)

    def test_tolerance_hold_reports_when_b_departs(self):
        a, b = flight(), flight()
        b.channels["pos_d"] = b.channels["pos_d"] + np.where(b.t > 12.0, 2.0, 0.0)
        r = compare.compare(a, b, checks=[{"channel": "pos_d", "tol": 1.0, "hold": 5.0}])
        m = {m.channel: m for m in r.measures}["pos_d"]
        self.assertAlmostEqual(m.exceeds_at, 12.0, delta=0.03)
        self.assertTrue(r.ok)
        r = compare.compare(a, b, checks=[{"channel": "pos_d", "tol": 1.0, "hold": 15.0}])
        self.assertFalse(r.ok)

    def test_a_check_on_a_missing_channel_fails(self):
        r = compare.compare(flight(), flight(), checks=[{"channel": "mag_x", "max": 1.0}])
        self.assertFalse(r.ok)

    def test_report_renders(self):
        r = compare.compare(flight(), flight(delay=0.5), checks=[{"channel": "path", "max": 1.0}])
        page = report.render(r, "test")
        self.assertIn("<svg", page)
        self.assertIn("Tracks from above", page)


class ColumnMaps(unittest.TestCase):
    def test_enu_flu_csv_becomes_ned_frd(self):
        """A ROS/Gazebo-style log (ENU world, FLU body) maps onto the trace frames."""
        a = flight()
        n, e, d = (a.channels[c] for c in ("pos_n", "pos_e", "pos_d"))
        # Body FRD -> NED attitude to body FLU -> ENU: q_enu_flu = q_enu_ned * q * q_frd_flu.
        h = math.sqrt(0.5)
        q = tuple(a.channels[c] for c in ("qw", "qx", "qy", "qz"))
        from traces import quaternion_multiply
        q_enu = quaternion_multiply(quaternion_multiply((0.0, h, h, 0.0), q), (0.0, 1.0, 0.0, 0.0))
        with tempfile.TemporaryDirectory() as dd:
            path = Path(dd) / "gazebo.csv"
            rows = np.column_stack([a.t * 1e9, e, n, -d, *q_enu])
            np.savetxt(path, rows, delimiter=",", header="stamp_ns,x,y,z,ow,ox,oy,oz", comments="")
            tr = sources.map_csv(path, {
                "t": "stamp_ns * 1e-9",
                "channels": {"pos_n": "y", "pos_e": "x", "pos_d": "-z"},
                "attitude": {"frame": "enu_flu", "q": ["ow", "ox", "oy", "oz"]},
            })
        r = compare.compare(a, tr, {"channels": ["pos_*", "roll", "pitch", "yaw", "attitude"]})
        for m in r.measures:
            self.assertLess(m.max, 1e-5, m.channel)

    def test_euler_quaternion_round_trip(self):
        r, p, y = 0.3, -0.2, 2.5
        self.assertTrue(np.allclose(euler_from_quaternion(*quaternion_from_euler(r, p, y)), (r, p, y)))


if __name__ == "__main__":
    unittest.main()
