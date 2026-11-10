"""Sanity tests: what must hold physically, whatever the parameters.

Static checks (always): the composer refuses what cannot be built.
Flight checks (FIRE_TEST_SITL=1; each builds a vehicle and flies ArduPilot's
copter mission check in SITL, a few minutes each): a vehicle that can fly
flies, and one that cannot does not, for the right reason.

    python3 -m unittest tools/test_sanity.py
    FIRE_TEST_SITL=1 python3 -m unittest tools/test_sanity.py -v
"""
from __future__ import annotations

import csv
import math
import os
from pathlib import Path
import re
import shutil
import socket
import subprocess
import tempfile
import time
import unittest

ROOT = Path(__file__).resolve().parents[1]
HOST = ROOT / "runtime" / "hosts" / "ardupilot"
QUAD = (ROOT / "configs" / "quad.toml").read_text()
COMPOSE = ["cargo", "run", "-q", "--manifest-path", str(ROOT / "tools/fire-compose/Cargo.toml"), "--"]
INSTANCE = 5


def env() -> dict:
    """PATH without ~/bin (its clang breaks the builds), with ArduPilot's build Python."""
    e = dict(os.environ)
    home_bin = str(Path.home() / "bin")
    paths = [p for p in e.get("PATH", "").split(":") if p != home_bin]
    e["PATH"] = ":".join(["/data/ardupilot-cpvehicle/venv/bin", *paths])
    return e


def quad(name: str, geometry: str = "", mass: str | None = None, extra: str = "") -> str:
    """configs/quad.toml as model `name`, with geometry lines, a [mass] section and more."""
    t = QUAD.replace('model_name = "ConfiguredQuad"', f'model_name = "{name}"')
    t = t.replace('preset = "QuadX"', 'preset = "QuadX"\n' + geometry)
    if mass is not None:
        t = t[:t.index("[mass]")] + mass + "\n" + t[t.index("[motor]"):]
    return t + "\n" + extra


def run_file(sitl_name: str, physics: str, speedup: int = 4) -> str:
    """A run file: [sitl] and the physics nested under [physics]."""
    lines = []
    for line in physics.splitlines():
        if line.startswith("[["):
            line = "[[physics." + line[2:]
        elif line.startswith("["):
            line = "[physics." + line[1:]
        lines.append(line)
    body = "\n".join(lines).replace("schema_version = 3", "[physics]\nschema_version = 3", 1)
    return f'[sitl]\nname = "{sitl_name}"\nspeedup = {speedup}\n\n{body}\n'


TUBE = """[mass]
mode = "assembled"
[mass.core]
mass = 1.43
inertia = [[0.018, 0.0, 0.0], [0.0, 0.018, 0.0], [0.0, 0.0, 0.036]]
[mass.arm]
part = "easycomposites_cf_tube_16x14"
"""


def max_tilt(log: str) -> float:
    return float(re.search(r"max tilt ([\d.]+) deg", log).group(1))


class StaticSanity(unittest.TestCase):
    def validate(self, text: str, table: str | None = None) -> subprocess.CompletedProcess:
        with tempfile.NamedTemporaryFile("w", suffix=".toml", delete=False) as f:
            f.write(text)
        try:
            return subprocess.run([*COMPOSE, "validate", f.name, *(["--config-table", table] if table else [])],
                                  capture_output=True, text=True, cwd=ROOT, env=env())
        finally:
            os.unlink(f.name)

    def test_overlapping_propellers_are_refused(self):
        # examples/quad_real_parts.toml: APC 12x4.5MR propellers (0.305 m) on a
        # quad, adjacent hubs armLength x sqrt(2) apart: 0.25 m arms fit, 0.2 m do not.
        base = (HOST / "examples" / "quad_real_parts.toml").read_text()
        ok = self.validate(base.replace('preset = "QuadX"', 'preset = "QuadX"\narmLength = 0.25'), "physics")
        self.assertEqual(ok.returncode, 0, ok.stderr)
        bad = self.validate(base.replace('preset = "QuadX"', 'preset = "QuadX"\narmLength = 0.2'), "physics")
        self.assertNotEqual(bad.returncode, 0)
        self.assertIn("overlap", bad.stderr)

    def test_arms_that_bend_or_break_are_refused(self):
        # 16x14 mm carbon tube (64 GPa, 650 MPa) under 10 N of thrust: 1.0 m bends
        # 3.9 % (allowed), 1.5 m 13 % (over the 5 % limit), 12 m breaks.
        self.assertEqual(self.validate(quad("Sane", "armLength = 1.0", TUBE)).returncode, 0)
        bent = self.validate(quad("Sane", "armLength = 1.5", TUBE))
        self.assertIn("bends", bent.stderr)
        broken = self.validate(quad("Sane", "armLength = 12.0", TUBE + "maxArmDeflection = 100.0\n"))
        self.assertIn("breaks", broken.stderr)

    def test_placed_items_cannot_outweigh_the_vehicle(self):
        r = self.validate(quad("Sane", extra="").replace(
            "[mass.aggregate]", "[mass.placements.battery]\nmass = 2.0\nposition = [0.1, 0, 0]\n\n[mass.aggregate]"))
        self.assertNotEqual(r.returncode, 0)


@unittest.skipUnless(os.environ.get("FIRE_TEST_SITL") == "1", "SITL flights: FIRE_TEST_SITL=1")
class FlightSanity(unittest.TestCase):
    """Each case is built (FMU, runtime, ArduPilot) and flown headless on SITL instance 5."""

    @classmethod
    def setUpClass(cls):
        cls.work = Path(tempfile.mkdtemp(prefix="sanity-"))

    @classmethod
    def tearDownClass(cls):
        shutil.rmtree(cls.work, ignore_errors=True)

    def fly(self, name: str, physics: str) -> str:
        path = self.work / f"{name}.toml"
        path.write_text(run_file(name, physics))
        r = subprocess.run([str(HOST / "run.sh"), str(path), "--headless", "--instance", str(INSTANCE)],
                           capture_output=True, text=True, env=env(), timeout=3000)
        log = r.stdout + r.stderr
        (self.work / f"{name}.log").write_text(log)
        return log

    def trace(self, name: str, seconds: float) -> list[dict]:
        """The built vehicle sitting in SITL for `seconds` (no arming): its truth, frame by frame."""
        spec = subprocess.run(["python3", str(HOST / "runspec.py"), str(self.work / f"{name}.toml")],
                              capture_output=True, text=True, check=True).stdout
        v = dict(re.findall(r"^(\w+)=(.*)$", spec, re.M))
        unq = lambda s: s.strip("'")
        out = self.work / f"{name}.trace.csv"
        e = env() | {"SITL_TRACE": str(out), "SITL_TRACE_EVERY": "100"}
        sitl = subprocess.Popen([str(ROOT / "third_party/ardupilot/build/sitl/bin" / unq(v["AUTOPILOT"])),
                                 "--model", f"cpvehicle:{unq(v['NAME'])}", "--defaults", unq(v["PARAMETERS"]),
                                 "--home", unq(v["HOME_LOCATION"]), "--speedup", "1", f"-I{INSTANCE}", "-w"],
                                cwd=self.work, env=e, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        try:
            for _ in range(50):
                try:
                    s = socket.create_connection(("127.0.0.1", 5760 + 10 * INSTANCE), timeout=1)
                    break
                except OSError:
                    time.sleep(0.2)
            time.sleep(seconds)
            s.close()
        finally:
            sitl.terminate()
            sitl.wait(timeout=20)
        with open(out) as f:
            return [r for r in csv.DictReader(l for l in f if not l.startswith("#")) if r["pitch"]]

    def test_the_baseline_flies(self):
        log = self.fly("sanity_base", quad("SanityBase"))
        self.assertIn("PASSED", log, log[-2000:])

    def test_a_cg_outside_the_rotors_tips_over_and_never_flies(self):
        # 0.5 kg of the 1.5 kg moved 0.6 m forward: the CG 0.2 m forward of the
        # reference, beyond the rotors (0.177 m) and the legs.
        physics = quad("SanityCg").replace(
            "[mass.aggregate]", "[mass.placements.battery]\nmass = 0.5\nposition = [0.6, 0.0, 0.0]\n\n[mass.aggregate]")
        log = self.fly("sanity_cg", physics)
        self.assertIn("FAILED", log)
        self.assertNotIn("PASSED", log)
        rows = self.trace("sanity_cg", 6)
        tilt = max(math.degrees(math.acos(max(-1, min(1, math.cos(float(r["roll"])) * math.cos(float(r["pitch"]))))))
                   for r in rows)
        self.assertGreater(tilt, 45, "it should fall over on the ground")

    def test_long_rigid_arms_fly_worse(self):
        # Rigid-body trend (the deflection limit lifted on purpose): authority
        # grows with the arm, inertia with its cube, so a 4 m frame overshoots.
        short = self.fly("sanity_arm_short", quad("SanityArmShort", "armLength = 0.25", TUBE))
        long = self.fly("sanity_arm_long", quad("SanityArmLong", "armLength = 4.0", TUBE + "maxArmDeflection = 100.0\n"))
        self.assertIn("PASSED", short)
        self.assertGreater(max_tilt(long), max_tilt(short) + 10, f"{max_tilt(short)} vs {max_tilt(long)}")

    def test_thrust_below_weight_does_not_leave_the_ground(self):
        # P0 rotors: 4 x kT omegaMax^2 = 4 x 1e-5 x 1000^2 = 40 N; 5 kg weighs 49 N.
        physics = quad("SanityHeavy").replace("mass = 1.5", "mass = 5.0")
        log = self.fly("sanity_heavy", physics)
        self.assertIn("timed out waiting for altitude", log, log[-2000:])


if __name__ == "__main__":
    unittest.main()
