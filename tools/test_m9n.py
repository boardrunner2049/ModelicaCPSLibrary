"""M9N Modelica and UBX checks; enable FIRE_TEST_OMC or FIRE_TEST_M9N_ENCODER."""
import csv
import json
import math
import os
from pathlib import Path
import shutil
import struct
import subprocess
import unittest

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "build/m9n-validation"


def tool(variable, name, fallback):
    return os.environ.get(variable) or shutil.which(name) or str(Path(fallback))


def rows(path):
    with path.open() as stream:
        return [{key: float(value) for key, value in row.items()}
                for row in csv.DictReader(stream)]


def observations(data, prefix):
    result = {}
    for row in data:
        sequence = int(row[f"{prefix}.navPvt[1]"])
        if sequence:
            result[sequence] = row
    return result


def decode_ubx(data):
    """Independent wire oracle: framing/checksum, not the producer's C++ layout."""
    result = []
    while data:
        if data[:2] != b"\xb5\x62":
            raise AssertionError("UBX preamble mismatch")
        length, = struct.unpack_from("<H", data, 4)
        packet, data = data[:length+8], data[length+8:]
        if len(packet) != length+8:
            raise AssertionError("Truncated UBX packet")
        a = b = 0
        for byte in packet[2:-2]:
            a = (a+byte) & 255
            b = (b+a) & 255
        if packet[-2:] != bytes([a, b]):
            raise AssertionError("UBX checksum mismatch")
        result.append((packet[2], packet[3], packet[6:-2]))
    return result


@unittest.skipUnless(os.environ.get("FIRE_TEST_OMC") == "1", "native Modelica opt-in")
class M9NModelTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        OUT.mkdir(parents=True, exist_ok=True)
        script = ['loadModel(Modelica,{"4.0.0"});',
                  f'loadFile({json.dumps(str(ROOT / "package.mo"))});', 'getErrorString();']
        cases = [("accepted", 3, .05), ("rejected", 1, .05),
                 ("accepted_refined", 3, .01), ("rejected_refined", 1, .01)]
        for name, limit, step in cases:
            (OUT / f"{name}_res.csv").unlink(missing_ok=True)
            script += [f'simulate(ModelicaCPSLibrary.Tests.M9N,stopTime=1.05,'
                       f'numberOfIntervals=105,tolerance=1e-9,outputFormat="csv",'
                       f'fileNamePrefix="{name}",'
                       f'simflags="-maxStepSize={step} -override=pdopLimit={limit}");',
                       'getErrorString();']
        (OUT / "native.mos").write_text("\n".join(script)+"\n")
        omc = tool("OMC", "omc", "C:/Program Files/OpenModelica1.27.0-64bit/bin/omc.exe")
        run = subprocess.run([omc, "native.mos"], cwd=OUT, capture_output=True,
                             text=True, timeout=180)
        log = run.stdout+run.stderr
        (OUT / "native.log").write_text(log)
        if run.returncode or log.count("The simulation finished successfully.") != len(cases):
            raise AssertionError(f"Modelica execution failed: {OUT / 'native.log'}")
        if "Warning:" in log or "Error:" in log:
            raise AssertionError(f"Modelica diagnostic: {OUT / 'native.log'}")
        cls.runs = {name: rows(OUT / f"{name}_res.csv") for name, _, _ in cases}

    def test_acquisition_hold_and_fixed_configuration(self):
        accepted = observations(self.runs["accepted"], "sensor.vendorConfig")
        rejected = observations(self.runs["rejected"], "sensor.vendorConfig")
        for name, flags in [("accepted", 1), ("rejected", 0)]:
            captured = observations(self.runs[name], "sensor.vendorConfig")
            self.assertEqual(list(captured), [1, 2, 3, 4, 5, 6])
            for seq, row in captured.items():
                epoch = (seq-1)*.2
                self.assertAlmostEqual(row["sensor.vendorConfig.navPvt[2]"], epoch, delta=1e-8)
                self.assertEqual(row["sensor.vendorConfig.navPvt[3]"], (seq-1)*200)
                self.assertEqual(row["sensor.vendorConfig.navPvt[4]"], 3)
                self.assertEqual(row["sensor.vendorConfig.navPvt[5]"], flags)
            for row in self.runs[name]:
                if not row["sensor.vendorConfig.navPvt[1]"]:
                    continue
                epoch = row["sensor.vendorConfig.navPvt[2]"]
                self.assertAlmostEqual(epoch, (row["sensor.vendorConfig.navPvt[1]"]-1)*.2, delta=1e-8)
                self.assertEqual(row["sensor.vendorConfig.navPvt[5]"], flags)
                for j, expected in enumerate([10+2*epoch, -4+3*epoch, 1-epoch], 1):
                    self.assertAlmostEqual(row[f"sensor.position[{j}]"], expected, delta=1e-8)
                for j, expected in enumerate([2, 3, -1], 1):
                    self.assertAlmostEqual(row[f"sensor.velocity[{j}]"], expected, delta=1e-8)
        for seq in accepted:
            for field in range(1, 15):
                if field != 5:
                    self.assertAlmostEqual(accepted[seq][f"sensor.vendorConfig.navPvt[{field}]"],
                        rejected[seq][f"sensor.vendorConfig.navPvt[{field}]"], delta=1e-8)

    def test_units_threshold_boundary_and_step_independence(self):
        for name in ["accepted", "rejected"]:
            captured = observations(self.runs[name], "sensor.vendorConfig")
            refined = observations(self.runs[f"{name}_refined"], "sensor.vendorConfig")
            self.assertEqual(list(refined), list(captured))
            for seq, row in captured.items():
                for field in range(1, 15):
                    self.assertAlmostEqual(row[f"sensor.vendorConfig.navPvt[{field}]"],
                        refined[seq][f"sensor.vendorConfig.navPvt[{field}]"], delta=1e-8)
                for field in ["position", "velocity"]:
                    for index in range(1, 4):
                        self.assertAlmostEqual(row[f"sensor.{field}[{index}]"],
                            refined[seq][f"sensor.{field}[{index}]"], delta=1e-8)
                expected = [10, 1, 1, .1, 2, math.sqrt(13), math.degrees(math.atan2(3, 2)), 180, 0]
                for field, value in enumerate(expected, 6):
                    self.assertAlmostEqual(row[f"sensor.vendorConfig.navPvt[{field}]"], value, delta=1e-8)
                self.assertEqual(row["equalLimit.vendorConfig.navPvt[5]"], 0)
                self.assertEqual(row["aboveLimit.vendorConfig.navPvt[5]"], 1)

    def test_unrepresentable_configuration_is_rejected(self):
        for index, setting in enumerate(["pdopLimit=2.01", "pdopLimit=0", "pdopLimit=6553.6",
                                         "reportedPdop=655.36", "samplePeriod=0.039",
                                         "staticHoldSpeedThreshold=-0.01", "staticHoldSpeedThreshold=2.56",
                                         "staticHoldSpeedThreshold=0.015", "staticHoldMaxDistance=-1",
                                         "staticHoldMaxDistance=65536", "staticHoldMaxDistance=0.5",
                                         "staticHoldSpeedThreshold=1,staticHoldMaxDistance=0"]):
            with self.subTest(setting=setting):
                name = f"M9NInvalid{index}"
                invalid = (f'model {name} '
                           f'ModelicaCPSLibrary.Systems.Sensing.GNSS.VendorConfig.M9N sensor({setting}); '
                           f'equation sensor.measuredPosition=zeros(3); sensor.measuredVelocity=zeros(3); end {name};')
                script = ['loadModel(Modelica,{"4.0.0"});',
                          f'loadFile({json.dumps(str(ROOT / "package.mo"))});',
                          f'loadString({json.dumps(invalid)});',
                          f'simulate({name},stopTime=0.01);', 'getErrorString();']
                (OUT / "invalid.mos").write_text("\n".join(script)+"\n")
                omc = tool("OMC", "omc", "C:/Program Files/OpenModelica1.27.0-64bit/bin/omc.exe")
                run = subprocess.run([omc, "invalid.mos"], cwd=OUT, capture_output=True,
                                     text=True, timeout=90)
                log = run.stdout+run.stderr
                (OUT / f"invalid-{index}.log").write_text(log)
                self.assertNotIn("The simulation finished successfully.", log)
                self.assertIn("M9N", log)
                self.assertIn("assertion has been violated during initialization", log)


@unittest.skipUnless(os.environ.get("FIRE_TEST_OMC") == "1", "native Modelica opt-in")
class M9NStaticHoldTests(unittest.TestCase):
    # P/V pairs come from the same analytic trajectory. These are synthetic
    # oracles, not recorded receiver filtering. Binary-valued .25 s boundary
    # cases use exact samples of those trajectories, so mixed event clocks do
    # not turn equality into a few-ULP exceedance through solver-time roundoff.
    cases = {
        "slow": ("1,staticHoldMaxDistance=2", ".25*time", ".25", .2),
        "vertical": ("1,staticHoldMaxDistance=2", ".25*time", ".25", .2),
        "off": ("0", ".25*time", ".25", .2),
        "invalid": ("1,staticHoldMaxDistance=2,pdopLimit=1", ".25*time", ".25", .2),
        "distance": ("2.55,staticHoldMaxDistance=1", "2.5*time", "2.5", .2),
        "maximum": ("2.55,staticHoldMaxDistance=65535", "time", "1", .2),
        "entrySpeed": ("1,staticHoldMaxDistance=2", "time", "1", .2),
        "entryAccel": ("1.25,staticHoldMaxDistance=65535", ".25*time^2", ".5*time", .25),
        "exitAccel": ("1.25,staticHoldMaxDistance=65535", ".25*time+.4375*max(time-1,0)^2",
                      ".25+.875*max(time-1,0)", .25),
        "aboveAccel": ("1.25,staticHoldMaxDistance=65535", ".25*time+.5*max(time-1,0)^2",
                       ".25+max(time-1,0)", .25),
        "exitSpeed": ("1,staticHoldMaxDistance=65535", ".25*time+.25*max(time-1,0)^2",
                      ".25+.5*max(time-1,0)", .25),
        "uncertain": (".04,staticHoldMaxDistance=2", ".01*time^2", ".02*time", .25),
        "uncertainAccel": (".04,staticHoldMaxDistance=2", ".0175*max(time-1,0)^2",
                           ".035*max(time-1,0)", .25),
        "accuracyBoundary": (".05,staticHoldMaxDistance=2", ".0125*time^2", ".025*time", .25),
    }

    @classmethod
    def setUpClass(cls):
        OUT.mkdir(parents=True, exist_ok=True)
        declarations = ['discrete Integer boundaryTick(start=-1,fixed=true);', 'Real boundaryTime;']
        equations = ['when sample(0,.25) then boundaryTick=pre(boundaryTick)+1; end when;',
                     'boundaryTime=.25*boundaryTick;']
        for name, (settings, position, velocity, period) in cls.cases.items():
            declarations.append(f'ModelicaCPSLibrary.Systems.Sensing.GNSS.VendorConfig.M9N {name}('
                f'staticHoldSpeedThreshold={settings},samplePeriod={period});')
            axis = "0,0,VALUE" if name == "vertical" else "VALUE,0,0"
            if period == .25:
                position, velocity = position.replace("time", "boundaryTime"), velocity.replace("time", "boundaryTime")
            equations += [f'{name}.measuredPosition={{{axis.replace("VALUE", position)}}};',
                          f'{name}.measuredVelocity={{{axis.replace("VALUE", velocity)}}};']
        fixture = "model M9NStaticHold\n"+"\n".join(declarations)+"\nequation\n"+"\n".join(equations)+"\nend M9NStaticHold;"
        script = ['loadModel(Modelica,{"4.0.0"});',
                  f'loadFile({json.dumps(str(ROOT / "package.mo"))});',
                  f'loadString({json.dumps(fixture)});', 'getErrorString();']
        for name, step in [("static", .05), ("static_refined", .01)]:
            (OUT / f"{name}_res.csv").unlink(missing_ok=True)
            script += [f'simulate(M9NStaticHold,stopTime=6.55,numberOfIntervals=655,'
                       f'tolerance=1e-9,outputFormat="csv",fileNamePrefix="{name}",'
                       f'simflags="-maxStepSize={step}");', 'getErrorString();']
        (OUT / "static-hold.mos").write_text("\n".join(script)+"\n")
        omc = tool("OMC", "omc", "C:/Program Files/OpenModelica1.27.0-64bit/bin/omc.exe")
        run = subprocess.run([omc, "static-hold.mos"], cwd=OUT, capture_output=True,
                             text=True, timeout=300)
        log = run.stdout+run.stderr
        (OUT / "static-hold.log").write_text(log)
        if run.returncode or log.count("The simulation finished successfully.") != 2 or "Warning:" in log or "Error:" in log:
            raise AssertionError(f"Modelica static-hold execution failed: {OUT / 'static-hold.log'}")
        cls.runs = {name: rows(OUT / f"{name}_res.csv") for name in ["static", "static_refined"]}
        cls.captured = {name: observations(cls.runs["static"], name) for name in cls.cases}

    def check_motion(self, name, sequence, position, velocity):
        row = self.captured[name][sequence]
        expected_p = [0, 0, position] if name == "vertical" else [position, 0, 0]
        expected_v = [0, 0, velocity] if name == "vertical" else [velocity, 0, 0]
        for field, values in [("position", expected_p), ("velocity", expected_v)]:
            for axis, expected in enumerate(values, 1):
                self.assertAlmostEqual(row[f"{name}.{field}[{axis}]"], expected, delta=1e-8,
                                       msg=f"{name} sequence {sequence}: {field}[{axis}]")
        self.assertAlmostEqual(row[f"{name}.navPvt[11]"], abs(expected_v[0]), delta=1e-8)


    def test_first_sample_constant_motion_vertical_and_disabled(self):
        for name in ["slow", "vertical"]:
            self.check_motion(name, 1, 0, .25)
            self.check_motion(name, 2, .05, .25)
            for sequence in list(self.captured[name])[2:]:
                self.check_motion(name, sequence, .1, 0)
        for name, speed in [("off", .25), ("invalid", .25), ("entrySpeed", 1)]:
            for sequence in self.captured[name]:
                self.check_motion(name, sequence, speed*(sequence-1)*.2, speed)
        for sequence in list(self.captured["maximum"])[2:]:
            self.check_motion("maximum", sequence, .4, 0)

    def test_strict_distance_speed_and_acceleration_boundaries(self):
        # Anchor at .4 s/1 m; exactly 1 m displacement stays held at .8 s.
        for sequence in [3, 4, 5]:
            self.check_motion("distance", sequence, 1, 0)
        self.check_motion("distance", 6, 2.5, 2.5)  # Release in this acquisition.
        self.check_motion("distance", 7, 3, 2.5)
        self.check_motion("distance", 8, 3.5, 0)  # Two quiet evaluations latch again.
        for sequence in self.captured["entryAccel"]:
            epoch = (sequence-1)*.25
            self.check_motion("entryAccel", sequence, .25*epoch**2, .5*epoch)
        for sequence in range(3, 11):  # .875 m/s2 equals the low-uncertainty quit limit.
            self.check_motion("exitAccel", sequence, .125, 0)
        self.check_motion("aboveAccel", 5, .125, 0)
        self.check_motion("aboveAccel", 6, .34375, .5)  # 1 m/s2 exceeds .875.
        for sequence in range(3, 20):  # Speed reaches exactly 2 m/s at 4.5 s.
            self.check_motion("exitSpeed", sequence, .125, 0)
        self.check_motion("exitSpeed", 20, 4.703125, 2.125)

    def test_z_speed_accuracy_branch_and_fresh_atomic_metadata(self):
        # At .04 threshold, .02 m/s2 enters only under the .6 factor, and
        # motion between 2x and 3x threshold must remain held.
        for sequence in range(3, 26):
            self.check_motion("uncertain", sequence, .0025, 0)
        self.check_motion("uncertain", 26, .390625, .125)
        for sequence in range(3, 10):  # .035 acceleration is below .04, above .7*.04.
            self.check_motion("uncertainAccel", sequence, 0, 0)
        for sequence in self.captured["accuracyBoundary"]:
            epoch = (sequence-1)*.25
            self.check_motion("accuracyBoundary", sequence, .0125*epoch**2, .025*epoch)
        for name, (_, _, _, period) in self.cases.items():
            refined = observations(self.runs["static_refined"], name)
            self.assertEqual(list(self.captured[name]), list(refined))
            for sequence, row in self.captured[name].items():
                epoch = (sequence-1)*period
                self.assertAlmostEqual(row[f"{name}.navPvt[2]"], epoch, delta=1e-9)
                self.assertEqual(row[f"{name}.navPvt[3]"], math.floor(1000*epoch+.5))
                self.assertEqual(row[f"{name}.navPvt[4]"], 3)
                self.assertEqual(row[f"{name}.navPvt[5]"], 0 if name == "invalid" else 1)
                for index, value in [(6, 10), (7, 1), (8, 1), (9, .1), (10, 2), (13, 180), (14, 0)]:
                    self.assertAlmostEqual(row[f"{name}.navPvt[{index}]"], value, delta=1e-8)
                for field in [f"navPvt[{j}]" for j in range(1, 15)]+[
                        f"{kind}[{j}]" for kind in ["position", "velocity"] for j in range(1, 4)]:
                    self.assertAlmostEqual(row[f"{name}.{field}"], refined[sequence][f"{name}.{field}"], delta=1e-8)
            # Between acquisitions every public output stays on its last epoch.
            for row in self.runs["static"]:
                sequence = int(row[f"{name}.navPvt[1]"])
                if sequence:
                    for field in [f"navPvt[{j}]" for j in range(1, 15)]+[
                            f"{kind}[{j}]" for kind in ["position", "velocity"] for j in range(1, 4)]:
                        self.assertAlmostEqual(row[f"{name}.{field}"],
                            self.captured[name][sequence][f"{name}.{field}"], delta=1e-8)


@unittest.skipUnless(os.environ.get("FIRE_TEST_OMC") == "1", "native Modelica opt-in")
class M9NConfigTests(unittest.TestCase):
    """NAV5/current-UART semantic boundary; native-driver claims are separate.

    Oracles were fixed from SPG 4.04 IF R02 pp.65-66, 236, 250-251 before
    implementation. Unsupported valid settings are coverage gaps, not device NAKs.
    Satellite geometry and proprietary dynamic-model filtering remain unmodeled.
    CFG-PRT uses IF pp.73-75 and an explicitly configured UBX-only UART1
    fixture at 230400 baud, not factory output selections or modeled baud timing.
    MON-VER uses IF pp.15-16,134, with an explicitly synthetic build label.
    """

    @classmethod
    def setUpClass(cls):
        OUT.mkdir(parents=True, exist_ok=True)

        def command(sequence, operation=0, mask=0, **settings):
            # Physical units, not UBX integer scaling. Annex factory fixtures are
            # overridden by the explicitly selected PDOP/static-hold scenario.
            values = [sequence, 6, 36, operation, mask, 0, 3, 0, 1, 5,
                      3, 25, 100, 350, 1, 60, 0, 0, 2, 0]
            fields = {"dyn": 5, "fix": 6, "alt": 7, "variance": 8,
                      "elevation": 9, "pdop": 10, "pacc": 12,
                      "speed": 14, "distance": 18}
            for name, value in settings.items():
                values[fields[name]] = value
            return values

        # Non-selected fields deliberately disagree with the current settings.
        # A masked write must preserve them rather than copying the whole payload.
        cls.commands = [
            (.05, command(1)),
            (.09, command(2, 1, 3, dyn=8, elevation=7, fix=255, alt=-123,
                          variance=999, pdop=0, pacc=0, speed=2.55, distance=65535)),
            (.13, command(3)),
            (.25, command(4, 1, 16, pdop=2)),
            (.29, command(5)),
            (.45, command(6, 1, 16, pdop=3)),
            (.49, command(7)),
            (.65, command(8, 1, 5, dyn=3, fix=1)),  # unsupported group: no partial write
            (.69, command(9)),
            (.85, command(10, 1, 16, pdop=1, pacc=99)),
            (.89, command(11)),
            (1.05, command(12, 1, 0, dyn=1, pdop=0, pacc=0, speed=2.55, distance=0)),
            (1.09, command(13)),
            (1.13, command(14)),  # identical poll payload, distinct request identity
            (1.25, command(15, 1, 1, dyn=1)),
            (1.29, command(16)),
            (1.45, command(17, 1, 64, speed=1, distance=0)),
            (1.49, command(18)),
            (1.65, command(19, 1, 16, pdop=0)),
            (1.69, command(20)),
            (1.73, command(21, 1, 1, dyn=.5)),
            (1.77, command(22)),
            (1.81, command(23, 1, 16, pacc=100.5)),
            (1.83, command(24)),
            (1.835, [25, 6, 0, 0]+[0]*16),  # Empty current-UART poll.
            (1.84, [26, 6, 0, 2]+[0]*16),  # Explicit-port/other shape unmodeled.
            (1.845, command(27)),
            (1.855, [28, 10, 4, 0]+[0]*16),
            (1.865, [29, 10, 4, 2]+[0]*16),
            (1.875, command(30)),
        ]
        switch_commands = [(.05, command(1, 1, 64, speed=1, distance=2)),
                           (.09, command(2)),
                           (.65, command(3, 1, 64, speed=0, distance=0)),
                           (.69, command(4))]

        # The expression is written in ascending decision order: later events
        # must take precedence over earlier events that remain true.
        def sequence_source(timeline):
            expression = "zeros(20)"
            for instant, values in timeline:
                vector = "{"+",".join(format(value, ".17g") for value in values)+"}"
                expression = f"if time>={instant} then {vector} else ({expression})"
            return expression

        fixture = f"""model M9NConfiguration
  Real commands[20], switchCommands[20];
  ModelicaCPSLibrary.Systems.Sensing.GNSS.VendorConfig.M9N receiver(
    staticHoldSpeedThreshold=1,staticHoldMaxDistance=2,configCommand=commands);
  ModelicaCPSLibrary.Systems.Sensing.GNSS.VendorConfig.M9N switched(configCommand=switchCommands);
  ModelicaCPSLibrary.Systems.Sensing.GNSS.VendorConfig.M9N isolated(
    staticHoldSpeedThreshold=1,staticHoldMaxDistance=2,configCommand=zeros(20));
  ModelicaCPSLibrary.Systems.Sensing.GNSS.VendorConfig.M9N accuracySwitch(
    staticHoldSpeedThreshold=1,staticHoldMaxDistance=2,
    configCommand={sequence_source([(.05, command(1, 1, 64, speed=.04, distance=2))])});
equation
  commands={sequence_source(cls.commands)};
  switchCommands={sequence_source(switch_commands)};
  receiver.measuredPosition={{.25*time,0,0}}; receiver.measuredVelocity={{.25,0,0}};
  switched.measuredPosition={{.25*time,0,0}}; switched.measuredVelocity={{.25,0,0}};
  isolated.measuredPosition={{.25*time,0,0}}; isolated.measuredVelocity={{.25,0,0}};
  accuracySwitch.measuredPosition={{.01*time^2,0,0}}; accuracySwitch.measuredVelocity={{.02*time,0,0}};
end M9NConfiguration;
"""
        script = ['loadModel(Modelica,{"4.0.0"});',
                  f'loadFile({json.dumps(str(ROOT / "package.mo"))});',
                  f'loadString({json.dumps(fixture)});', 'getErrorString();']
        for name, step in [("config", .05), ("config_refined", .01)]:
            (OUT / f"{name}_res.csv").unlink(missing_ok=True)
            script += [f'simulate(M9NConfiguration,stopTime=1.9,numberOfIntervals=1900,'
                       f'tolerance=1e-9,outputFormat="csv",fileNamePrefix="{name}",'
                       f'simflags="-maxStepSize={step}");', 'getErrorString();']
        (OUT / "config.mos").write_text("\n".join(script)+"\n")
        omc = tool("OMC", "omc", "C:/Program Files/OpenModelica1.27.0-64bit/bin/omc.exe")
        run = subprocess.run([omc, "config.mos"], cwd=OUT, capture_output=True,
                             text=True, timeout=180)
        log = run.stdout+run.stderr
        (OUT / "config.log").write_text(log)
        if run.returncode or log.count("The simulation finished successfully.") != 2:
            raise AssertionError(f"Modelica configuration execution failed: {OUT / 'config.log'}")
        if "Warning:" in log or "Error:" in log:
            raise AssertionError(f"Modelica diagnostic: {OUT / 'config.log'}")
        cls.runs = {name: rows(OUT / f"{name}_res.csv")
                    for name in ["config", "config_refined"]}

    def response_rows(self, run, sensor):
        return {int(row[f"{sensor}.configResponse[1]"]): row for row in self.runs[run]
                if row[f"{sensor}.configResponse[1]"]}

    def test_readback_masked_writes_and_atomic_coverage_gaps(self):
        captured = self.response_rows("config", "receiver")
        self.assertEqual(list(captured), list(range(1, 31)))
        gaps = {8, 10, 15, 17, 19, 21, 23, 26, 29}
        writes = {2, 4, 6, 12}
        for sequence, row in captured.items():
            expected = [sequence, 6, 36,
                        4 if sequence in gaps else 2 if sequence in writes else 1,
                        0, 8 if sequence >= 2 else 0, 3, 0, 1,
                        7 if sequence >= 2 else 5,
                        2 if sequence in [4, 5] else 3,
                        25, 100, 350, 1, 60, 0, 0, 2, 0]
            if sequence == 25:
                expected = [25, 6, 0, 1, 1, 0, 2240, 230400, 1, 1]+[0]*10
            elif sequence == 26:
                expected[2] = 0
            elif sequence == 28:
                expected = [28, 10, 4, 1, 190000, 4, 4, 32, 1, 9]+[0]*10
            elif sequence == 29:
                expected[1:3] = [10, 4]
            for index, value in enumerate(expected, 1):
                self.assertAlmostEqual(row[f"receiver.configResponse[{index}]"], value, delta=1e-8)
        # Repeated polls are distinct even though their semantic payload is equal.
        for index in range(2, 21):
            self.assertEqual(captured[13][f"receiver.configResponse[{index}]"],
                             captured[14][f"receiver.configResponse[{index}]"])

    def test_off_navigation_commands_and_next_acquisition_effects(self):
        captured = observations(self.runs["config"], "receiver")
        self.assertEqual(list(captured), list(range(1, 11)))
        for sequence, row in captured.items():
            self.assertEqual(row["receiver.navPvt[5]"], 0 if sequence == 3 else 1)
            self.assertAlmostEqual(row["receiver.navPvt[2]"], (sequence-1)*.2, delta=1e-8)
        # All first three requests complete before the second 5 Hz acquisition.
        for sequence in [1, 2, 3]:
            row = next(row for row in self.runs["config"]
                       if row["receiver.configResponse[1]"] == sequence)
            self.assertEqual(row["receiver.navPvt[1]"], 1)
        before_rejection = next(row for row in self.runs["config"] if row["time"] >= .3)
        self.assertEqual(before_rejection["receiver.configResponse[11]"], 2)
        self.assertEqual(before_rejection["receiver.navPvt[5]"], 1)

    def test_static_hold_command_effect_and_instance_isolation(self):
        captured = observations(self.runs["config"], "switched")
        for sequence, row in captured.items():
            epoch = (sequence-1)*.2
            self.assertAlmostEqual(row["switched.position[1]"], .1 if sequence in [3, 4] else .25*epoch,
                                   delta=1e-8)
            self.assertAlmostEqual(row["switched.velocity[1]"], 0 if sequence in [3, 4] else .25,
                                   delta=1e-8)
        replies = self.response_rows("config", "switched")
        self.assertEqual(list(replies), [1, 2, 3, 4])
        for sequence, row in replies.items():
            self.assertEqual(row["switched.configResponse[4]"], 2 if sequence in [1, 3] else 1)
            self.assertEqual(row["switched.configResponse[15]"], 1 if sequence <= 2 else 0)
            self.assertEqual(row["switched.configResponse[19]"], 2 if sequence <= 2 else 0)
        for row in self.runs["config"]:
            self.assertEqual(row["isolated.configResponse[1]"], 0)
            if row["isolated.navPvt[1]"]:
                self.assertEqual(row["isolated.navPvt[5]"], 1)
        # After reducing the threshold to .04 m/s, the .1 m/s uncertainty
        # selects Figure 3's .6 factor: .02 m/s2 is below .024, above .016.
        # Keeping a factor computed from the initial threshold would miss entry.
        for sequence, row in observations(self.runs["config"], "accuracySwitch").items():
            epoch = (sequence-1)*.2
            self.assertAlmostEqual(row["accuracySwitch.position[1]"],
                                   .0016 if sequence >= 3 else .01*epoch**2, delta=1e-8)
            self.assertAlmostEqual(row["accuracySwitch.velocity[1]"],
                                   0 if sequence >= 3 else .02*epoch, delta=1e-8)

    def test_solver_step_independence(self):
        for sensor in ["receiver", "switched", "isolated", "accuracySwitch"]:
            for extractor, field in [(self.response_rows, "configResponse"),
                                     (lambda run, name: observations(self.runs[run], name), "navPvt")]:
                coarse, fine = extractor("config", sensor), extractor("config_refined", sensor)
                self.assertEqual(list(coarse), list(fine))
                for sequence in coarse:
                    for index in range(1, 21 if field == "configResponse" else 15):
                        name = f"{sensor}.{field}[{index}]"
                        self.assertAlmostEqual(coarse[sequence][name], fine[sequence][name], delta=1e-8)


# Compile the actual publisher unchanged; these types stand in for unrelated HAL
# dependencies. Only the captured UBX bytes are accepted as encoder evidence.
STUB = r'''
#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <cassert>
#include <fstream>
#include <iomanip>
#include <stdexcept>
#include <vector>
#define PACKED __attribute__((packed))
#define CLASS_NO_COPY(x)
#define ARRAY_SIZE(x) (sizeof(x)/sizeof((x)[0]))
#define MAX(a,b) std::max(a,b)
inline double radians(double d) { return d*3.141592653589793/180; }
inline double degrees(double r) { return r*180/3.141592653589793; }
inline double ToDeg(double r) { return degrees(r); }
inline double norm(double a,double b,double c=0) { return std::sqrt(a*a+b*b+c*c); }
struct Vector3f {
    double x=0,y=0,z=0;
    Vector3f(double a=0,double b=0,double c=0):x(a),y(b),z(c){}
    Vector3f operator-(Vector3f b)const{return {x-b.x,y-b.y,z-b.z};}
    Vector3f operator*(double b)const{return {x*b,y*b,z*b};}
    double length()const{return norm(x,y,z);}
};
struct Vector2f {
    double x,y; Vector2f(double a,double b):x(a),y(b){}
    double angle()const{return atan2(y,x);}
};
struct Matrix3f {
    void from_euler(double,double,double){} void rotate(Vector3f){}
    Vector3f operator*(Vector3f v)const{return v;}
};
namespace AP_HAL {
inline uint32_t millis(){return 1234;}
[[noreturn]] inline void panic(const char *message, ...) {throw std::runtime_error(message);}
}
namespace SITL {
struct SIM {
    enum {GPS_HEADING_NONE=0};
    int gps_hdg_enabled[2]={}; float gps_delay_ms[2]={};
    struct Offset { Vector3f get()const{return {};} } gps_pos_offset[2];
    double gps_ubx_command[20] {};
    struct {
        double rollRate=0,pitchRate=0,yawRate=0,rollDeg=0,pitchDeg=0;
        struct {struct {struct {bool present=false; double response[20] {};} ubx;} gps;} model_sensors;
    } state;
};
inline SIM fixture;
inline std::vector<uint8_t> output, input;
inline size_t input_offset=0, output_capacity=SIZE_MAX, base_reads=0;
GPS_DATA
class GPS_Backend {
public:
    virtual void publish(const GPS_Data*)=0;
    virtual void update_read(){base_reads++; char byte; read_from_autopilot(&byte,1);}
    struct GPS_TOW {uint16_t week; uint32_t ms;};
    static GPS_TOW gps_time(){return {2200,1234};}
    size_t read_from_autopilot(char*p,size_t n)const {
        n=std::min(n,input.size()-input_offset);
        if(!n) return 0;
        memcpy(p,input.data()+input_offset,n); input_offset+=n; return n;
    }
    size_t write_to_autopilot(const char*p,size_t n)const {
        if(n>output_capacity) return 0;
        output_capacity-=n; output.insert(output.end(),p,p+n); return n;
    }
protected: SIM *_sitl=&fixture; uint8_t instance=0;
};
}
'''

LISTENER_MAIN = r'''
// This fixture supplies semantic replies externally. It proves the actual
// patched UART parser/encoder, not Modelica execution or native driver behavior.
static std::vector<uint8_t> load_bytes(const char *name) {
    std::ifstream stream(name,std::ios::binary);
    return {std::istreambuf_iterator<char>(stream),std::istreambuf_iterator<char>()};
}
static void feed(const std::vector<uint8_t> &bytes) {
    SITL::input.insert(SITL::input.end(),bytes.begin(),bytes.end());
}
static void capture(const char *name) {
    std::ofstream stream(name,std::ios::binary);
    stream.write((const char*)SITL::output.data(),SITL::output.size());
}
static void respond(unsigned sequence,unsigned status) {
    const double values[20] {double(sequence),SITL::fixture.gps_ubx_command[1],
                            SITL::fixture.gps_ubx_command[2],double(status),0,8,3,-12.34,.25,-7,
                            2.1,25,100,350,2.55,60,0,0,65535,0};
    memcpy(SITL::fixture.state.model_sensors.gps.ubx.response,values,sizeof(values));
    if(SITL::fixture.gps_ubx_command[2]==0 && status==1) {
        const double port[20] {double(sequence),6,0,1,1,0,2240,230400,1,1};
        memcpy(SITL::fixture.state.model_sensors.gps.ubx.response,port,sizeof(port));
    }
    if(SITL::fixture.gps_ubx_command[1]==10 && SITL::fixture.gps_ubx_command[2]==4 && status==1) {
        const double identity[20] {double(sequence),10,4,1,190000,4,4,32,1,9};
        memcpy(SITL::fixture.state.model_sensors.gps.ubx.response,identity,sizeof(identity));
    }
}
static void reset_fixture() {
    SITL::fixture={}; SITL::input.clear(); SITL::output.clear();
    SITL::input_offset=0; SITL::output_capacity=SIZE_MAX; SITL::base_reads=0;
}
static void exercise_listener() {
    using namespace SITL;
    const auto poll=load_bytes("poll-input.ubx"), write=load_bytes("write-input.ubx");
    reset_fixture(); fixture.state.model_sensors.gps.ubx.present=true;
    GPS_UBlox listener;
    feed({poll.begin(),poll.begin()+3}); listener.update_read();
    assert(fixture.gps_ubx_command[0]==0 && output.empty());
    feed({poll.begin()+3,poll.end()-1}); listener.update_read();
    assert(fixture.gps_ubx_command[0]==0);
    feed({poll.back()}); listener.update_read();
    assert(fixture.gps_ubx_command[0]==1 && fixture.gps_ubx_command[3]==0);
    feed(write); feed(poll); const size_t queued=input_offset;
    listener.update_read();
    assert(input_offset==queued && output.empty()); // no reply before acceptance

    respond(1,1); output_capacity=0; listener.update_read();
    assert(input_offset==queued && output.empty());
    output_capacity=44; listener.update_read();
    assert(output.size()==44 && input_offset==queued);
    listener.update_read(); assert(output.size()==44); // ACK blocked; no duplicate readback
    output_capacity=10; listener.update_read();
    assert(output.size()==54 && fixture.gps_ubx_command[0]==2);
    assert(input_offset==queued+write.size());
    capture("poll-retry.ubx"); output.clear();
    std::ofstream decoded("decoded-write.txt"); decoded<<std::setprecision(17);
    for(double value:fixture.gps_ubx_command) decoded<<value<<'\n';
    listener.update_read(); assert(output.empty() && input_offset==queued+write.size());
    respond(2,2); output_capacity=10; listener.update_read();
    assert(output.size()==10 && fixture.gps_ubx_command[0]==3);
    capture("write-ack.ubx"); output.clear();
    respond(3,1); output_capacity=1024; listener.update_read();
    capture("second-poll.ubx"); output.clear();

    feed(load_bytes("unsupported-input.ubx")); listener.update_read();
    assert(fixture.gps_ubx_command[0]==4 && fixture.gps_ubx_command[1]==6 && fixture.gps_ubx_command[2]==8);
    respond(4,4); listener.update_read(); assert(output.empty());
    capture("coverage-gap.ubx");
    feed(load_bytes("bad-checksum-input.ubx")); listener.update_read();
    assert(fixture.gps_ubx_command[0]==4 && output.empty());
    feed(poll); listener.update_read(); assert(fixture.gps_ubx_command[0]==5);
    respond(5,1); listener.update_read(); output.clear();

    // VALGET permits repeated keys (IF p.88). Its complete 40-byte payload
    // exceeds the retained NAV5 payload; consume/checksum it without truncating
    // the following frame or pretending the unmodeled command was accepted.
    feed(load_bytes("large-input.ubx")); feed(poll); listener.update_read();
    assert(fixture.gps_ubx_command[0]==6 && fixture.gps_ubx_command[2]==139 &&
           fixture.gps_ubx_command[3]==2);
    respond(6,4); listener.update_read();
    assert(output.empty() && fixture.gps_ubx_command[0]==7 && fixture.gps_ubx_command[3]==0);
    respond(7,1); listener.update_read(); capture("after-large-poll.ubx"); output.clear();
    feed(load_bytes("wrong-shape-input.ubx")); feed(poll); listener.update_read();
    assert(fixture.gps_ubx_command[0]==8 && fixture.gps_ubx_command[3]==2);
    respond(8,4); listener.update_read();
    assert(output.empty() && fixture.gps_ubx_command[0]==9 && fixture.gps_ubx_command[3]==0);
    respond(9,1); listener.update_read(); capture("after-shape-poll.ubx"); output.clear();

    // The current-port poll unblocks native message-rate requests. It has a
    // different 20-byte payload and must preserve the following NAV5 request.
    feed(load_bytes("port-poll-input.ubx")); feed(poll); listener.update_read();
    assert(fixture.gps_ubx_command[0]==10 && fixture.gps_ubx_command[2]==0 &&
           fixture.gps_ubx_command[3]==0);
    respond(10,1); listener.update_read(); capture("port-reply.ubx"); output.clear();
    assert(fixture.gps_ubx_command[0]==11 && fixture.gps_ubx_command[2]==36);
    respond(11,1); listener.update_read(); capture("after-port-poll.ubx"); output.clear();
    feed(load_bytes("explicit-port-input.ubx")); listener.update_read();
    assert(fixture.gps_ubx_command[0]==12 && fixture.gps_ubx_command[3]==2);
    respond(12,4); listener.update_read(); assert(output.empty());
    feed(load_bytes("port-write-input.ubx")); listener.update_read();
    assert(fixture.gps_ubx_command[0]==13 && fixture.gps_ubx_command[3]==2);
    respond(13,4); listener.update_read(); assert(output.empty());

    // MON-VER is a polled output, not a CFG command: one complete identity
    // frame and no ACK, including a capacity-limited retry and queued NAV5.
    feed(load_bytes("identity-poll-input.ubx")); feed(poll); listener.update_read();
    assert(fixture.gps_ubx_command[0]==14 && fixture.gps_ubx_command[1]==10 &&
           fixture.gps_ubx_command[2]==4 && fixture.gps_ubx_command[3]==0);
    respond(14,1); output_capacity=137; listener.update_read();
    assert(output.empty() && fixture.gps_ubx_command[0]==14);
    output_capacity=138; listener.update_read(); capture("identity-reply.ubx"); output.clear();
    assert(fixture.gps_ubx_command[0]==15 && fixture.gps_ubx_command[2]==36);
    respond(15,1); output_capacity=1024; listener.update_read(); capture("after-identity-poll.ubx"); output.clear();
    feed(load_bytes("identity-shape-input.ubx")); listener.update_read();
    assert(fixture.gps_ubx_command[0]==16 && fixture.gps_ubx_command[3]==2);
    respond(16,4); listener.update_read(); assert(output.empty());

    // Repeat with a payload fragmented at every byte, including both checksums.
    reset_fixture(); fixture.state.model_sensors.gps.ubx.present=true;
    GPS_UBlox fragmented;
    for(size_t i=0;i<write.size();i++) {
        feed({write[i]}); fragmented.update_read();
        assert(fixture.gps_ubx_command[0]==(i+1==write.size() ? 1 : 0));
    }
    reset_fixture(); GPS_UBlox stock;
    feed(poll); stock.update_read();
    assert(base_reads==1 && input_offset==1 && fixture.gps_ubx_command[0]==0);

    reset_fixture(); fixture.state.model_sensors.gps.ubx.present=true;
    fixture.gps_ubx_command[0]=INT32_MAX; GPS_UBlox exhausted; feed(poll);
    bool panicked=false;
    try {exhausted.update_read();} catch(const std::runtime_error&) {panicked=true;}
    assert(panicked && fixture.gps_ubx_command[0]==INT32_MAX);
}
'''

MAIN = r'''
int main() {
    SITL::fixture.state.model_sensors.gps.ubx.present=true;
    SITL::GPS_UBlox encoder;
    SITL::GPS_Data d{};
    d.from_model=true; d.itow_ms=400; d.fix_type=3; d.flags=1;
    d.latitude=47.397742; d.longitude=8.5455945; d.altitude=530;
    d.speedN=2; d.speedE=3; d.speedD=-1; d.num_sats=10;
    d.horizontal_acc=1; d.vertical_acc=1; d.speed_acc=.1f;
    d.pdop=2; d.ground_speed=std::sqrt(13); d.course_deg=degrees(atan2(3,2));
    d.course_acc_deg=180; d.geoid_separation=30;
    for(int flag: {1,0}) {
        d.flags=flag; SITL::output.clear(); encoder.publish(&d);
        std::ofstream out(flag ? "accepted.ubx" : "rejected.ubx",std::ios::binary);
        out.write((const char*)SITL::output.data(),SITL::output.size());
    }
    for(float pdop: {2.1f,655.35f}) {
        d.pdop=pdop; SITL::output.clear(); encoder.publish(&d);
        std::ofstream out(pdop<3 ? "pdop_2_1.ubx" : "pdop_max.ubx",std::ios::binary);
        out.write((const char*)SITL::output.data(),SITL::output.size());
    }
    d.pdop=2;
    SITL::fixture.state.model_sensors.gps.ubx.present=false;
    d.from_model=false; d.have_lock=true; SITL::output.clear(); encoder.publish(&d);
    std::ofstream out("stock.ubx",std::ios::binary);
    out.write((const char*)SITL::output.data(),SITL::output.size());
    SITL::output.clear(); SITL::output_capacity=6; encoder.publish(&d);
    assert(SITL::output.size()==6); // Stock header write survives a full payload buffer.
    exercise_listener();
}
'''


@unittest.skipUnless(os.environ.get("FIRE_TEST_M9N_ENCODER") == "1", "patched encoder opt-in")
class M9NEncoderTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        source_root = Path(os.environ.get("M9N_SITL_SOURCE", ROOT / "build/m9n-sitl-source")).resolve()
        if "M9N_SITL_SOURCE" not in os.environ:
            if not source_root.is_relative_to((ROOT / "build").resolve()):
                raise AssertionError("Encoder source overlay must stay inside workspace build/")
            upstream = ROOT / "third_party/ardupilot"
            revision = subprocess.check_output(["git", "-C", str(upstream), "rev-parse", "HEAD"], text=True).strip()
            if revision != "1ebd4d996e8b6bbeb1462a9655c00aa861d5beef":
                raise AssertionError(f"M9N test requires pinned ArduPilot revision, found {revision}")
            patches = [ROOT / "runtime/hosts/ardupilot" / name
                       for name in ["hooks.patch", "gnss.patch"]]
            paths = {line[6:] for patch in patches for line in patch.read_text().splitlines()
                     if line.startswith("--- a/")}
            for path in paths:
                destination = (source_root / path).resolve()
                if not destination.is_relative_to(source_root):
                    raise AssertionError(f"Patch path escapes source overlay: {path}")
                destination.parent.mkdir(parents=True, exist_ok=True)
                destination.write_bytes(subprocess.check_output(["git", "-C", str(upstream), "show", f"HEAD:{path}"]))
            for patch in patches:
                subprocess.run(["git", "apply", f"--directory={source_root.relative_to(ROOT).as_posix()}", str(patch)],
                               cwd=ROOT, check=True, capture_output=True, text=True)
        source = source_root / "libraries/SITL"
        target = OUT / "encoder"
        target.mkdir(parents=True, exist_ok=True)
        (target / "SITL").mkdir(exist_ok=True)
        header = (source / "SIM_GPS.h").read_text()
        gps_data = header[header.index("struct GPS_Data {"):header.index("    // Get course")]+"};"
        (target / "fixture.h").write_text(STUB.replace("GPS_DATA", gps_data))
        (target / "SIM_config.h").write_text("#define AP_SIM_GPS_UBLOX_ENABLED 1\n")
        (target / "SIM_GPS.h").write_text('#include "fixture.h"\n')
        shutil.copyfile(source / "SIM_GPS_UBLOX.h", target / "SIM_GPS_UBLOX.h")
        (target / "SITL/SITL.h").write_text('#include "fixture.h"\n')
        code = (source / "SIM_GPS_UBLOX.cpp").read_text()
        (target / "publisher.cpp").write_text(code+LISTENER_MAIN+MAIN)
        def packet(message_class, message_id, payload=b""):
            body = bytes([message_class, message_id])+struct.pack("<H", len(payload))+payload
            a = b = 0
            for byte in body:
                a, b = (a+byte) & 255, (b+a+byte) & 255
            return b"\xb5\x62"+body+bytes([a, b])

        nav5 = struct.pack("<HBBiIbBHHHHBBBB2xHB5x", 83, 8, 3, -1234, 2500, -7, 0,
                           21, 250, 100, 350, 255, 60, 0, 0, 65535, 0)
        (target / "poll-input.ubx").write_bytes(packet(6, 36))
        (target / "port-poll-input.ubx").write_bytes(packet(6, 0))
        (target / "explicit-port-input.ubx").write_bytes(packet(6, 0, b"\x01"))
        port_payload = struct.pack("<BBHIIHHHH", 1, 0, 0, 2240, 230400, 1, 1, 0, 0)
        (target / "port-write-input.ubx").write_bytes(packet(6, 0, port_payload))
        (target / "write-input.ubx").write_bytes(packet(6, 36, nav5))
        (target / "unsupported-input.ubx").write_bytes(packet(6, 8))
        (target / "identity-poll-input.ubx").write_bytes(packet(10, 4))
        (target / "identity-shape-input.ubx").write_bytes(packet(10, 4, b"\x00"))
        valget = struct.pack("<BBH9I", 0, 0, 0, *([0x20110021]*9))
        (target / "large-input.ubx").write_bytes(packet(6, 139, valget))
        (target / "wrong-shape-input.ubx").write_bytes(packet(6, 36, b"\x00"))
        damaged = bytearray(packet(6, 36, nav5))
        damaged[-1] ^= 1
        (target / "bad-checksum-input.ubx").write_bytes(damaged)
        compiler = tool("CXX", "g++", "C:/mingw64/bin/g++.exe")
        executable = target / ("publisher.exe" if os.name == "nt" else "publisher")
        run = subprocess.run([compiler, "-std=c++17", "-I.", "publisher.cpp", "-o", str(executable)],
                             cwd=target, capture_output=True, text=True, timeout=60)
        (target / "compile.log").write_text(run.stdout+run.stderr)
        if run.returncode:
            raise AssertionError(f"Publisher compile failed: {target / 'compile.log'}")
        environment = dict(os.environ, PATH=str(Path(compiler).resolve().parent)+os.pathsep+os.environ["PATH"])
        subprocess.run([str(executable)], cwd=target, env=environment, check=True, timeout=10)
        cls.target = target
        cls.code = code

    def test_listener_fractional_settings_with_sitl_compiler_flags(self):
        # SITL makes floating-point literals single precision. Legal U2/U1
        # settings must still reach Modelica on its exact 0.1/0.01 grids.
        source = self.target / "native-scales.cpp"
        source.write_text(self.code+LISTENER_MAIN+r'''
int main(int argc, char **argv) {
    reset_fixture(); SITL::fixture.state.model_sensors.gps.ubx.present=true;
    SITL::GPS_UBlox listener;
    feed(load_bytes(argv[1])); listener.update_read();
    assert(SITL::fixture.gps_ubx_command[0]==1);
    std::ofstream out("native-decoded.txt"); out<<std::setprecision(17);
    out<<SITL::fixture.gps_ubx_command[10]<<'\n'<<SITL::fixture.gps_ubx_command[14]<<'\n';
}
''')
        compiler = tool("CXX", "g++", "C:/mingw64/bin/g++.exe")
        executable = self.target / ("native-scales.exe" if os.name == "nt" else "native-scales")
        run = subprocess.run([compiler, "-std=c++17", "-O3", "-fsingle-precision-constant",
                              "-I.", source.name, "-o", str(executable)],
                             cwd=self.target, capture_output=True, text=True, timeout=60)
        (self.target / "native-compile.log").write_text(run.stdout+run.stderr)
        self.assertEqual(run.returncode, 0, run.stdout+run.stderr)
        environment = dict(os.environ, PATH=str(Path(compiler).resolve().parent)+os.pathsep+os.environ["PATH"])
        for pdop, speed in [(11, 1), (11, 255), (21, 1), (21, 255)]:
            with self.subTest(pdop=pdop, speed=speed):
                frame = bytearray((self.target / "write-input.ubx").read_bytes())
                struct.pack_into("<H", frame, 6+14, pdop)  # NAV5 pDop, U2 x 0.1.
                frame[6+22] = speed                       # staticHoldThresh, U1 x 0.01.
                a = b = 0
                for byte in frame[2:-2]:
                    a, b = (a+byte) & 255, (b+a+byte) & 255
                frame[-2:] = bytes([a, b])
                path = self.target / "native-input.ubx"
                path.write_bytes(frame)
                subprocess.run([str(executable), path.name], cwd=self.target, env=environment,
                               check=True, timeout=10)
                actual = [float(value) for value in (self.target / "native-decoded.txt").read_text().split()]
                self.assertEqual(len(actual), 2)
                for value, expected in zip(actual, [pdop/10, speed/100]):
                    self.assertAlmostEqual(value, expected, delta=1e-12)

    def test_listener_decode_order_checksum_and_backpressure(self):
        decoded = [float(value) for value in (self.target / "decoded-write.txt").read_text().split()]
        expected = [2, 6, 36, 1, 83, 8, 3, -12.34, .25, -7,
                    2.1, 25, 100, 350, 2.55, 60, 0, 0, 65535, 0]
        self.assertEqual(len(decoded), 20)
        for actual, wanted in zip(decoded, expected):
            self.assertAlmostEqual(actual, wanted, delta=1e-12)
        expected_nav5 = struct.pack("<HBBiIbBHHHHBBBB2xHB5x", 0, 8, 3, -1234, 2500, -7, 0,
                                   21, 250, 100, 350, 255, 60, 0, 0, 65535, 0)
        for name in ["poll-retry.ubx", "second-poll.ubx", "after-large-poll.ubx", "after-shape-poll.ubx",
                     "after-port-poll.ubx", "after-identity-poll.ubx"]:
            self.assertEqual(decode_ubx((self.target / name).read_bytes()),
                             [(6, 36, expected_nav5), (5, 1, b"\x06\x24")])
        self.assertEqual(decode_ubx((self.target / "write-ack.ubx").read_bytes()),
                         [(5, 1, b"\x06\x24")])
        self.assertEqual((self.target / "coverage-gap.ubx").read_bytes(), b"")
        # IF pp.73-75: portID, reserved, TX-ready, mode, baud, input/output
        # masks, flags, reserved. 8N1 is (3<<6)|(4<<9), reserved bits zero.
        expected_port = struct.pack("<BBHIIHHHH", 1, 0, 0, (3 << 6) | (4 << 9),
                                    230400, 1, 1, 0, 0)
        self.assertEqual(decode_ubx((self.target / "port-reply.ubx").read_bytes()),
                         [(6, 0, expected_port), (5, 1, b"\x06\x00")])
        # IF pp.15-16,134. The build label discloses a synthetic identity;
        # numerical firmware/protocol/hardware/module fields follow the manual.
        identity = struct.pack("<30s10s30s30s30s", b"EXT CORE 4.04 (emulated)", b"00190000",
                               b"FWVER=SPG 4.04", b"PROTVER=32.01", b"MOD=NEO-M9N")
        self.assertEqual(decode_ubx((self.target / "identity-reply.ubx").read_bytes()),
                         [(10, 4, identity)])

    def test_nav_pvt_units_validity_and_stock_branch(self):
        payloads = []
        for name, flags in [("accepted", 1), ("rejected", 0)]:
            packets = decode_ubx((self.target / f"{name}.ubx").read_bytes())
            self.assertEqual([(c, i, len(p)) for c, i, p in packets], [(1, 7, 92)])
            payload = packets[0][2]
            payloads.append(payload)
            self.assertEqual(struct.unpack_from("<I", payload)[0], 400)
            self.assertEqual(tuple(payload[20:24]), (3, flags, 0, 10))
            for offset, value in [(24, 85455945), (28, 473977420), (32, 530000), (36, 500000),
                                  (48, 2000), (52, 3000), (56, -1000), (60, 3605), (64, 5630993)]:
                self.assertAlmostEqual(struct.unpack_from("<i", payload, offset)[0], value, delta=1)
            for offset, value in [(40, 1000), (44, 1000), (68, 100), (72, 18000000)]:
                self.assertEqual(struct.unpack_from("<I", payload, offset)[0], value)
            self.assertEqual(struct.unpack_from("<H", payload, 76)[0], 200)
            self.assertEqual(payload[4:20], bytes(16))
            self.assertEqual(payload[78:84], bytes(6))
            self.assertEqual(struct.unpack_from("<i", payload, 84)[0],
                             struct.unpack_from("<i", payload, 64)[0])
            self.assertEqual(payload[88:], bytes(4))
        self.assertEqual(payloads[0][:21]+payloads[0][22:], payloads[1][:21]+payloads[1][22:])
        for name, expected in [("pdop_2_1", 210), ("pdop_max", 65535)]:
            payload = decode_ubx((self.target / f"{name}.ubx").read_bytes())[0][2]
            self.assertEqual(struct.unpack_from("<H", payload, 76)[0], expected)
        stock = decode_ubx((self.target / "stock.ubx").read_bytes())
        self.assertEqual([i for c, i, p in stock], [2, 3, 18, 6, 4, 7, 48])
        self.assertEqual(next(p[21] for c, i, p in stock if i == 7), 0x83)
        damaged = bytearray((self.target / "accepted.ubx").read_bytes())
        damaged[30] ^= 1
        with self.assertRaisesRegex(AssertionError, "checksum"):
            decode_ubx(damaged)


if __name__ == "__main__":
    unittest.main()
