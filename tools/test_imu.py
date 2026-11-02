"""IMU selection and numerical evidence. Set FIRE_TEST_OMC=1 for native/FMI2 tests."""
import cmath
import csv
import ctypes as ct
import json
import math
import os
from pathlib import Path
import statistics
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
BIN = ROOT / "tools/fire-compose/target/debug/fire-compose"
OUT = ROOT / "build/imu-validation"


def samples(path, prefix):
    """Last event iteration for each delivered sample, not solver/output-grid rows."""
    result = {}
    with path.open() as stream:
        for row in csv.DictReader(stream):
            if float(row[prefix + ".valid"]):
                result[int(float(row[prefix + ".sequence"]))] = row
    return result


class ImuTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        subprocess.run(["cargo", "build", "--locked", "--offline", "--manifest-path",
                        str(ROOT / "tools/fire-compose/Cargo.toml")], check=True)

    def emit(self, raw, work, profile="vio", expect=True):
        work.mkdir(parents=True, exist_ok=True)
        source = work / "config.toml"
        source.write_text(raw)
        run = subprocess.run([str(BIN), "emit", str(source), "--profile", profile,
                              "--output-dir", str(work / "generated")], capture_output=True, text=True)
        self.assertEqual(run.returncode == 0, expect, run.stdout + run.stderr)
        return work / "generated"

    def omc(self, work, commands):
        work.mkdir(parents=True, exist_ok=True)
        script = ('loadModel(Modelica,{"4.0.0"});\n'
                  f'loadFile({json.dumps(str(ROOT / "package.mo"))});\ngetErrorString();\n'
                  + "\ngetErrorString();\n".join(commands) + "\ngetErrorString();\n")
        (work / "run.mos").write_text(script)
        run = subprocess.run(["omc", "run.mos"], cwd=work, capture_output=True, text=True, timeout=180)
        log = run.stdout + run.stderr
        (work / "run.log").write_text(log)
        self.assertEqual(run.returncode, 0, log)
        self.assertNotIn("Error:", log, log)
        self.assertNotIn("Warning:", log, log)
        return log

    def test_selection_contract_and_cache(self):
        with tempfile.TemporaryDirectory() as temp:
            work = Path(temp)
            for level in (1, 2, 3):
                raw = (ROOT / f"configs/quad_i{level}.toml").read_text()
                for profile in ("plant", "vio"):
                    out = self.emit(raw, work / f"i{level}_{profile}", profile)
                    manifest = out / f"ConfiguredQuadI{level}.manifest.json"
                    m = json.loads(manifest.read_text())
                    self.assertEqual(m["imu"]["level"], f"I{level}")
                    self.assertEqual(m["core_contract"], "ned_frd_sampled_imu_v1")
                    self.assertTrue(m["event_requirements"]["sampled_sensors"])
                    self.assertFalse(m["event_requirements"]["sampled_actuators"])
                    text = (out / f"ConfiguredQuadI{level}.mo").read_text()
                    self.assertIn("redeclare model ImuModel=", text)
                    for field in ("imuSampleTime", "imuDeliveryTime", "imuSequence", "imuValid"):
                        self.assertIn(field, text)
                    m["verification"]["fmu_simulation"] = "passed"
                    manifest.write_text(json.dumps(m))
                    self.emit(raw.replace("latencySamples = 1", "latencySamples = 2"), out.parent, profile)
                    changed = json.loads(manifest.read_text())
                    self.assertEqual(changed["verification"]["fmu_simulation"], "not_run")
                    self.assertNotEqual(m["generated_model_sha256"], changed["generated_model_sha256"])
            # Legacy explicit ideal remains the same I0 composition.
            ideal = 'schema_version=3\n[models.imu]\nresponse="ideal"\n'
            out = self.emit(ideal, work / "ideal")
            m = json.loads((out / "ConfiguredCopter.manifest.json").read_text())
            self.assertEqual(m["imu"]["level"], "I0")
            self.assertEqual(m["core_contract"], "ned_frd_continuous_v1")
            self.assertFalse(m["event_requirements"]["sampled_sensors"])

    def test_reject_invalid_or_ignored_parameters(self):
        i1 = (ROOT / "configs/quad_i1.toml").read_text()
        i2 = (ROOT / "configs/quad_i2.toml").read_text()
        i3 = (ROOT / "configs/quad_i3.toml").read_text()
        invalid = [
            i1.replace('model = "digital"', 'model = "unknown"'),
            i1.replace("samplePeriod = 0.0025", "samplePeriod = 0"),
            i1 + "noiseStd = [1,1,1,1,1,1]\n",
            i1.replace("latencySamples = 1", "latencySamples = -1"),
            i1.replace("latencySamples = 1", "latencySamples = 10001"),
            i1.replace("adcDecimation = 4", "adcDecimation = 0"),
            i1.replace("analogTau = [0.001", "analogTau = [0.0"),
            i1.replace("quantum = [0.0048", "quantum = [-1"),
            i2.replace("seed = 2718\n", ""),
            i2.replace("seed = 2718", "seed = true"),
            i2.replace("streamId = 1", "streamId = -1"),
            i2.replace("noiseStd = [0.01, 0.01, 0.01, 0.001, 0.001, 0.001]", "noiseStd = [1,2]"),
            i2.replace("0.01, 0.01, 0.01", "nan, 0.01, 0.01"),
            i3.replace("mass = 2.5e-9", "mass = -1"),
            i3.replace("driveQ = 30.0", "driveQ = 0.0"),
            i3.replace("driveQ = 30.0", "driveQ = 30.0\npllRange = 1.0"),
            i3.replace("driveAmplitude = 2e-7", "driveAmplitude = 1"),
            i3.replace("driveQ = 30.0", "driveQ = 30.0\ncrossStiffness = 1e9"),
            i3.replace("seed = 2718", "seed = 2718\nanalogTau = [1,1,1,1,1,1]"),
        ]
        with tempfile.TemporaryDirectory() as temp:
            for i, raw in enumerate(invalid):
                with self.subTest(i=i):
                    out = self.emit(raw, Path(temp) / str(i), expect=False)
                    self.assertFalse(out.exists())

    def test_independent_vulnerability_and_legacy_selection(self):
        with tempfile.TemporaryDirectory() as tmp:
            work=Path(tmp)
            for level, mode, field in [(2,"equivalent_rate","imuAcousticRmsPressure"),(3,"modal_force","imuAcousticPressure")]:
                raw=(ROOT/f"configs/quad_i{level}_acoustic.toml").read_text()
                out=self.emit(raw,work/f"acoustic{level}")
                name=f"ConfiguredQuadI{level}Acoustic"
                m=json.loads((out/f"{name}.manifest.json").read_text())
                self.assertEqual(m["acoustic_vulnerability"]["configuration"]["model"],mode)
                text=(out/f"{name}.mo").read_text()
                self.assertIn(field,text)
                self.assertIn("acousticVulnerability",text)
                self.emit(raw.replace(f'model = "{mode}"', 'model = "invalid"'),work/f"invalid{level}",expect=False)
                self.emit(raw.replace("effectiveArea = 1e-10","effectiveArea = -1"),work/f"area{level}",expect=False)
            for level,name in [(1,"statistical"),(2,"device"),(3,"acoustic")]:
                out=self.emit((ROOT/f"configs/legacy_quad_{name}.toml").read_text(),work/name)
                m=json.loads((out/f"ConfiguredQuadI{level}.manifest.json").read_text())
                self.assertTrue(m["imu"]["legacy"])
                self.assertEqual(m["imu"]["level"],f"legacy_I{level}")

    @unittest.skipUnless(os.environ.get("FIRE_TEST_OMC") == "1", "native Modelica opt-in")
    def test_current_physics_and_acoustic_surrogate(self):
        work=OUT/"current"
        commands=[]
        for name,stop in [("ImuDigitalErrors",.2),("ImuElectromechanical",1),("ImuAcousticCoupling",.5)]:
            commands.append(f'simulate(ModelicaCPSLibrary.Tests.{name},stopTime={stop},numberOfIntervals=1000,'
                f'tolerance=1e-10,outputFormat="csv",fileNamePrefix="{name}",simflags="-maxStepSize=0.00002");')
        commands.append('simulate(ModelicaCPSLibrary.Tests.ImuAcousticCoupling,stopTime=.5,numberOfIntervals=1000,'
                'tolerance=1e-10,outputFormat="csv",fileNamePrefix="Refined",simflags="-maxStepSize=0.00001");')
        log=self.omc(work,commands)
        self.assertEqual(log.count("The simulation finished successfully."),4)
        with (work/"ImuElectromechanical_res.csv").open() as f: physical=list(csv.DictReader(f))
        self.assertLess(abs(float(physical[0]["imu.gyro[1]"])),1e-12,"I3 must not bypass mechanics with truth")
        final=physical[-1]
        rate_error=max(abs(float(final[f"imu.gyro[{j}]"])-expected) for j,expected in enumerate([.2,-.3,.1],1))
        self.assertLess(rate_error,.003)
        with (work/"ImuAcousticCoupling_res.csv").open() as f: acoustic=list(csv.DictReader(f))
        with (work/"Refined_res.csv").open() as f: refined={round(float(x["time"]),10):x for x in csv.DictReader(f)}
        errors=[]; convergence=[]
        for row in acoustic:
            t=float(row["time"])
            if .15<t<.49:
                y=float(row["forced.measuredRate"])
                errors.append(abs(y-float(row["equivalent.equivalentError[6]"])))
                convergence.append(abs(y-float(refined[round(t,10)]["forced.measuredRate"])))
        self.assertGreater(len(errors),500)
        self.assertLess(max(errors),.002)
        self.assertLess(max(convergence),1e-4)
        report={"zero_error_I2_equals_I1":"passed", "physical_gyro_max_rate_error_rad_s":rate_error,
            "startup_PLL_frequency_Hz":float(final["startup.driveControl.frequency"])/(2*math.pi),
            "startup_AGC_amplitude_m":float(final["startup.driveControl.amplitude"]),
            "acoustic_surrogate_max_error_rad_s":max(errors),"step_refinement_max_error_rad_s":max(convergence),
            "power_pressure_units":"passed","zero_exposure":"passed","AGC_drive_disturbance":"passed"}
        (work/"report.json").write_text(json.dumps(report,indent=2)+"\n")

    @unittest.skipUnless(os.environ.get("FIRE_TEST_OMC") == "1", "native/FMI2 opt-in")
    def test_acoustic_composed_fmi2_inputs_have_effect(self):
        from probe_export import Fmi2CS
        work=OUT/"acoustic-fmi2"
        commands=['setCommandLineOptions("--fmiFlags=s:cvode");']
        for level in (2,3):
            name=f"ConfiguredQuadI{level}Acoustic"
            out=self.emit((ROOT/f"configs/quad_i{level}_acoustic.toml").read_text(),work/f"i{level}")
            commands.extend([f'loadFile({json.dumps(str(out/(name+".mo")))});',
                f'simulate({name},stopTime=.02,numberOfIntervals=200,tolerance=1e-9,fileNamePrefix="Native{level}");',
                f'buildModelFMU({name},version="2.0",fmuType="cs",fileNamePrefix="{name}",platforms={{"static"}});'])
            (work/(name+".fmu")).unlink(missing_ok=True)
        log=self.omc(work,commands)
        self.assertEqual(log.count("The simulation finished successfully."),2)
        report={}
        for level in (2,3):
            histories=[]
            field="imuAcousticPressure" if level==3 else "imuAcousticRmsPressure"
            # OMC CVODE can fail on a stop exactly at an ADC event; use a non-aligned grid.
            dt=.00002/math.sqrt(2)
            for amplitude in (0.,.01):
                host=Fmi2CS(work/f"ConfiguredQuadI{level}Acoustic.fmu",work/f"run{level}_{amplitude}",4)
                try:
                    self.assertEqual(host.variables[field].attrib["causality"],"input")
                    self.assertEqual(float(host.variables[field].find("Real").attrib["start"]),0.)
                    host.initialize([1100]*4,.12)
                    values=[]
                    for i in range(int(.12/dt)):
                        t=i*dt
                        pressure=amplitude*math.sqrt(2)*math.cos(2*math.pi*1001*t) if level==3 else amplitude
                        host._check(host.fmi2SetReal(host.component,host._references([field]),1,(ct.c_double*1)(pressure)),"pressure input")
                        host.step(t,dt)
                        if i%25==24: values.append(host.snapshot()["gyro[3]"])
                    histories.append(values)
                finally: host.close()
            difference=max(abs(a-b) for a,b in zip(*histories))
            self.assertGreater(difference,.01,"Exported acoustic input must affect gyro measurements")
            report[f"I{level}"]={"input":field,"max_excited_minus_quiet_rad_s":difference,"communication_step_s":dt}
        (work/"report.json").write_text(json.dumps(report,indent=2)+"\n")

    @unittest.skipUnless(os.environ.get("FIRE_TEST_OMC") == "1", "native Modelica opt-in")
    def test_native_profiles_statistics_replay_and_acoustic_convergence(self):
        work = OUT / "physics"
        commands = []
        for name, stop, intervals in (("ImuStatistical", 1, 1000), ("ImuDevice", .05, 500),
                                      ("ImuAcoustic", 1, 1000)):
            commands.append(f'simulate(ModelicaCPSLibrary.Tests.{name},stopTime={stop},'
                            f'numberOfIntervals={intervals},tolerance=1e-10,outputFormat="csv",'
                            f'fileNamePrefix="{name}",simflags="-maxStepSize=0.0001");')
        commands += [
            'simulate(ModelicaCPSLibrary.Tests.ImuStatistical,stopTime=1,numberOfIntervals=2000,'
            'tolerance=1e-8,outputFormat="csv",fileNamePrefix="Replay",simflags="-maxStepSize=0.0003");',
            'simulate(ModelicaCPSLibrary.Tests.ImuAcoustic,stopTime=1,numberOfIntervals=2000,'
            'tolerance=1e-10,outputFormat="csv",fileNamePrefix="Refined",simflags="-maxStepSize=0.00005");',
        ]
        log = self.omc(work, commands)
        self.assertEqual(log.count("The simulation finished successfully."), 5)
        a = samples(work / "ImuStatistical_res.csv", "repeatA")
        b = samples(work / "Replay_res.csv", "repeatA")
        self.assertEqual(set(a), set(b))
        noise, bias = [], []
        for seq, row in a.items():
            for channel in range(1, 4):
                for field in (f"acceleration[{channel}]", f"gyro[{channel}]", f"biasState[{channel}]"):
                    self.assertEqual(float(row["repeatA." + field]), float(b[seq]["repeatA." + field]))
            drift = float(row["repeatA.biasState[1]"])
            bias.append(drift)
            noise.append(float(row["repeatA.acceleration[1]"]) - drift)
        self.assertLess(abs(statistics.mean(noise)), .025)
        self.assertLess(abs(statistics.pvariance(noise) - .04), .008)
        lag = statistics.covariance(bias[:-1], bias[1:]) / statistics.variance(bias[:-1])
        self.assertLess(abs(lag - math.exp(-.001/.02)), .06)
        reference = samples(work / "ImuAcoustic_res.csv", "acoustic")
        refined = samples(work / "Refined_res.csv", "acoustic")
        frequency, natural, damping, tau = 110, 100, .1, .0001
        w = 2 * math.pi * frequency
        mode = 1 / (1 - (frequency/natural)**2 + 2j*damping*frequency/natural)
        transfer = mode / (1 + 1j*w*tau)
        errors, convergence, rectification = [], [], []
        for seq, row in reference.items():
            t = float(row["acoustic.sampleTime"])
            if .6 <= t < 1-1e-10:
                y = float(row["acoustic.acceleration[1]"])
                errors.append(abs(y - (transfer * cmath.exp(1j*w*t)).imag))
                convergence.append(abs(y - float(refined[seq]["acoustic.acceleration[1]"])))
                rectification.append(float(row["rectified.acceleration[1]"]))
        self.assertEqual(len(errors), 40)
        self.assertLess(max(errors), 1e-5)
        self.assertLess(max(convergence), 1e-5)
        self.assertLess(abs(statistics.mean(rectification) - .5*abs(mode)**2), 1e-5)
        report = {"noise_mean": statistics.mean(noise), "noise_variance": statistics.pvariance(noise),
                  "bias_lag_correlation": lag, "acoustic_max_analytic_error": max(errors),
                  "acoustic_step_refinement_error": max(convergence), "alias_frequency_hz": 10,
                  "rectified_mean": statistics.mean(rectification), "rng_replay": "exact"}
        (work / "report.json").write_text(json.dumps(report, indent=2) + "\n")

    @unittest.skipUnless(os.environ.get("FIRE_TEST_OMC") == "1", "native Modelica opt-in")
    def test_composed_quad_hexa_plant_and_vio(self):
        work = OUT / "composed"
        commands, names = [], []
        for level in (1, 2, 3):
            for preset in ("QuadX", "HexaX"):
                for profile in ("plant", "vio"):
                    name = f"Imu{level}{preset}{profile.title()}"
                    # Reuse illustrative sensor settings with the selected vehicle defaults.
                    raw = (ROOT / f"configs/quad_i{level}.toml").read_text()
                    sensor = raw[raw.index("[models.imu]"):]
                    raw = f'schema_version=3\nmodel_name="{name}"\n[geometry]\npreset="{preset}"\n' + sensor
                    out = self.emit(raw, work / name, profile)
                    commands += [f'loadFile({json.dumps(str(out / (name + ".mo")))});',
                                 f'simulate({name},stopTime=0.0201,numberOfIntervals=201,'
                                 f'tolerance=1e-9,outputFormat="csv",fileNamePrefix="Run{name}");']
                    names.append(name)
        log = self.omc(work, commands)
        self.assertEqual(log.count("The simulation finished successfully."), len(names))
        for name in names:
            with (work / ("Run" + name + "_res.csv")).open() as stream:
                row = list(csv.DictReader(stream))[-1]
            self.assertEqual(int(float(row["imuSequence"])), 8)
            self.assertEqual(float(row["imuValid"]), 1)
            self.assertAlmostEqual(float(row["imuSampleTime"]), .0175, places=10)
            self.assertAlmostEqual(float(row["imuDeliveryTime"]), .0200, places=10)

    @unittest.skipUnless(os.environ.get("FIRE_TEST_OMC") == "1", "FMI2 opt-in")
    def test_fmi2_delivery_and_instance_replay(self):
        from probe_export import Fmi2CS
        work = OUT / "fmi2"
        commands = ['setCommandLineOptions("--fmiFlags=s:cvode");']
        for level in (1, 2, 3):
            raw = (ROOT / f"configs/quad_i{level}.toml").read_text()
            out = self.emit(raw, work / f"i{level}")
            name = f"ConfiguredQuadI{level}"
            commands += [f'loadFile({json.dumps(str(out / (name + ".mo")))});',
                         f'buildModelFMU({name},version="2.0",fmuType="cs",'
                         f'fileNamePrefix="{name}",platforms={{"static"}});']
        for level in (1, 2, 3):
            (work / f"ConfiguredQuadI{level}.fmu").unlink(missing_ok=True)
        self.omc(work, commands)
        metrics = {}
        for level in (1, 2, 3):
            fmu = work / f"ConfiguredQuadI{level}.fmu"
            self.assertTrue(fmu.is_file())
            results = []
            for instance in (1, 2):
                host = Fmi2CS(fmu, work / f"unpacked_i{level}_{instance}", 4)
                try:
                    for field, kind in (("imuValid", "Boolean"), ("imuSequence", "Integer"),
                                        ("imuSampleTime", "Real"), ("imuDeliveryTime", "Real")):
                        self.assertEqual(host.variables[field].attrib["causality"], "output")
                        self.assertIsNotNone(host.variables[field].find(kind))
                    if level>=2:
                        self.assertEqual(host.variables["imuTemperature"].attrib["causality"], "input")
                    if level==3:
                        for field in ("imuGyroDriveForce[1]", "imuGyroSenseForce[1]", "imuAccelForce[1]"):
                            self.assertEqual(host.variables[field].attrib["causality"], "input")
                        self.assertEqual(host.variables["imuGyroLocked[1]"].attrib["causality"], "output")
                    host.observables += ["imuSampleTime", "imuDeliveryTime"]
                    host._bind("fmi2GetInteger", [ct.c_void_p, ct.POINTER(ct.c_uint), ct.c_size_t, ct.POINTER(ct.c_int)])
                    host._bind("fmi2GetBoolean", [ct.c_void_p, ct.POINTER(ct.c_uint), ct.c_size_t, ct.POINTER(ct.c_int)])
                    host.initialize([1100]*4, .021)
                    initial = ct.c_int()
                    host._check(host.fmi2GetBoolean(host.component,host._references(["imuValid"]),1,ct.byref(initial)), "valid")
                    self.assertEqual(initial.value, 0)
                    # Step across sensor events, with a communication period not dividing output period.
                    dt = .00007
                    history = []
                    last_seq = 0
                    for i in range(round(.021/dt)):
                        host.step(i*dt, dt)
                        row = host.snapshot()
                        seq = ct.c_int()
                        host._check(host.fmi2GetInteger(host.component,host._references(["imuSequence"]),1,ct.byref(seq)), "sequence")
                        if seq.value != last_seq:
                            self.assertEqual(seq.value, last_seq+1)
                            self.assertAlmostEqual(row["imuSampleTime"], (seq.value-1)*.0025, places=9)
                            self.assertAlmostEqual(row["imuDeliveryTime"]-row["imuSampleTime"],.0025,places=9)
                            history.append(tuple(row[k] for k in ("imuSampleTime","imuDeliveryTime","accel[1]","gyro[1]")))
                            last_seq = seq.value
                    self.assertEqual(last_seq, 8)
                    results.append(history)
                finally:
                    host.close()
            self.assertEqual(results[0], results[1])
            metrics[f"I{level}"] = {"delivered_samples": len(results[0]), "instance_replay": "exact"}
        (work / "report.json").write_text(json.dumps(metrics, indent=2) + "\n")
