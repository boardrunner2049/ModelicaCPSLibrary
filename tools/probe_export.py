#!/usr/bin/env python3
"""Compose and step P0/P1 quad/hexa FMI 2.0 CS models with OpenModelica (Linux)."""
import argparse
import ctypes as ct
import json
import math
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET
import zipfile

ROOT = Path(__file__).resolve().parents[1]


def require(condition, message):
    if not condition:
        raise AssertionError(message)


class Fmi2CS:
    """Small FMI 2.0 Co-Simulation host; no fallback to native Modelica simulation."""

    def __init__(self, fmu: Path, directory: Path, n: int, n_rotors: int | None = None):
        self.inputs = [f"pwm[{i}]" for i in range(1, n+1)]
        self.observables = ([f"rotorSpeed[{i}]" for i in range(1, (n if n_rotors is None else n_rotors)+1)]
            + [f"{key}[{i}]" for key in ("gps", "vel_ned", "accel", "gyro", "mag", "truth.p_w", "truth.v_w") for i in range(1, 4)]
            + ["baro_altitude_m", "baro_climb_rate_mps", "baro_pressure_pa", "baro_temperature_c", "yaw_deg"])
        directory.mkdir(parents=True, exist_ok=True)
        with zipfile.ZipFile(fmu) as archive:
            for member in archive.infolist():
                require((directory / member.filename).resolve().is_relative_to(directory.resolve()),
                        "FMU contains a path outside the extraction directory")
            archive.extractall(directory)
        description = ET.parse(directory / "modelDescription.xml").getroot()
        require(description.attrib.get("fmiVersion") == "2.0", "Expected FMI 2.0")
        cs = description.find("CoSimulation")
        require(cs is not None, "FMU has no CoSimulation interface")
        self.identifier = cs.attrib["modelIdentifier"]
        self.variables = {
            variable.attrib["name"]: variable
            for variable in description.find("ModelVariables")
        }
        for name in self.inputs:
            require(name in self.variables and self.variables[name].attrib.get("causality") == "input",
                    f"Missing real PWM input {name}")
        for name in self.observables:
            require(name in self.variables and self.variables[name].find("Real") is not None,
                    f"Missing real observable {name}")
            if name.startswith("truth."):
                # OMC exports record fields as local diagnostics, not scalar output ports.
                # Read their own value references; do not advertise them as public sensor IO.
                require(self.variables[name].attrib.get("causality", "local") in {"local", "output"},
                        f"Unexpected truth diagnostic causality: {name}")
            else:
                require(self.variables[name].attrib.get("causality") == "output",
                        f"Missing exported output {name}")
        binary = directory / "binaries" / "linux64" / f"{self.identifier}.so"
        require(binary.is_file(), f"No Linux 64-bit FMI binary at {binary}")
        self.library = ct.CDLL(str(binary))
        self.log: list[str] = []
        logger_type = ct.CFUNCTYPE(None, ct.c_void_p, ct.c_char_p, ct.c_int, ct.c_char_p, ct.c_char_p)

        def log_message(_environment, _instance, status, category, message):
            self.log.append(f"{status}:{(category or b'').decode(errors='replace')}:"
                            f"{(message or b'').decode(errors='replace')}")

        self.logger = logger_type(log_message)
        self.libc = ct.CDLL(None)
        self.libc.calloc.argtypes = [ct.c_size_t, ct.c_size_t]
        self.libc.calloc.restype = ct.c_void_p
        self.libc.free.argtypes = [ct.c_void_p]
        self.libc.free.restype = None

        class Callbacks(ct.Structure):
            _fields_ = [(name, ct.c_void_p) for name in (
                "logger", "allocateMemory", "freeMemory", "stepFinished", "componentEnvironment")]

        self.callbacks = Callbacks(ct.cast(self.logger, ct.c_void_p),
                                   ct.cast(self.libc.calloc, ct.c_void_p),
                                   ct.cast(self.libc.free, ct.c_void_p), None, None)
        self._bind("fmi2Instantiate", [ct.c_char_p, ct.c_int, ct.c_char_p, ct.c_char_p,
                                      ct.POINTER(Callbacks), ct.c_int, ct.c_int], ct.c_void_p)
        self._bind("fmi2SetupExperiment", [ct.c_void_p, ct.c_int, ct.c_double,
                                          ct.c_double, ct.c_int, ct.c_double])
        for name in ("fmi2EnterInitializationMode", "fmi2ExitInitializationMode", "fmi2Terminate"):
            self._bind(name, [ct.c_void_p])
        self._bind("fmi2FreeInstance", [ct.c_void_p], None)
        for name in ("fmi2SetReal", "fmi2GetReal"):
            self._bind(name, [ct.c_void_p, ct.POINTER(ct.c_uint), ct.c_size_t, ct.POINTER(ct.c_double)])
        self._bind("fmi2DoStep", [ct.c_void_p, ct.c_double, ct.c_double, ct.c_int])
        self.component = self.fmi2Instantiate(
            b"fire_export_probe", 1, description.attrib["guid"].encode(),
            ((directory / "resources").resolve().as_uri() + "/").encode(),
            ct.byref(self.callbacks), 0, 0)
        require(bool(self.component), "fmi2Instantiate failed")
        self.held_inputs = None

    def _bind(self, name: str, arguments: list, result=ct.c_int) -> None:
        function = getattr(self.library, name, None)
        if function is None:
            function = getattr(self.library, self.identifier + "_" + name)
        function.argtypes, function.restype = arguments, result
        setattr(self, name, function)

    def _check(self, status: int, operation: str) -> None:
        require(status <= 1, f"{operation} returned FMI status {status}: {self.log[-5:]}")

    def _references(self, names: list[str]):
        return (ct.c_uint * len(names))(*(int(self.variables[name].attrib["valueReference"]) for name in names))

    def set_inputs(self, values: list[float]) -> None:
        self._check(self.fmi2SetReal(self.component, self._references(self.inputs), len(self.inputs),
                                   (ct.c_double * len(values))(*values)), "fmi2SetReal")
        self.held_inputs = list(values)

    def initialize(self, values: list[float], stop: float) -> None:
        self._check(self.fmi2SetupExperiment(self.component, 1, 1e-8, 0, 1, stop), "setup")
        self.set_inputs(values)
        self._check(self.fmi2EnterInitializationMode(self.component), "enter initialization")
        self._check(self.fmi2ExitInitializationMode(self.component), "exit initialization")

    def snapshot(self) -> dict[str, float]:
        # OpenModelica 1.26.1 internalGetEventIndicators clears _need_update
        # after evaluating only the ODE. With contact events, algebraic sensor
        # outputs can therefore lag the states by one CS step. Reapply the
        # unchanged held inputs at this communication point to invalidate the
        # runtime cache before fmi2GetReal; no input value or state is changed.
        # A production host must handle this too, or use a corrected runtime.
        if self.held_inputs is not None:
            self.set_inputs(self.held_inputs)
        values = (ct.c_double * len(self.observables))()
        self._check(self.fmi2GetReal(self.component, self._references(self.observables), len(values), values), "get real")
        require(all(math.isfinite(value) for value in values), "Nonfinite exported real value")
        result = dict(zip(self.observables, values))
        return result

    def step(self, time: float, step: float) -> None:
        self._check(self.fmi2DoStep(self.component, time, step, 1), f"fmi2DoStep at t={time}, dt={step}")

    def close(self) -> None:
        if self.component:
            self.fmi2Terminate(self.component)
            self.fmi2FreeInstance(self.component)
            self.component = None

def run(output, omc):
    output.mkdir(parents=True, exist_ok=True)
    (output / "report.json").unlink(missing_ok=True)
    metrics = {}
    subprocess.run(['cargo', 'build', '--locked', '--offline', '--manifest-path',
                    str(ROOT / 'tools/fire-compose/Cargo.toml')], check=True)
    composer = ROOT / 'tools/fire-compose/target/debug/fire-compose'
    cases = [(preset, n, propulsion) for propulsion in ('P0', 'P1')
             for preset, n in [('QuadX', 4), ('HexaX', 6)]]
    for preset, n, propulsion in cases:
        case = f'{propulsion}-{preset}'
        work = output / case
        work.mkdir(exist_ok=True)
        config = work / 'vehicle.toml'
        raw = ('schema_version=3\nmodel_name="ExportCopter"\n'
               f'[geometry]\npreset="{preset}"\n'
               '[models]\nrotor="quadratic_blade"\n')
        if propulsion == 'P1':
            raw += 'esc="average_voltage"\nmotor="first_order_voltage_to_speed"\n[motor]\nKv=62.5\n[power]\ndcVoltage=16\n'
        config.write_text(raw)
        generated = work / 'generated'
        composed = subprocess.run([str(composer), 'emit', str(config), '--profile', 'vio',
                                   '--output-dir', str(generated)], capture_output=True, text=True)
        (work / 'compose.log').write_text(composed.stdout + composed.stderr)
        require(composed.returncode == 0, f'Composition failed: {work / "compose.log"}')
        manifest = json.loads((generated / 'ExportCopter.manifest.json').read_text())
        require(manifest['profile'] == 'vio' and manifest['physics']['propulsion'] == propulsion,
                'Wrong composer profile')
        pwm_min = manifest['input_adapter']['pwm_min_us']
        pwm_span = manifest['input_adapter']['pwm_max_us'] - pwm_min
        harness = generated / 'ExportCopter.mo'
        script = work / "export.mos"
        script.write_text('loadModel(Modelica,{"4.0.0"});\n'
            f'loadFile({json.dumps(str(generated / "sources/ModelicaCPSLibrary/package.mo"))});\n'
            f'loadFile({json.dumps(str(harness))});\n'
            'buildModelFMU(ExportCopter,version="2.0",fmuType="cs",fileNamePrefix="ExportCopter",platforms={"static"});\ngetErrorString();\n')
        fmu = work / "ExportCopter.fmu"
        fmu.unlink(missing_ok=True)  # Only this tool's generated artifact; reject stale exports.
        proc = subprocess.run([omc,str(script)],cwd=work,capture_output=True,text=True,timeout=300)
        log = proc.stdout + proc.stderr
        (work / "build.log").write_text(log)
        require(proc.returncode == 0 and "Error:" not in log and fmu.is_file(), f"Export failed: {work / 'build.log'}")
        host = Fmi2CS(fmu, work / "unpacked", n)
        dt = 0.0001
        try:
            host.initialize([pwm_min]*n, 0.061)  # Leave margin for binary roundoff at the last doStep.
            for tick in range(200):
                host.step(tick*dt,dt)
            state = host.snapshot()
            require(abs(state['truth.p_w[3]']-(-1+0.5*9.80665*0.02**2))<2e-5, 'Freefall must be Down-positive')
            require(abs(state['accel[3]'])<1e-8, 'Freefall specific force must be zero')
            demand = [(i+1)/(2*n) for i in range(n)]
            host.set_inputs([pwm_min+pwm_span*u for u in demand])
            for tick in range(200,600):
                host.step(tick*dt,dt)
            state = host.snapshot()
            errors = [abs(state[f'rotorSpeed[{i+1}]']-1000*u*(1-math.exp(-0.04/0.03))) for i,u in enumerate(demand)]
            require(max(errors)<1.0, f'Independent {propulsion} motor response failed: {errors}')
            for i in range(1,4):
                require(abs(state[f'vel_ned[{i}]']-state[f'truth.v_w[{i}]'])<1e-8, 'Velocity converted twice')
            expected_gps = [40.414929 + state['truth.p_w[1]']/6378137*180/math.pi,
                            -86.932387 + state['truth.p_w[2]']/(6378137*math.cos(math.radians(40.414929)))*180/math.pi,
                            149-state['truth.p_w[3]']]
            for i, expected in enumerate(expected_gps, 1):
                require(abs(state[f'gps[{i}]']-expected)<1e-8, 'Geodetic GNSS conversion changed')
            require(abs(state['baro_pressure_pa']-101325)<1e-8, 'Plant pressure supply changed')
            require(abs(state['baro_temperature_c']-15)<1e-8, 'Plant temperature supply or Celsius conversion changed')
            require(abs(state['baro_altitude_m']+state['truth.p_w[3]'])<1e-8, 'Barometer altitude must be Up-positive')
            require(abs(state['baro_climb_rate_mps']+state['vel_ned[3]'])<1e-8, 'Climb rate must be Up-positive')
            require(state['accel[3]']<0, 'Thrust must give negative body Down specific force')
            metrics[case] = {"channels":n,"max_speed_error_rad_s":max(errors),"step_s":dt}
        finally:
            host.close()
        # A fresh instance exercises touchdown/liftoff events inside the exported FMU.
        # Never overwrite a shared library that is still mapped by ctypes.
        host = Fmi2CS(fmu, work / "unpacked-ground", n)
        try:
            host.initialize([pwm_min]*n, 5.501)
            checkpoints = {}
            takeoff_pwm = pwm_min+pwm_span*math.sqrt(1.3*1.5*9.80665/(n*1e-5))/1000
            for tick in range(55000):
                if tick == 20000:
                    host.set_inputs([takeoff_pwm]*n)
                elif tick == 27000:
                    host.set_inputs([pwm_min]*n)
                host.step(tick*dt,dt)
                if tick+1 in (15000,25000,55000):
                    checkpoints[(tick+1)*dt] = host.snapshot()
            for time in (1.5,5.5):
                state = checkpoints[time]
                require(abs(state['truth.p_w[3]']-(-0.1+1.5*9.80665/12000))<1e-5,
                        f'Four-point FMU support/compression failed at {time}')
                require(abs(state['truth.v_w[3]'])<1e-5, f'FMU did not settle at {time}')
                require(abs(state['accel[3]']+9.80665)<1e-3,
                        f'Grounded FMU accelerometer must report -g at {time}')
            airborne = checkpoints[2.5]
            require(airborne['truth.p_w[3]'] < -0.15 and airborne['truth.v_w[3]'] < 0,
                    'FMU must release contact and rise during thrust')
            metrics[case]['landing_gear'] = {
                'settled_altitude_m': -checkpoints[1.5]['truth.p_w[3]'],
                'airborne_altitude_m': -airborne['truth.p_w[3]'],
                'landed_altitude_m': -checkpoints[5.5]['truth.p_w[3]'],
                'supported_accel_z_mps2': checkpoints[1.5]['accel[3]'],
            }
        finally:
            host.close()
    report = {"compiler":subprocess.check_output([omc,"--version"],text=True).strip(),
              "target":"OpenModelica FMI 2.0 Co-Simulation", "core_contract":"ned_frd_continuous_v1",
              "output_refresh":"Reapply held inputs before getReal (OpenModelica 1.26.1 contact-event cache workaround)",
              "truth_access":"Record truth fields read as OMC diagnostic variables; public sensor outputs checked separately",
              "metrics":metrics}
    (output / "report.json").write_text(json.dumps(report,indent=2)+"\n")
    print("PASS: composer P0/P1 quad/hexa FMUs, motor/sensor outputs, settling, takeoff and landing")
    print(output / "report.json")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--omc",default="omc")
    parser.add_argument("--output",type=Path,default=ROOT / "build/export-validation")
    args = parser.parse_args()
    run(args.output.resolve(),args.omc)
