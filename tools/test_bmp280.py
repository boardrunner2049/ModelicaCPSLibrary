"""BMP280 FMI2 sensor bench through unchanged ArduPilot driver/HAL/backend code.

Run with FIRE_TEST_BMP280=1. This sensor-only bench owns its clock; each bus
transaction takes an explicit synthetic 10 us. It is not a SITL bus binding.
"""
import ctypes as ct
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import unittest
import xml.etree.ElementTree as ET
import zipfile

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "build/bmp280-implementation/integration"
OMC_HOME = Path("C:/Program Files/OpenModelica1.27.0-64bit")


def tool(variable, name, fallback):
    return os.environ.get(variable) or shutil.which(name) or str(fallback)


def require(condition, message):
    if not condition:
        raise AssertionError(message)


class SensorFMU:
    """Only the scalar sensor FMI2-CS ports; no cached receiver state."""
    inputs = ["commandSequence", "commandRegister", "commandValue",
              "ambientPressure", "ambientTemperature"]
    outputs = (["processedCommandSequence", "commandStatus", "ctrlMeas", "config",
                "chipId", "rawPressure", "rawTemperature", "converting", "sequence",
                "sampleTime", "deliveryTime", "filterCoefficient", "pressure", "temperature", "nextTransition"]
               + [f"calibration[{i}]" for i in range(1, 13)])

    def __init__(self, fmu, work, max_step=0.001):
        work.mkdir(parents=True, exist_ok=True)
        with zipfile.ZipFile(fmu) as archive:
            for member in archive.infolist():
                require((work / member.filename).resolve().is_relative_to(work.resolve()),
                        "FMU member escapes extraction directory")
            archive.extractall(work)
        description = ET.parse(work / "modelDescription.xml").getroot()
        require(description.attrib["fmiVersion"] == "2.0", "Expected FMI2")
        self.identifier = description.find("CoSimulation").attrib["modelIdentifier"]
        exported = {v.attrib["name"]: v for v in description.find("ModelVariables")}
        # OpenModelica cannot promote this when-algorithm's discrete outputs.
        # Read the child outputs as FMI locals in this bench, as probe_export
        # does for truth diagnostics; this is not a public vehicle-runtime ABI.
        self.variables = {name: exported[name if name in self.inputs + ["pressure", "temperature"]
                                         else "sensor.vendorConfig." + name]
                          for name in self.inputs + self.outputs}
        for name in self.inputs + self.outputs:
            require(self.variables[name].find("Integer" if name == "sequence" else "Real") is not None,
                    f"Wrong FMI scalar type: {name}")
            expected = "input" if name in self.inputs else "output" if name in ("pressure", "temperature") else "local"
            require(self.variables[name].attrib.get("causality", "local") == expected, f"Wrong causality: {name}")
        platform, extension = ("win64", "dll") if os.name == "nt" else ("linux64", "so")
        self.library = ct.CDLL(str(work / "binaries" / platform / f"{self.identifier}.{extension}"))
        self.log = []
        logger_type = ct.CFUNCTYPE(None, ct.c_void_p, ct.c_char_p, ct.c_int, ct.c_char_p, ct.c_char_p)
        self.logger = logger_type(lambda _e, _i, s, _c, m: self.log.append((s, (m or b"").decode(errors="replace"))))
        self.allocator = ct.CDLL("msvcrt" if os.name == "nt" else None)
        self.allocator.calloc.argtypes, self.allocator.calloc.restype = [ct.c_size_t, ct.c_size_t], ct.c_void_p
        self.allocator.free.argtypes, self.allocator.free.restype = [ct.c_void_p], None

        class Callbacks(ct.Structure):
            _fields_ = [(name, ct.c_void_p) for name in
                        ("logger", "allocateMemory", "freeMemory", "stepFinished", "componentEnvironment")]

        self.callbacks = Callbacks(ct.cast(self.logger, ct.c_void_p),
                                   ct.cast(self.allocator.calloc, ct.c_void_p),
                                   ct.cast(self.allocator.free, ct.c_void_p), None, None)
        self.bind("fmi2Instantiate", [ct.c_char_p, ct.c_int, ct.c_char_p, ct.c_char_p,
                                     ct.POINTER(Callbacks), ct.c_int, ct.c_int], ct.c_void_p)
        self.bind("fmi2SetupExperiment", [ct.c_void_p, ct.c_int, ct.c_double, ct.c_double, ct.c_int, ct.c_double])
        for name in ("fmi2EnterInitializationMode", "fmi2ExitInitializationMode", "fmi2Terminate"):
            self.bind(name, [ct.c_void_p])
        self.bind("fmi2FreeInstance", [ct.c_void_p], None)
        for name in ("fmi2SetReal", "fmi2GetReal"):
            self.bind(name, [ct.c_void_p, ct.POINTER(ct.c_uint), ct.c_size_t, ct.POINTER(ct.c_double)])
        self.bind("fmi2GetInteger", [ct.c_void_p, ct.POINTER(ct.c_uint), ct.c_size_t, ct.POINTER(ct.c_int)])
        self.bind("fmi2DoStep", [ct.c_void_p, ct.c_double, ct.c_double, ct.c_int])
        self.component = self.fmi2Instantiate(b"bmp280_bench", 1, description.attrib["guid"].encode(),
                                            ((work / "resources").resolve().as_uri() + "/").encode(),
                                            ct.byref(self.callbacks), 0, 0)
        require(bool(self.component), "FMI instance creation failed")
        self.time = 0.0
        self.max_step = max_step
        self.values = [0.0, 0.0, 0.0, 100000.0, 298.15]
        self.check(self.fmi2SetupExperiment(self.component, 1, 1e-9, 0, 1, 10), "setup")
        self.set_inputs()
        self.check(self.fmi2EnterInitializationMode(self.component), "enter initialization")
        self.check(self.fmi2ExitInitializationMode(self.component), "exit initialization")

    def bind(self, name, arguments, result=ct.c_int):
        function = getattr(self.library, name, None) or getattr(self.library, self.identifier + "_" + name)
        function.argtypes, function.restype = arguments, result
        setattr(self, name, function)

    def check(self, status, operation):
        require(status <= 1, f"FMI {operation} failed ({status}): {self.log[-5:]}")

    def references(self, names):
        return (ct.c_uint * len(names))(*(int(self.variables[name].attrib["valueReference"]) for name in names))

    def set_inputs(self):
        self.check(self.fmi2SetReal(self.component, self.references(self.inputs), len(self.inputs),
                                   (ct.c_double * len(self.values))(*self.values)), "set inputs")

    def snapshot(self):
        # Reapply held values to invalidate the OpenModelica algebraic-output cache.
        self.set_inputs()
        names = [name for name in self.outputs if name != "sequence"]
        values = (ct.c_double * len(names))()
        self.check(self.fmi2GetReal(self.component, self.references(names), len(values), values), "read outputs")
        require(all(math.isfinite(x) for x in values), "Nonfinite sensor output")
        sequence = ct.c_int()
        self.check(self.fmi2GetInteger(self.component, self.references(["sequence"]), 1, ct.byref(sequence)), "read sequence")
        return dict(zip(names, values), sequence=sequence.value)

    def advance(self, target, ramp=None):
        # This exporter evaluates discrete events at CS-step starts. Its existing
        # internal-variable export lets the bench respect Modelica's own deadline.
        # A 1 ps positive service step processes an event at that deadline; an
        # exact-deadline observation may consequently advance by at most 1 ps.
        epsilon = 1e-12
        require(target >= self.time - 2 * epsilon, "Bench clock moved backwards")
        while True:
            deadline = self.snapshot()["nextTransition"]
            if deadline <= self.time + 1e-14:
                step = epsilon
            elif target - self.time > 1e-14:
                step = min(self.max_step, target - self.time, deadline - self.time)
            else:
                break
            # A pressure-chamber ramp at constant height/temperature, not a flight claim.
            if ramp is not None:
                self.values[3] = 100000 + (20 * self.time if ramp else 0)
            self.set_inputs()
            self.check(self.fmi2DoStep(self.component, self.time, step, 1), "advance")
            self.time += step

    def write(self, register, value, target, ramp=None):
        # FMI observes this input event within the declared 10 us transaction;
        # its exact internal event instant is not asserted to be bus completion.
        self.advance(max(self.time, target - 1e-5), ramp)
        self.values[:3] = [self.values[0] + 1, register, value]
        self.set_inputs()
        self.advance(target, ramp)
        state = self.snapshot()
        require(state["processedCommandSequence"] == self.values[0], "Bus write was not processed in order")
        require(state["commandStatus"] in (1, 2), f"Emulator coverage gap, not device NAK: {state}")
        return state

    def close(self):
        if self.component:
            self.fmi2Terminate(self.component)
            self.fmi2FreeInstance(self.component)
            self.component = None


NATIVE_FILES = {
    "AP_Baro/AP_Baro_BMP280.cpp": "AP_Baro_BMP280.cpp",
    "AP_Baro/AP_Baro_BMP280.h": "AP_Baro_BMP280.h",
    "AP_Baro/AP_Baro_Backend.cpp": "AP_Baro_Backend.cpp",
    "AP_Baro/AP_Baro_Backend.h": "AP_Baro_Backend.h",
    "AP_HAL/Device.cpp": "AP_HAL/Device.cpp", "AP_HAL/Device.h": "AP_HAL/Device.h",
    "AP_HAL/utility/functor.h": "AP_HAL/utility/functor.h",
    "AP_HAL/utility/OwnPtr.h": "AP_HAL/utility/OwnPtr.h",
}
STUBS = {
    "AP_HAL/AP_HAL_Namespace.h": """#pragma once
namespace AP_HAL { class Device; class Semaphore; class Dummy; using Proc=void(*)(); using MemberProc=void(Dummy::*)(); }
""",
    "AP_HAL/AP_HAL_Boards.h": "#pragma once\n#define CONFIG_HAL_BOARD 3\n#define HAL_BOARD_QURT 100\n",
    "AP_Common/AP_Common.h": """#pragma once
#include <new>
#include <cstddef>
#include <cstdint>
#define NEW_NOTHROW new(std::nothrow)
#define WITH_SEMAPHORE(x) do { (void)(x); } while(0)
""",
    "AP_HAL/AP_HAL.h": """#pragma once
#include <AP_Common/AP_Common.h>
#include <AP_HAL/utility/functor.h>
namespace AP_HAL { class Semaphore {}; struct HAL {}; uint32_t millis(); }
using HAL_Semaphore=AP_HAL::Semaphore;
""",
    "AP_Math/definitions.h": "#pragma once\n#define AP_USEC_PER_MSEC 1000U\n",
    "AP_Math/AP_Math.h": """#pragma once
#include <cmath>
#include <limits>
using std::isinf; using std::isnan;
template<class A,class B> bool is_equal(A a,B b){return std::fabs(float(a-b))<std::numeric_limits<float>::epsilon();}
template<class A> bool is_zero(A a){return std::fabs(float(a))<std::numeric_limits<float>::epsilon();}
""",
    "AP_Baro.h": """#pragma once
#include <AP_HAL/AP_HAL.h>
#define AP_BARO_BMP280_ENABLED 1
#define AP_BARO_MSP_ENABLED 0
#define AP_BARO_EXTERNALAHRS_ENABLED 0
#define BARO_TIMEOUT_MS 500
#define BARO_DATA_CHANGE_TIMEOUT_MS 2000
struct Param {int32_t value=0;void set(int32_t x){value=x;}};
class AP_Baro {public:
 struct Reading {float pressure=0,temperature=0;uint32_t last_change_ms=0,last_update_ms=0;bool healthy=false;Param bus_id;} sensors[1];
 uint8_t _num_sensors=0;uint8_t register_sensor(){return _num_sensors++;}
 float get_filter_range()const{return 0.0f;}
};
""",
}
NATIVE_MAIN = r'''#include "AP_Baro_BMP280.h"
#include <AP_HAL/Device.h>
#include <cassert>
#include <cstdio>
#include <cstdlib>
#include <cstring>

// ArduPilot's AP_Common/c++.cpp uses calloc-backed new.
void* operator new(size_t n){auto p=std::calloc(n?n:1,1);if(!p)throw std::bad_alloc();return p;}
void* operator new[](size_t n){return ::operator new(n);}
void* operator new(size_t n,const std::nothrow_t&)noexcept{return std::calloc(n?n:1,1);}
void* operator new[](size_t n,const std::nothrow_t&)noexcept{return std::calloc(n?n:1,1);}
void operator delete(void*p)noexcept{std::free(p);}void operator delete[](void*p)noexcept{std::free(p);}
void operator delete(void*p,size_t)noexcept{std::free(p);}void operator delete[](void*p,size_t)noexcept{std::free(p);}
static uint64_t now_us=0;
namespace AP_HAL {uint32_t millis(){return now_us/1000;}}
const AP_HAL::HAL native_hal{};const AP_HAL::HAL&hal=native_hal;
class SensorDevice final:public AP_HAL::Device {
public:
 PeriodicCb cb;uint32_t period=0;AP_HAL::Semaphore sem;
 SensorDevice():Device(BUS_TYPE_I2C){}
 bool set_speed(Speed)override{return true;}
 AP_HAL::Semaphore*get_semaphore()override{return &sem;}
 PeriodicHandle register_periodic_callback(uint32_t us,PeriodicCb f)override{period=us;cb=f;return this;}
 bool adjust_periodic_callback(PeriodicHandle,uint32_t)override{return false;}
 bool transfer(const uint8_t*s,uint32_t n,uint8_t*r,uint32_t z)override{
  assert(s && n>=1);now_us+=10;
  if(z){assert(n==1);std::printf("R %llu %u %u\n",(unsigned long long)now_us,s[0],z);std::fflush(stdout);
   for(uint32_t i=0;i<z;++i){unsigned x;assert(std::scanf("%u",&x)==1 && x<256);r[i]=x;}return true;}
  assert(n==2);std::printf("W %llu %u %u\n",(unsigned long long)now_us,s[0],s[1]);std::fflush(stdout);
  unsigned processed;assert(std::scanf("%u",&processed)==1 && processed==1);return true;
 }
 void tick(uint64_t t,AP_Baro_Backend*driver){now_us=t;cb();driver->backend_update(0);}
};
int main(int argc,char**argv){
 assert(argc==3);now_us=std::strtoull(argv[1],nullptr,10);const bool repair=std::strcmp(argv[2],"repair")==0;
 AP_Baro b;auto*d=NEW_NOTHROW SensorDevice;AP_HAL::OwnPtr<AP_HAL::Device> owned(d);
 auto*driver=AP_Baro_BMP280::probe(b,std::move(owned));assert(driver && d->period==20000);
 uint64_t base=now_us;
 for(unsigned i=1;i<=160;++i){
  d->tick(base+i*20000,driver);
  std::printf("S %llu %.9g %.9g %u %u\n",(unsigned long long)now_us,b.sensors[0].pressure,b.sensors[0].temperature,b.sensors[0].healthy,b.sensors[0].last_change_ms);std::fflush(stdout);
  if(repair && i==60){
   std::printf("IDLE %llu\n",(unsigned long long)now_us);std::fflush(stdout);
   unsigned long long idle;assert(std::scanf("%llu",&idle)==1);now_us=idle;
   d->write_register(0xF4,0x54);d->write_register(0xF5,0x08);d->write_register(0xF4,0x57);
   base=now_us-i*20000;
  }
 }
 std::puts("DONE");std::fflush(stdout);delete driver;
}
'''


def prepare_native():
    native = OUT / "native"
    native.mkdir(parents=True, exist_ok=True)
    manifest = {}
    for source, destination in NATIVE_FILES.items():
        original = ROOT / "third_party/ardupilot/libraries" / source
        target = native / destination
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(original, target)
        manifest[source] = hashlib.sha256(target.read_bytes()).hexdigest()
        require(manifest[source] == hashlib.sha256(original.read_bytes()).hexdigest(), "Native source copy changed")
    for name, content in STUBS.items():
        target = native / name
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text(content, encoding="utf-8")
    (native / "bench.cpp").write_text(NATIVE_MAIN, encoding="utf-8")
    compiler = tool("CXX", "g++", OMC_HOME / "tools/msys/ucrt64/bin/g++.exe")
    environment = dict(os.environ, PATH=str(Path(compiler).resolve().parent) + os.pathsep + os.environ["PATH"])
    executable = native / ("native-bmp280.exe" if os.name == "nt" else "native-bmp280")
    run = subprocess.run([compiler, "-std=c++17", "-O2", "-I.", "bench.cpp", "AP_Baro_BMP280.cpp",
                          "AP_Baro_Backend.cpp", "AP_HAL/Device.cpp", "-o", str(executable)],
                         cwd=native, env=environment, capture_output=True, text=True, timeout=90)
    (native / "compile.log").write_text(run.stdout + run.stderr)
    require(run.returncode == 0, f"Native compile failed: {native / 'compile.log'}")
    (native / "manifest.json").write_text(json.dumps({"native_sha256": manifest, "stubs": list(STUBS),
        "controller_commit": subprocess.check_output(["git", "-C", str(ROOT / "third_party/ardupilot"),
                                                       "rev-parse", "HEAD"], text=True).strip(),
        "frontend_filter_range": 0, "transaction_duration_s": 1e-5,
        "boundary": "FMI2 sensor + native BMP280/HAL checked registers/backend; not SITL or flight"}, indent=2))
    return executable, environment


def export_models():
    def model_hashes():
        return {str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest()
                for path in sorted((ROOT / "Systems/Sensing/Barometer").rglob("*.mo"))}

    source_hashes = model_hashes()
    script = ['setCommandLineOptions("--fmiFilter=internal");', 'loadModel(Modelica,{"4.0.0"});',
              f'loadFile({json.dumps(str(ROOT / "package.mo"))});', 'getErrorString();']
    for name, accepted in (("BMP280Accepted", "true"), ("BMP280Ignored", "false")):
        model = f'''model {name}
 input Real commandSequence=0,commandRegister=0,commandValue=0;
 input Real ambientPressure(unit="Pa")=100000,ambientTemperature(unit="K")=298.15;
 output Real pressure,temperature;
 ModelicaCPSLibrary.Systems.Sensing.Barometer.Device sensor(
  redeclare model VendorConfigModel=ModelicaCPSLibrary.Systems.Sensing.Barometer.VendorConfig.BMP280,
  vendorConfig(normalModeConfigWritesAccepted={accepted}));
equation
 sensor.ambientPressure=ambientPressure;sensor.ambientTemperature=ambientTemperature;
 // A stationary pressure chamber supplies independent ambient pressure.
 sensor.altitude_w=0;sensor.climbRate_w=0;
 sensor.vendorConfig.commandSequence=commandSequence;
 sensor.vendorConfig.commandRegister=commandRegister;
 sensor.vendorConfig.commandValue=commandValue;
 pressure=sensor.pressure;temperature=sensor.temperature;
end {name};'''
        (OUT / f"{name}.mo").write_text(model)
        script += [f'loadFile({json.dumps(str(OUT / (name + ".mo")))});', 'getErrorString();',
                   f'buildModelFMU({name},version="2.0",fmuType="cs",fileNamePrefix="{name}",platforms={{"static"}});', 'getErrorString();']
        (OUT / f"{name}.fmu").unlink(missing_ok=True)
    (OUT / "export.mos").write_text("\n".join(script))
    run = subprocess.run([tool("OMC", "omc", OMC_HOME / "bin/omc.exe"), "export.mos"],
                         cwd=OUT, capture_output=True, text=True, timeout=300)
    log = run.stdout + run.stderr
    (OUT / "export.log").write_text(log)
    require(run.returncode == 0 and all((OUT / f"{name}.fmu").is_file()
            for name in ("BMP280Accepted", "BMP280Ignored")), f"FMU export failed: {OUT / 'export.log'}")
    require("Error:" not in log and "Warning:" not in log, f"Modelica diagnostic: {OUT / 'export.log'}")
    require(source_hashes == model_hashes(), "Modelica source changed during FMU compilation; rebuild required")
    (OUT / "model-manifest.json").write_text(json.dumps({
        "fmu_sha256": {name: hashlib.sha256((OUT / f"{name}.fmu").read_bytes()).hexdigest()
                       for name in ("BMP280Accepted", "BMP280Ignored")},
        "modelica_sha256": source_hashes,
        "export_option": "--fmiFilter=internal",
        "clock": "Modelica nextTransition schedules bench CS steps; 1 ps positive event service",
    }, indent=2))


def native_case(name, executable, environment, accepted, retained, ramp, repair=False):
    work = OUT / name
    work.mkdir(parents=True, exist_ok=True)
    (work / "result.json").unlink(missing_ok=True)
    sensor = SensorFMU(OUT / ("BMP280Accepted.fmu" if accepted else "BMP280Ignored.fmu"), work / "fmu")
    transactions, samples = [], []
    process = None
    error_stream = None
    try:
        if retained:
            sensor.write(0xF5, 0x10, sensor.time + 1e-5)
            sensor.write(0xF4, 0x57, sensor.time + 1e-5)
            sensor.advance(sensor.time + 0.114, ramp)
            require(sensor.snapshot()["filterCoefficient"] == 16, "Retained setup did not configure filter16")
        start = round(sensor.time * 1e6)
        error_stream = (work / "native.stderr").open("w")
        process = subprocess.Popen([str(executable), str(start), "repair" if repair else "ordinary"],
                                   cwd=work, env=environment, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                   stderr=error_stream, text=True, bufsize=1)
        for line in process.stdout:
            fields = line.split()
            if fields[0] in ("R", "W"):
                operation, stamp, register, count_or_value = fields
                target, register, count_or_value = int(stamp) / 1e6, int(register), int(count_or_value)
                if operation == "W":
                    state = sensor.write(register, count_or_value, target, ramp)
                    response = "1"
                else:
                    sensor.advance(target, ramp)
                    state = sensor.snapshot()
                    if register == 0xD0 and count_or_value == 1:
                        data = [round(state["chipId"])]
                    elif register == 0x88 and count_or_value == 24:
                        data = []
                        for i in range(1, 13):
                            coefficient = round(state[f"calibration[{i}]"]) & 65535
                            data += [coefficient & 255, coefficient >> 8]
                    elif register in (0xF4, 0xF5) and count_or_value == 1:
                        data = [round(state["ctrlMeas" if register == 0xF4 else "config"])]
                    elif register == 0xF7 and count_or_value == 6:
                        data = []
                        for key in ("rawPressure", "rawTemperature"):
                            value = round(state[key]) << 4
                            data += [(value >> shift) & 255 for shift in (16, 8, 0)]
                    else:
                        raise AssertionError(f"Unhandled valid native request, emulator gap: {fields}")
                    response = " ".join(map(str, data))
                transactions.append({"operation": operation, "time": target, "register": register,
                                     "begin_time": target - 1e-5,
                                     "value_or_count": count_or_value, "state": state, "response": response})
                process.stdin.write(response + "\n")
                process.stdin.flush()
            elif fields[0] == "S":
                samples.append(dict(zip(("time_us", "pressure_pa", "temperature_c", "healthy", "last_change_ms"),
                                        map(float, fields[1:]))))
            elif fields[0] == "IDLE":
                # Explicit bench scheduling, not a claimed firmware-visible status register.
                deadline = sensor.time + 0.038
                while sensor.snapshot()["converting"]:
                    require(sensor.time < deadline, "Conversion did not finish within the modeled cycle")
                    sensor.advance(sensor.time + 1e-5, ramp)
                process.stdin.write(str(math.ceil(sensor.time * 1e6)) + "\n")
                process.stdin.flush()
            elif fields[0] == "DONE":
                break
            else:
                raise AssertionError(f"Unexpected native bridge output: {line}")
        require(process.wait(timeout=10) == 0, f"Native driver failed: {work / 'native.stderr'}")
        require(len(samples) == 160, "Incomplete native driver execution")
        result = {"transactions": transactions, "samples": samples, "final": sensor.snapshot()}
        (work / "result.json").write_text(json.dumps(result, indent=2))
        return result
    except Exception as error:
        (work / "failure.json").write_text(json.dumps({"error": repr(error),
            "transactions": transactions, "samples": samples}, indent=2))
        raise
    finally:
        if process is not None and process.poll() is None:
            process.kill()
            process.wait(timeout=5)
        if process is not None:
            process.stdin.close()
            process.stdout.close()
        if error_stream is not None:
            error_stream.close()
        sensor.close()


@unittest.skipUnless(os.environ.get("FIRE_TEST_BMP280") == "1", "BMP280 native/FMI bench opt-in")
class BMP280NativeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        OUT.mkdir(parents=True, exist_ok=True)
        cls.executable, cls.environment = prepare_native()
        export_models()
        cls.results = {name: native_case(name, cls.executable, cls.environment, *settings)
                       for name, settings in {
                           "accepted_retained": (True, True, True),
                           "ignored_retained": (False, True, True),
                           "ignored_cold": (False, False, True),
                           "sleep_repair": (False, True, True, True),
                           "constant_protected": (True, True, False),
                       }.items()}

    def test_modelica_raw_filter_and_conversion_clock(self):
        # Closed form, independent of the model's recurrence. These raw counts
        # map inside the disclosed synthetic-trim operating envelope.
        def pressure_for_raw(raw):
            base = (1048576 - raw) * 6250 / 16384
            return base + base * base / 2097152

        cases = [(c, config, n75, step) for c, config, n75 in ((4, 8, 5), (16, 16, 22))
                 for step in (0.001, 0.0007, 0.00013)]
        for coefficient, config, n75, step in cases:
            with self.subTest(coefficient=coefficient, max_step=step):
                work = OUT / f"raw-filter-{coefficient}-{step:g}"
                sensor = SensorFMU(OUT / "BMP280Accepted.fmu", work / "fmu", max_step=step)
                (work / "result.json").unlink(missing_ok=True)
                try:
                    initial = sensor.snapshot()
                    self.assertEqual([initial[key] for key in ("ctrlMeas", "config", "sequence")], [0, 0, 0])
                    self.assertEqual(initial["rawPressure"], 0x80000)
                    self.assertEqual(initial["rawTemperature"], 0x80000)
                    sensor.values[3] = pressure_for_raw(850000)
                    sensor.write(0xF5, config, 1e-5)
                    began = sensor.time
                    sensor.write(0xF4, 0x57, sensor.time + 1e-5)
                    sensor.advance(began + 0.0375 - 1e-6)
                    self.assertEqual(sensor.snapshot()["sequence"], 0)
                    sensor.advance(began + 0.0375 + 1.1e-5)
                    first = sensor.snapshot()
                    self.assertEqual(first["sequence"], 1)
                    self.assertAlmostEqual(first["rawPressure"], 850000, delta=0.5)
                    self.assertAlmostEqual(first["deliveryTime"] - first["sampleTime"], 0.0375, delta=1e-8)
                    self.assertGreaterEqual(first["sampleTime"], began - 1e-9)
                    self.assertLessEqual(first["sampleTime"], began + 1e-5 + 1e-9)
                    sensor.values[3] = pressure_for_raw(800000)
                    sensor.set_inputs()
                    previous = first
                    for n in range(1, n75 + 1):
                        deadline = first["deliveryTime"] + n * 0.038
                        sensor.advance(deadline - 1e-6)
                        self.assertEqual(sensor.snapshot()["sequence"], n)
                        sensor.advance(deadline + 1e-6)
                        state = sensor.snapshot()
                        expected = 800000 + 50000 * (1 - 1 / coefficient) ** n
                        self.assertEqual(state["sequence"], n + 1)
                        self.assertAlmostEqual(state["rawPressure"], expected, delta=0.5 + 1e-6)
                        self.assertAlmostEqual(state["sampleTime"] - previous["sampleTime"], 0.038, delta=1e-8)
                        self.assertAlmostEqual(state["deliveryTime"] - state["sampleTime"], 0.0375, delta=1e-8)
                        self.assertEqual(sensor.snapshot(), state, "Reading a register must not acquire a new sample")
                        previous = state
                    self.assertGreaterEqual(1 - (1 - 1 / coefficient) ** n75, 0.75)
                    self.assertLess(1 - (1 - 1 / coefficient) ** (n75 - 1), 0.75)
                    # An accepted identical CONFIG write also resets filter memory.
                    self.assertGreater(state["rawPressure"], 800001)
                    sensor.write(0xF5, config, sensor.time + 1e-5)
                    self.assertEqual(sensor.snapshot()["commandStatus"], 1)
                    sensor.advance(state["deliveryTime"] + 0.038 + 1e-6)
                    self.assertEqual(sensor.snapshot()["rawPressure"], 800000)
                    (work / "result.json").write_text(json.dumps(
                        {"first": first, "before_identical_write": state, "after_identical_write": sensor.snapshot()}, indent=2))
                finally:
                    sensor.close()

    def test_native_startup_and_checked_register_retries(self):
        for name, result in self.results.items():
            writes = [t for t in result["transactions"] if t["operation"] == "W"]
            self.assertEqual([(t["register"], t["value_or_count"]) for t in writes[:2]], [(0xF4, 0x57), (0xF5, 8)])
            checked = [t for t in result["transactions"] if t["operation"] == "R" and t["register"] in (0xF4, 0xF5)]
            self.assertEqual([t["register"] for t in checked[:2]], [0xF4, 0xF5])
            # 20 ms native callback, one register checked every 20 callbacks.
            # Each callback's preceding data read and checked read take 10 us.
            self.assertAlmostEqual(checked[0]["time"] - writes[1]["time"], 0.40002, delta=1e-8)
            self.assertAlmostEqual(checked[1]["time"] - writes[1]["time"], 0.80002, delta=1e-8)
            if name == "ignored_retained":
                self.assertGreaterEqual(len(writes), 8)
                self.assertTrue(all(t["state"]["commandStatus"] == 2 for t in writes if t["register"] == 0xF5))
                self.assertEqual(result["final"]["config"], 16)
                self.assertEqual(result["final"]["filterCoefficient"], 16)
                retries = [t for t in writes[2:] if t["register"] == 0xF5]
                for previous, following in zip(retries, retries[1:]):
                    self.assertAlmostEqual(following["time"] - previous["time"], 0.4, delta=1e-8)
        self.assertEqual(self.results["accepted_retained"]["final"]["filterCoefficient"], 4)
        self.assertEqual(self.results["ignored_cold"]["final"]["filterCoefficient"], 0)

    def test_sleep_configuration_repairs_same_modelica_state(self):
        result = self.results["sleep_repair"]
        writes = [t for t in result["transactions"] if t["operation"] == "W"]
        sequence = [(t["register"], t["value_or_count"]) for t in writes]
        self.assertEqual(sequence[-3:], [(0xF4, 0x54), (0xF5, 8), (0xF4, 0x57)])
        self.assertEqual(result["final"]["config"], 8)
        self.assertEqual(result["final"]["filterCoefficient"], 4)
        self.assertTrue(all(t["state"]["commandStatus"] == 1 for t in writes[-3:]))

    def test_filter_setting_changes_native_pressure_response(self):
        accepted = self.results["accepted_retained"]
        ignored = self.results["ignored_retained"]
        self.assertAlmostEqual(accepted["final"]["sampleTime"], ignored["final"]["sampleTime"], delta=1e-8)
        # Steady ramp lag difference: 20 Pa/s * .038 s * (16 - 4) = 9.12 Pa.
        # Bounds allow finite-history, raw quantization, and integer compensation.
        difference = accepted["samples"][-1]["pressure_pa"] - ignored["samples"][-1]["pressure_pa"]
        self.assertGreaterEqual(difference, 8.5)
        self.assertLessEqual(difference, 9.8)

    def test_native_measurement_and_health_boundaries(self):
        for result in self.results.values():
            reads = [t for t in result["transactions"] if t["operation"] == "R" and t["register"] == 0xF7]
            self.assertEqual(len(reads), len(result["samples"]))
            for transaction, sample in zip(reads, result["samples"]):
                raw = transaction["state"]
                # Independent specialization of Bosch's compensation formula
                # for the disclosed synthetic trim, evaluated from wire ADC counts.
                base = (1048576 - raw["rawPressure"]) * 6250 / 16384
                expected_pressure = base + base * base / 2097152
                expected_temperature = raw["rawTemperature"] / 5120 - 51.2
                self.assertLessEqual(abs(sample["pressure_pa"] - expected_pressure), 0.25)
                self.assertLessEqual(abs(sample["temperature_c"] - expected_temperature), 0.007)
        for name in ("accepted_retained", "ignored_retained", "ignored_cold", "sleep_repair"):
            result = self.results[name]
            self.assertEqual(result["samples"][-1]["healthy"], 1, name)
            self.assertLess(abs(result["samples"][-1]["temperature_c"] - 25), 0.007, name)
            # Native integer compensation and Modelica compensation read the same ADC sample.
            self.assertLess(abs(result["samples"][-1]["pressure_pa"] - result["final"]["pressure"]), 0.25, name)
        self.assertEqual(self.results["constant_protected"]["samples"][-1]["healthy"], 0)


if __name__ == "__main__":
    unittest.main()
