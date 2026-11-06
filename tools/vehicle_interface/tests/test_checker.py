"""Tests of the vehicle interface contract, checker and descriptor writer."""
from __future__ import annotations

import json
from pathlib import Path
import sys
import tempfile
import unittest
import zipfile

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from cp_vehicle_interface import (MetadataError, PrepareError, check, load_contract,  # noqa: E402
                                  parse, prepare, read_model_description)
from cp_vehicle_interface.contract import LIBRARY_ROOT  # noqa: E402
from cp_vehicle_interface.descriptor import platform_tuple, sha256  # noqa: E402

CONTRACT = load_contract()
ALL = list(CONTRACT.capabilities)
BUILT = LIBRARY_ROOT / "build" / "vehicle-interface"


def var(name, vr, *, type="Float64", causality="output", variability=None, start=None, dims=(),
        unit=None, initial=None, dim_refs=()):
    attrs = {"name": name, "valueReference": str(vr), "causality": causality}
    if variability:
        attrs["variability"] = variability
    if initial:
        attrs["initial"] = initial
    if start is not None:
        attrs["start"] = " ".join(map(str, start)) if isinstance(start, (list, tuple)) else str(start)
    if unit:
        attrs["unit"] = unit
    head = " ".join(f'{k}="{v}"' for k, v in attrs.items())
    children = "".join(f'<Dimension start="{d}"/>' for d in dims)
    children += "".join(f'<Dimension valueReference="{r}"/>' for r in dim_refs)
    return f"<{type} {head}>{children}</{type}>"


def standard_variables(n_pwm=4):
    """A complete, conforming set of variables, keyed by name for easy mutation."""
    v = {}
    vr = iter(range(1, 1000))
    v["cp_vehicle_interface_version"] = var("cp_vehicle_interface_version", next(vr), type="Int32",
                                            causality="local", variability="constant", start=1)
    v["pwm"] = var("pwm", next(vr), causality="input", start=[1000] * n_pwm, dims=[n_pwm])
    for p, s in (("pwm_min", 1100), ("pwm_max", 1900), ("lat0", 40.4), ("lon0", -86.9),
                 ("ground_alt_wgs84", 149), ("earth_radius_m", 6378137), ("mass", 1.5)):
        v[p] = var(p, next(vr), causality="parameter", variability="tunable", start=s)
    for name, n in (("state.position_ned", 3), ("state.velocity_ned", 3),
                    ("state.quaternion_wb", 4), ("state.angular_velocity_frd", 3),
                    ("accel", 3), ("gyro", 3), ("mag", 3), ("gps", 3), ("vel_ned", 3)):
        v[name] = var(name, next(vr), dims=[n])
    for name in ("yaw_deg", "baro_pressure_pa", "baro_temperature_c", "baro_altitude_m",
                 "baro_climb_rate_mps"):
        v[name] = var(name, next(vr))
    for name in ("external_force_frd", "external_moment_frd"):
        v[name] = var(name, next(vr), causality="input", start=[0, 0, 0], dims=[3])
    return v


def model_description(variables, *, fmi="3.0", cosim=True, variable_step="true", extra_cosim=""):
    cs = (f'<CoSimulation modelIdentifier="Test" canHandleVariableCommunicationStepSize='
          f'"{variable_step}"{extra_cosim}/>' if cosim else "")
    body = "".join(variables.values() if isinstance(variables, dict) else variables)
    return (f'<?xml version="1.0"?><fmiModelDescription fmiVersion="{fmi}" modelName="Test" '
            f'instantiationToken="{{token}}">{cs}<ModelVariables>{body}</ModelVariables>'
            f"</fmiModelDescription>").encode()


def run(variables=None, required=ALL, profile="standard", **kwargs):
    xml = model_description(standard_variables() if variables is None else variables, **kwargs)
    return check(parse(xml), CONTRACT, required, profile)


class ContractTests(unittest.TestCase):
    def test_contract_is_consistent(self):
        self.assertEqual(CONTRACT.version, CONTRACT.marker.value)
        names = [f.name for c in CONTRACT.capabilities.values() for f in c.fields]
        self.assertEqual(len(names), len(set(names)))
        for profile in CONTRACT.profiles.values():
            self.assertNotIn("kinematic_state_v1", profile.capabilities)

    def test_documentation_names_every_field(self):
        doc = (LIBRARY_ROOT / "docs" / "vehicle-interface.md").read_text()
        for cap in CONTRACT.capabilities.values():
            self.assertIn(cap.name, doc)
            for f in cap.fields:
                self.assertIn(f"`{f.name}`", doc, f"docs/vehicle-interface.md lacks {f.name}")
        self.assertIn(CONTRACT.marker.name, doc)


class CheckerTests(unittest.TestCase):
    def test_conforming_model_provides_every_capability(self):
        report = run()
        self.assertEqual(report.errors, [])
        self.assertEqual(sorted(report.provided), sorted(ALL))
        pwm = report.provided["pwm_actuation_v1"]["pwm"]
        self.assertEqual((pwm.count, pwm.start), (4, (1000.0,) * 4))

    def test_variable_channel_count(self):
        v = standard_variables(n_pwm=8)
        self.assertEqual(run(v).provided["pwm_actuation_v1"]["pwm"].count, 8)

    def test_dimension_from_structural_parameter(self):
        v = standard_variables()
        v["n"] = var("nPwm", 900, type="Int32", causality="structuralParameter",
                     variability="fixed", start=6)
        v["pwm"] = var("pwm", 2, causality="input", start=[1500] * 6, dim_refs=[900])
        self.assertEqual(run(v).provided["pwm_actuation_v1"]["pwm"].count, 6)
        v["n"] = var("nPwm", 900, type="Int32", causality="calculatedParameter",
                     variability="fixed")
        report = run(v)
        self.assertTrue(any("no literal start" in e for e in report.errors), report.errors)

    def test_marker_rules(self):
        v = standard_variables()
        del v["cp_vehicle_interface_version"]
        report = run(v)
        self.assertTrue(any("legacy profile" in e for e in report.errors), report.errors)
        v["cp_vehicle_interface_version"] = var("cp_vehicle_interface_version", 1, type="Int32",
                                                causality="local", variability="constant", start=2)
        self.assertTrue(any("declares version 2" in e for e in run(v).errors))
        # A final parameter as Rumoca exports it: no value in the metadata.
        v["cp_vehicle_interface_version"] = var("cp_vehicle_interface_version", 1, type="Int32",
                                                causality="calculatedParameter",
                                                variability="fixed", initial="calculated")
        errors = run(v).errors
        self.assertTrue(any("no literal value" in e for e in errors), errors)
        self.assertTrue(any("causality" in e for e in errors), errors)

    def test_legacy_profile_is_explicit_and_limited(self):
        v = standard_variables()
        del v["cp_vehicle_interface_version"]
        for name in [k for k in v if k.startswith("state.")]:
            del v[name]
        legacy = ["pwm_actuation_v1", "imu_v1"]
        report = run(v, required=legacy, profile="fastdyn_legacy")
        self.assertEqual(report.errors, [])
        self.assertEqual(len(report.warnings), 1)
        report = run(v, required=["kinematic_state_v1"], profile="fastdyn_legacy")
        self.assertTrue(any("cannot provide" in e for e in report.errors), report.errors)
        # A marked FMU must not hide behind the legacy profile.
        self.assertTrue(any("use the standard profile" in e
                            for e in run(required=legacy, profile="fastdyn_legacy").errors))
        self.assertTrue(any("unknown profile" in e for e in run(profile="nope").errors))

    def test_field_errors_are_all_reported_with_names(self):
        v = standard_variables()
        v["pwm"] = var("pwm", 2, type="Int32", causality="output", start=[1000] * 4, dims=[4])
        v["state.quaternion_wb"] = var("state.quaternion_wb", 10, dims=[3])
        v["accel"] = var("accel", 11, dims=[3], unit="m/s")
        v["gyro"] = var("gyro", 12)
        report = run(v)
        joined = "\n".join(report.errors)
        for fragment in ("'pwm' must be Float64", "'pwm' must have causality input",
                         "'state.quaternion_wb' must have shape [4]", "'accel' must have unit",
                         "'gyro' must be 1 dimension"):
            self.assertIn(fragment, joined)
        for cap in ("pwm_actuation_v1", "kinematic_state_v1", "imu_v1"):
            self.assertNotIn(cap, report.provided)

    def test_start_values(self):
        v = standard_variables()
        v["pwm"] = var("pwm", 2, causality="input", dims=[4])
        self.assertTrue(any("finite literal start" in e for e in run(v).errors))
        v["pwm"] = var("pwm", 2, causality="input", start=[1000, 1000], dims=[4])
        self.assertTrue(any("ambiguous" in e for e in run(v).errors))
        v["pwm"] = var("pwm", 2, causality="input", start=[1000, "NaN", 1000, 1000], dims=[4])
        self.assertTrue(any("finite literal start" in e for e in run(v).errors))
        v["pwm"] = var("pwm", 2, causality="input", start=[1500, 1500, 1000, 1500], dims=[4])
        self.assertEqual(run(v).provided["pwm_actuation_v1"]["pwm"].start,
                         (1500.0, 1500.0, 1000.0, 1500.0))

    def test_declared_units_are_compared(self):
        v = standard_variables()
        v["accel"] = var("accel", 11, dims=[3], unit="m/s2")
        self.assertTrue(run(v).provided["imu_v1"]["accel"].unit_declared)

    def test_partial_and_missing_capabilities(self):
        v = standard_variables()
        del v["state.quaternion_wb"]
        report = run(v)
        self.assertTrue(any("partially exported" in e and "state.quaternion_wb" in e
                            for e in report.errors), report.errors)
        v = standard_variables()
        del v["external_force_frd"], v["external_moment_frd"]
        self.assertEqual(run(v, required=["pwm_actuation_v1"]).errors, [])
        self.assertTrue(any("does not provide" in e
                            for e in run(v, required=["external_wrench_v1"]).errors))
        self.assertTrue(any("unknown capability" in e
                            for e in run(required=["warp_drive_v1"]).errors))

    def test_optional_fields_may_be_absent(self):
        v = standard_variables()
        del v["yaw_deg"], v["pwm_min"], v["pwm_max"]
        report = run(v)
        self.assertEqual(report.errors, [])
        self.assertIsNone(report.provided["gnss_v1"]["yaw_deg"])

    def test_fmi_interface_requirements(self):
        self.assertTrue(any("not FMI 3" in e for e in run(fmi="2.0").errors))
        self.assertTrue(any("CoSimulation" in e for e in run(cosim=False).errors))
        self.assertTrue(any("canHandleVariableCommunicationStepSize" in e
                            for e in run(variable_step="false").errors))
        self.assertTrue(any("needsExecutionTool" in e for e in
                            run(extra_cosim=' needsExecutionTool="true"').errors))
        v = list(standard_variables().values()) + [var("accel", 999, dims=[3])]
        self.assertTrue(any("more than once" in e for e in run(v).errors))

    def test_malformed_metadata(self):
        with self.assertRaises(MetadataError):
            parse(b"<fmiModelDescription")
        with self.assertRaises(MetadataError):
            parse(b"<notAnFmu/>")
        with self.assertRaises(MetadataError):
            parse(b'<fmiModelDescription fmiVersion="3.0"><ModelVariables>'
                  b'<Float64 name="x" valueReference="a"/></ModelVariables></fmiModelDescription>')


def fake_fmu(directory: Path, variables=None, members=()) -> Path:
    fmu = directory / "test.fmu"
    with zipfile.ZipFile(fmu, "w") as archive:
        archive.writestr("modelDescription.xml",
                         model_description(variables or standard_variables()))
        archive.writestr(f"binaries/{platform_tuple()}/Test.so", b"not really a library")
        for name, data in members:
            archive.writestr(name, data)
    return fmu


class PrepareTests(unittest.TestCase):
    def test_descriptor_is_bound_to_the_extracted_files(self):
        with tempfile.TemporaryDirectory() as tmp:
            tmp = Path(tmp)
            path, descriptor, _ = prepare(fake_fmu(tmp), tmp / "out", required=ALL,
                                          overrides={"mass": 2.0})
            self.assertEqual(json.loads(path.read_text()), descriptor)
            art = descriptor["artifact"]
            self.assertEqual(art["library_sha256"], sha256(tmp / "out" / art["library"]))
            self.assertEqual(art["model_description_sha256"],
                             sha256(tmp / "out" / art["model_description"]))
            self.assertFalse(Path(art["library"]).is_absolute())
            self.assertEqual(descriptor["overrides"], {"mass": 2.0})
            self.assertIn("lat0", descriptor["parameters"])
            self.assertEqual(descriptor["marker"]["value"], 1)
            self.assertEqual(descriptor["capabilities"]["pwm_actuation_v1"]["pwm"]["count"], 4)
            self.assertEqual(descriptor["required_capabilities"], ALL)

    def test_unsupported_overrides_are_refused_together(self):
        with tempfile.TemporaryDirectory() as tmp:
            tmp = Path(tmp)
            with self.assertRaises(PrepareError) as ctx:
                prepare(fake_fmu(tmp), tmp / "out", overrides={
                    "nonexistent": 1.0, "pwm": 1500.0, "cp_vehicle_interface_version": 2.0,
                    "accel": 1.0, "mass": float("nan")})
            message = str(ctx.exception)
            for name in ("nonexistent", "pwm", "cp_vehicle_interface_version", "accel", "mass"):
                self.assertIn(f"override '{name}'", message)
            self.assertFalse((tmp / "out" / "vehicle.json").exists())

    def test_nonconforming_fmu_writes_nothing(self):
        with tempfile.TemporaryDirectory() as tmp:
            tmp = Path(tmp)
            v = standard_variables()
            del v["accel"]
            with self.assertRaises(PrepareError):
                prepare(fake_fmu(tmp, v), tmp / "out", required=["imu_v1"])
            self.assertFalse((tmp / "out").exists())

    def test_archive_paths_cannot_escape(self):
        with tempfile.TemporaryDirectory() as tmp:
            tmp = Path(tmp)
            fmu = fake_fmu(tmp, members=[("../escape.txt", b"x")])
            with self.assertRaises(PrepareError):
                prepare(fmu, tmp / "out")
            self.assertFalse((tmp / "escape.txt").exists())


@unittest.skipUnless((BUILT / "quad.fmu").is_file() and (BUILT / "hex.fmu").is_file(),
                     "build the example FMUs first (runtime/tests/run_conformance.sh)")
class GeneratedFmuTests(unittest.TestCase):
    """The exports Rumoca actually produces from the reference partials."""

    def test_examples_conform(self):
        for name, channels in (("quad", 4), ("hex", 8)):
            meta = parse(read_model_description(BUILT / f"{name}.fmu"))
            report = check(meta, CONTRACT, ALL)
            self.assertEqual(report.errors, [], name)
            self.assertEqual(report.provided["pwm_actuation_v1"]["pwm"].count, channels)
            marker = meta.variables["cp_vehicle_interface_version"]
            self.assertEqual((marker.causality, marker.variability, marker.start),
                             ("local", "constant", ("1",)))


if __name__ == "__main__":
    unittest.main()
