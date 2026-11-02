"""Minimum NED/FRD frontend and artifact behavior: python3 -m unittest discover -s tools -p test_composer.py."""
import json
import hashlib
from itertools import product
import math
import os
import csv
from pathlib import Path
import subprocess
import tempfile
import unittest
import zipfile
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
BIN = ROOT / 'tools/fire-compose/target/debug/fire-compose'


class RustComposerTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        subprocess.run(['cargo', 'build', '--locked', '--offline', '--manifest-path',
                        str(ROOT / 'tools/fire-compose/Cargo.toml')], check=True)

    def run_config(self, text, action='validate', *args):
        with tempfile.TemporaryDirectory() as work:
            config = Path(work) / 'vehicle.toml'
            config.write_text(text)
            return subprocess.run([str(BIN), action, str(config), *args],
                                  capture_output=True, text=True)

    def assert_config_equal(self, a, b):
        if isinstance(a, dict):
            self.assertEqual(set(a), set(b))
            for key in a:
                self.assert_config_equal(a[key], b[key])
        elif isinstance(a, list):
            self.assertEqual(len(a), len(b))
            for left, right in zip(a, b):
                self.assert_config_equal(left, right)
        elif isinstance(a, (int, float)) and not isinstance(a, bool):
            self.assertTrue(math.isclose(a, b, rel_tol=1e-14, abs_tol=1e-15), (a,b))
        else:
            self.assertEqual(a, b)

    def test_minimal_examples_resolve_ned_frd(self):
        for source in sorted((ROOT / 'configs').glob('*.toml')):
            with self.subTest(source=source.name):
                run = self.run_config(source.read_text(), 'validate', '--json')
                self.assertEqual(run.returncode, 0, run.stderr)
                resolved = json.loads(run.stdout)
                self.assertEqual(resolved['schema_version'], 3)
                self.assertEqual(resolved['acquisition'], 'continuous')
                self.assertLess(resolved['initial']['p_start'][2], 0)
                self.assertEqual(resolved['geometry']['R_br'][0], [[1,0,0],[0,-1,0],[0,0,-1]])
                self.assertLess(resolved['geometry']['armMount'][0][1], 0)
                self.assertNotIn('nLegs', resolved['geometry'])
                self.assertTrue(resolved['landing_gear']['enabled'])
                self.assertEqual(len(resolved['landing_gear']['position_C']), 4)

    def test_nested_physics_table_matches_standalone_configuration(self):
        standalone = ('schema_version=3\nmodel_name="NestedHexa"\nacquisition="continuous"\n'
                      '[geometry]\npreset="HexaX"\nnActuators=6\n')
        integrated = ('[Machine]\nplatform="STM32F427"\n'
                      '[FMU.models."hexa.variant".physics]\n'
                      + standalone.replace('[geometry]', '[FMU.models."hexa.variant".physics.geometry]'))
        selector = 'FMU.models."hexa.variant".physics'
        original = self.run_config(standalone, 'validate', '--json')
        selected = self.run_config(integrated, 'validate', '--config-table', selector, '--json')
        self.assertEqual(original.returncode, 0, original.stderr)
        self.assertEqual(selected.returncode, 0, selected.stderr)
        self.assert_config_equal(json.loads(original.stdout), json.loads(selected.stdout))
        with tempfile.TemporaryDirectory() as work:
            for name, raw, extra in [('standalone', standalone, ()),
                                     ('integrated', integrated, ('--config-table', selector))]:
                run = self.run_config(raw, 'emit', '--profile', 'vio',
                                      '--output-dir', str(Path(work) / name), *extra)
                self.assertEqual(run.returncode, 0, run.stderr)
            expected = (Path(work) / 'standalone/NestedHexa.mo').read_bytes()
            self.assertEqual((Path(work) / 'integrated/NestedHexa.mo').read_bytes(), expected)

    def test_invalid_table_selection_does_not_emit(self):
        valid = '[physics]\nschema_version=3\nmodel_name="SelectedQuad"\n'
        cases = [(valid, 'missing'), (valid, 'physics.schema_version'), (valid, ''),
                 (valid, 'physics]\n[other'),
                 (valid + 'unknown_field=1\n', 'physics'),
                 (valid + '[physics.motor]\ntau=-1\n', 'physics')]
        with tempfile.TemporaryDirectory() as work:
            output = Path(work) / 'generated'
            for raw, selector in cases:
                with self.subTest(raw=raw, selector=selector):
                    run = self.run_config(raw, 'emit', '--config-table', selector,
                                          '--output-dir', str(output))
                    self.assertNotEqual(run.returncode, 0)
                    self.assertFalse(output.exists())

    def test_invalid_configuration_does_not_emit(self):
        cases = [
            'schema_version=true',
            'schema_version=1',
            'schema_version=2',
            'schema_version=3\nacquisition="sampled"',
            'schema_version=3\n[geometry]\nnLegs=0',
            'schema_version=3\n[ground]\nlegStiffness=1500',
            'schema_version=3\n[landing_gear]\nenabled=1',
            'schema_version=3\n[landing_gear]\nstiffness=-1',
            'schema_version=3\n[landing_gear]\ndamping=nan',
            'schema_version=3\n[landing_gear]\ntangentialDamping=-1',
            'schema_version=3\n[landing_gear]\nfrictionCoefficient=-1',
            'schema_version=3\n[landing_gear]\ngroundZ=inf',
            'schema_version=3\n[landing_gear]\nposition_C=[[0,0,0.1]]',
            'schema_version=3\n[landing_gear]\nnLegs=6',
            'schema_version=3\nacquisition=false',
            'schema_version=3\nacquisition="unknown"',
            'schema_version=3\nmodel_name="Bad; end Bad"',
            'schema_version=3\n[motor]\ntau=nan',
            'schema_version=3\n[motor]\ntau=true',
            'schema_version=3\n[geometry]\nactuatorIndex=[1,2,3,5]',
            'schema_version=3\n[mass.aggregate]\ninertia=[[1,0,0],[0,1,0],[0,0,3]]',
            'schema_version=3\n[models.imu]\nresponse="first_order"\ntau=[0.1,0.1]',
        ]
        with tempfile.TemporaryDirectory() as work:
            output = Path(work) / 'generated'
            for raw in cases:
                with self.subTest(raw=raw):
                    run = self.run_config(raw, 'emit', '--output-dir', str(output))
                    self.assertNotEqual(run.returncode, 0)
                    self.assertFalse(output.exists())

    def test_artifacts_contain_one_plant_and_track_acquisition(self):
        raw = 'schema_version=3\nmodel_name="TestHexa"\nacquisition="continuous"\n[geometry]\npreset="HexaX"\nnActuators=8\nactuatorIndex=[1,2,3,4,6,8]\n'
        with tempfile.TemporaryDirectory() as work:
            output = Path(work) / 'generated'
            run = self.run_config(raw, 'emit', '--profile', 'vio', '--output-dir', str(output))
            self.assertEqual(run.returncode, 0, run.stderr)
            source = (output / 'TestHexa.mo').read_text()
            self.assertEqual(source.count('MultirotorWithSensors plant('), 1)
            self.assertNotIn('sampledSensors', source)
            self.assertNotIn('sampledActuators', source)
            manifest = json.loads((output / 'TestHexa.manifest.json').read_text())
            self.assertEqual(manifest['interface']['nActuators'], 8)
            self.assertEqual(manifest['event_requirements'],
                             {'ground_contact': True, 'sampled_actuators': False, 'sampled_sensors': False})
            self.assertEqual(manifest['physics']['contact'], 'four_point_spring_damper')
            self.assertEqual(manifest['core_contract'], 'ned_frd_continuous_v1')
            self.assertEqual(manifest['frames']['body'], 'FRD')
            self.assertFalse(any(k.startswith('deprecated/') for k in manifest['source']['files']))
            self.assertFalse((output / 'sources/ModelicaCPSLibrary/deprecated').exists())
            self.assertEqual(manifest['verification']['fmu_simulation'], 'not_run')
            self.assertTrue((output / 'sources/ModelicaCPSLibrary/package.mo').is_file())
            manifest['verification']['modelica_check'] = 'test-result'
            (output / 'TestHexa.manifest.json').write_text(json.dumps(manifest))
            stale = output / 'sources/ModelicaCPSLibrary/RemovedModel.mo'
            stale.write_text('model RemovedModel end RemovedModel;')
            adapters = output / 'sources/ModelicaCPSLibrary/Adapters'
            for obsolete in ('VIO', 'FastDyn'):
                package = adapters / obsolete
                package.mkdir()
                (package / 'package.mo').write_text(f'package {obsolete} end {obsolete};')
            removed_classes = ('CopterInterface', 'PartialCopterInterface',
                               'MultirotorValueAdapter', 'MultirotorFmu')
            for name in removed_classes:
                (adapters / 'VirtualIO' / f'{name}.mo').write_text(f'model {name} end {name};')
            repeat = self.run_config(raw, 'emit', '--profile', 'vio', '--output-dir', str(output))
            self.assertEqual(repeat.returncode, 0, repeat.stderr)
            self.assertFalse(stale.exists())
            for name in removed_classes:
                self.assertFalse((adapters / 'VirtualIO' / f'{name}.mo').exists())
            for obsolete in ('VIO', 'FastDyn'):
                self.assertFalse((adapters / obsolete).exists())
            manifest = json.loads((output / 'TestHexa.manifest.json').read_text())
            self.assertEqual(manifest['verification']['modelica_check'], 'test-result')

    def test_vio_and_legacy_cli_alias_share_virtualio_artifacts(self):
        for name, port, mapping in [('quad.toml', 'demand', 'linear_target_speed_ratio'),
                                    ('quad_p1.toml', 'escCommand', 'linear_voltage_modulation')]:
            with self.subTest(config=name), tempfile.TemporaryDirectory() as work:
                raw = (ROOT / 'configs' / name).read_text()
                sources, manifests = [], []
                for profile in ('vio', 'fastdyn'):
                    output = Path(work) / profile
                    run = self.run_config(raw, 'emit', '--profile', profile, '--output-dir', str(output))
                    self.assertEqual(run.returncode, 0, run.stderr)
                    source = next(output.glob('*.mo')).read_text()
                    manifest = json.loads(next(output.glob('*.manifest.json')).read_text())
                    self.assertEqual(manifest['profile'], 'vio')
                    adapter = manifest['input_adapter']
                    self.assertEqual(adapter['class'], 'ModelicaCPSLibrary.Adapters.VirtualIO.PwmInput')
                    self.assertEqual(adapter['output_contract'], 'normalized_pwm_command_v1')
                    self.assertEqual(adapter['command_meaning'], mapping)
                    self.assertEqual(adapter['propulsion_input'], 'escs.escCommand')
                    self.assertEqual((adapter['pwm_min_us'], adapter['pwm_max_us']), (1100, 1900))
                    self.assertIn('extends ModelicaCPSLibrary.Exports.FMU.CopterInterface', source)
                    self.assertIn('escs.escCommand = {commands.normalizedCommand[3],commands.normalizedCommand[2],commands.normalizedCommand[4],commands.normalizedCommand[1]}', source)
                    self.assertNotIn('Adapters.FastDyn', source)
                    if port == 'escCommand':
                        self.assertEqual(manifest['propulsion_contract']['supply'], 'internal_constant')
                    adapters = output / 'sources/ModelicaCPSLibrary/Adapters'
                    self.assertTrue((adapters / 'VirtualIO/package.mo').is_file())
                    self.assertEqual({p.stem for p in (adapters / 'VirtualIO').glob('*.mo')},
                                     {'package', 'PwmInput', 'SensorValues', 'SpiTransfer'})
                    exports = output / 'sources/ModelicaCPSLibrary/Exports/FMU'
                    self.assertTrue((exports / 'CopterInterface.mo').is_file())
                    self.assertNotIn('ambientPressure', (exports / 'PartialCopterInterface.mo').read_text())
                    self.assertIn('pressure=101325', source)
                    self.assertIn('temperature=288.15', source)
                    for obsolete in ('VIO', 'FastDyn'):
                        self.assertFalse((adapters / obsolete).exists())
                        self.assertFalse(any(path.startswith(f'Adapters/{obsolete}/')
                                             for path in manifest['source']['files']))
                    sources.append(source)
                    manifests.append(manifest)
                self.assertEqual(*sources)
                for key in ('profile', 'input_adapter', 'propulsion_contract', 'generated_model_sha256'):
                    self.assertEqual(manifests[0][key], manifests[1][key])
                run = self.run_config(raw, 'emit', '--profile', 'plant', '--output-dir', str(Path(work) / 'plant'))
                self.assertEqual(run.returncode, 0, run.stderr)
                manifest = json.loads(next((Path(work) / 'plant').glob('*.manifest.json')).read_text())
                self.assertNotIn('input_adapter', manifest)

    def test_contact_can_be_disabled_explicitly(self):
        raw = 'schema_version=3\nmodel_name="FreeVehicle"\n[landing_gear]\nenabled=false\n'
        with tempfile.TemporaryDirectory() as work:
            run = self.run_config(raw, 'emit', '--output-dir', work)
            self.assertEqual(run.returncode, 0, run.stderr)
            manifest = json.loads((Path(work)/'FreeVehicle.manifest.json').read_text())
            self.assertFalse(manifest['event_requirements']['ground_contact'])
            self.assertEqual(manifest['physics']['contact'], 'none')
            self.assertFalse(manifest['config']['landing_gear']['enabled'])

    def test_mass_ids_stay_in_manifest_and_do_not_change_physics_source(self):
        aggregate = (ROOT / 'configs/quad.toml').read_text().replace(
            '[mass.aggregate]', '[mass.aggregate]\ncomponentId="vehicle.aggregate"')
        assembled = (ROOT / 'configs/payload.toml').read_text() + (
            '\n[[mass.additional_parts]]\ncomponentId="battery"\nmass=0.1\n'
            'inertia=[[0.001,0,0],[0,0.001,0],[0,0,0.001]]\n')
        for raw, model, expected in [
            (aggregate, 'ConfiguredQuad', ['vehicle.aggregate']),
            (assembled, 'ConfiguredPayload',
             ['frame.core', 'arm.1', 'arm.2', 'arm.3', 'arm.4', 'payload.camera', 'battery']),
        ]:
            for profile in ('plant', 'vio'):
                with self.subTest(model=model, profile=profile), tempfile.TemporaryDirectory() as work:
                    output = Path(work)
                    run = self.run_config(raw, 'emit', '--profile', profile, '--output-dir', work)
                    self.assertEqual(run.returncode, 0, run.stderr)
                    source = (output / f'{model}.mo').read_text()
                    self.assertNotIn('componentId', source)
                    manifest = json.loads((output / f'{model}.manifest.json').read_text())
                    mass = manifest['config']['mass']
                    parts = ([mass['aggregate']] if mass['mode'] == 'aggregate' else
                             [mass['core'], *mass['arms'], *mass['payloads'], *mass['additional_parts']])
                    self.assertEqual([p['componentId'] for p in parts], expected)
                    renamed = raw
                    for identity in expected:
                        renamed = renamed.replace(f'"{identity}"', f'"renamed.{identity}"')
                    run = self.run_config(renamed, 'emit', '--profile', profile, '--output-dir', work)
                    self.assertEqual(run.returncode, 0, run.stderr)
                    self.assertEqual((output / f'{model}.mo').read_text(), source)
                    updated = json.loads((output / f'{model}.manifest.json').read_text())
                    self.assertNotEqual(updated['config_sha256'], manifest['config_sha256'])

    def test_duplicate_mass_ids_are_rejected_before_emission(self):
        raw = (ROOT / 'configs/payload.toml').read_text()
        duplicates = [
            raw.replace('"payload.camera"', '"frame.core"'),
            raw.replace('"arm.2"', '"arm.1"'),
            raw.replace('"frame.core"', '"arm.1"'),
            raw + ('\n[[mass.additional_parts]]\ncomponentId="payload.camera"\nmass=0.1\n'
                   'inertia=[[0.001,0,0],[0,0.001,0],[0,0,0.001]]\n'),
        ]
        for index, duplicate in enumerate(duplicates):
            with self.subTest(collision=index), tempfile.TemporaryDirectory() as work:
                output = Path(work) / 'generated'
                run = self.run_config(duplicate, 'emit', '--output-dir', str(output))
                self.assertNotEqual(run.returncode, 0)
                self.assertIn('duplicate nonempty componentId', run.stderr)
                self.assertFalse(output.exists())
        for identity in ['frame.core', 'arm.1', 'arm.2', 'arm.3', 'arm.4', 'payload.camera']:
            raw = raw.replace(f'"{identity}"', '""')
        run = self.run_config(raw, 'validate')
        self.assertEqual(run.returncode, 0, run.stderr)

    @unittest.skipUnless(os.environ.get("FIRE_TEST_OMC") == "1", "set FIRE_TEST_OMC=1")
    def test_composed_assembled_mass_preserves_mass_and_cg(self):
        raw = (ROOT / 'configs/payload.toml').read_text()
        with tempfile.TemporaryDirectory() as work:
            work = Path(work)
            run = self.run_config(raw, 'emit', '--profile', 'vio', '--output-dir', str(work/'generated'))
            self.assertEqual(run.returncode, 0, run.stderr)
            harness = work/'MassRun.mo'
            harness.write_text('''model MassRun
  extends ConfiguredPayload(pwm=fill(1100,4));
initial equation
  assert(abs(plant.chassis.mass-1.7)<1e-12,"Composed assembled mass changed");
  assert(max(abs(plant.chassis.cg_C-{0.018/1.7,0,0.024/1.7}))<1e-12,
    "Composed assembled CG changed");
end MassRun;
''')
            files = [work/'generated/sources/ModelicaCPSLibrary/package.mo',
                     work/'generated/ConfiguredPayload.mo', harness]
            script = work/'simulate.mos'
            script.write_text('loadModel(Modelica,{"4.0.0"});\n'
                + ''.join('loadFile('+json.dumps(str(f))+');\n' for f in files)
                + 'simulate(MassRun,stopTime=0.01);\ngetErrorString();\n')
            run = subprocess.run(['omc',str(script)],cwd=work,text=True,capture_output=True,timeout=120)
            self.assertIn('The simulation finished successfully',run.stdout,run.stdout+run.stderr)

    @unittest.skipUnless(os.environ.get("FIRE_TEST_OMC") == "1", "set FIRE_TEST_OMC=1")
    def test_composed_ground_support_and_takeoff(self):
        for preset, count in [('QuadX', 4), ('HexaX', 6)]:
            with self.subTest(preset=preset), tempfile.TemporaryDirectory() as work:
                work = Path(work)
                raw = (f'schema_version=3\nmodel_name="GroundVehicle"\n[geometry]\npreset="{preset}"\n'
                       '[landing_gear]\nenabled=true\ngroundZ=0.4\nstiffness=2400\ndamping=120\n'
                       'position_C=[[0.17,0.17,0.15],[-0.17,-0.17,0.15],'
                       '[0.17,-0.17,0.15],[-0.17,0.17,0.15]]\n'
                       '[initial]\np_start=[0,0,0.23]\n')
                run = self.run_config(raw, 'emit', '--profile', 'vio', '--output-dir', str(work/'generated'))
                self.assertEqual(run.returncode, 0, run.stderr)
                thrust_pwm = 1100 + 800*math.sqrt(1.3*1.5*9.80665/(count*1e-5))/1000
                harness = work/'GroundRun.mo'
                harness.write_text(f'''model GroundRun
  extends GroundVehicle(pwm=fill(if time<2 then 1100 else {thrust_pwm},{count}));
equation
  when time>=1.5 then
    assert(abs(truth.p_w[3]-(0.4-0.15+1.5*9.80665/9600))<1e-6,"Composed ground height/compression failed");
    assert(abs(accel[3]+9.80665)<1e-4,"Composed boot accelerometer must read -g");
    assert(sum(if plant.gear.contact[i] then 1 else 0 for i in 1:4)==4,"Expected four supporting feet");
  end when;
  when time>=2.5 then
    assert(abs(sum(plant.gear.normalForce))<1e-12 and truth.v_w[3]<0,"Composed vehicle must take off");
  end when;
end GroundRun;
''')
                files = [work/'generated/sources/ModelicaCPSLibrary/package.mo',work/'generated/GroundVehicle.mo',harness]
                script = work/'simulate.mos'
                script.write_text('loadModel(Modelica,{"4.0.0"});\n'+''.join('loadFile('+json.dumps(str(f))+');\n' for f in files)
                    +'simulate(GroundRun,stopTime=2.6,tolerance=1e-8,outputFormat="csv");\ngetErrorString();\n')
                run = subprocess.run(['omc',str(script)],cwd=work,text=True,capture_output=True,timeout=120)
                self.assertIn('The simulation finished successfully',run.stdout,run.stdout+run.stderr)
                with (work/'GroundRun_res.csv').open() as handle:
                    last = list(csv.DictReader(handle))[-1]
                self.assertAlmostEqual(float(last['time']),2.6)
                self.assertLess(float(last['truth.p_w[3]']),0.2)

    def test_previous_namespace_outputs_can_be_regenerated(self):
        raw = 'schema_version=3\nmodel_name="MigratedQuad"'
        for namespace in ('fire_modelica_models', 'FIRE_Modelica'):
            with self.subTest(namespace=namespace), tempfile.TemporaryDirectory() as work:
                output = Path(work)
                run = self.run_config(raw, 'emit', '--output-dir', work)
                self.assertEqual(run.returncode, 0, run.stderr)
                model = output / 'MigratedQuad.mo'
                model.write_text(model.read_text().replace('ModelicaCPSLibrary', namespace))
                manifest_path = output / 'MigratedQuad.manifest.json'
                manifest = json.loads(manifest_path.read_text())
                manifest['generator_id'] = f'{namespace}/tools/fire-compose'
                manifest['generated_model_sha256'] = hashlib.sha256(model.read_bytes()).hexdigest()
                manifest['verification']['modelica_check'] = 'old-result'
                manifest_path.write_text(json.dumps(manifest))
                old = output / 'sources' / namespace
                (output / 'sources/ModelicaCPSLibrary').rename(old)
                (old / '.fire-compose').write_text(f'{namespace}/tools/fire-compose')
                run = self.run_config(raw, 'emit', '--output-dir', work)
                self.assertEqual(run.returncode, 0, run.stderr)
                self.assertNotIn(namespace, model.read_text())
                self.assertIn('ModelicaCPSLibrary.Vehicles.', model.read_text())
                self.assertFalse(old.exists())
                self.assertTrue((output / 'sources/ModelicaCPSLibrary/package.mo').is_file())
                updated = json.loads(manifest_path.read_text())
                self.assertEqual(updated['generator_id'], 'ModelicaCPSLibrary/tools/fire-compose')
                self.assertEqual(updated['verification']['modelica_check'], 'not_run')

    def test_previous_namespace_cleanup_preserves_unowned_and_symlink_trees(self):
        raw = 'schema_version=3\nmodel_name="PreservedQuad"'
        for namespace, symlink in product(('fire_modelica_models', 'FIRE_Modelica'), (False, True)):
            with self.subTest(namespace=namespace, symlink=symlink), tempfile.TemporaryDirectory() as work:
                output = Path(work) / 'generated'
                sources = output / 'sources'
                sources.mkdir(parents=True)
                old = sources / namespace
                if symlink:
                    target = Path(work) / 'external'
                    target.mkdir()
                    (target / '.fire-compose').write_text(f'{namespace}/tools/fire-compose')
                    old.symlink_to(target, target_is_directory=True)
                else:
                    old.mkdir()
                (old / 'user.mo').write_text('user-authored')
                run = self.run_config(raw, 'emit', '--output-dir', str(output))
                self.assertEqual(run.returncode, 0, run.stderr)
                self.assertEqual((old / 'user.mo').read_text(), 'user-authored')
                self.assertEqual(old.is_symlink(), symlink)

    def test_user_source_and_symlink_outputs_are_preserved(self):
        raw = 'schema_version=3\nmodel_name="KeepMe"'
        with tempfile.TemporaryDirectory() as work:
            output = Path(work)
            source = output / 'KeepMe.mo'
            source.write_text('user-authored')
            run = self.run_config(raw, 'emit', '--output-dir', work)
            self.assertNotEqual(run.returncode, 0)
            self.assertEqual(source.read_text(), 'user-authored')
            source.unlink()
            target = output / 'user.mo'
            target.write_text('user-authored')
            source.symlink_to(target)
            run = self.run_config(raw, 'emit', '--output-dir', work)
            self.assertNotEqual(run.returncode, 0)
            self.assertEqual(target.read_text(), 'user-authored')

    @unittest.skipUnless(os.environ.get("FIRE_TEST_OMC") == "1", "set FIRE_TEST_OMC=1")
    def test_composed_quad_and_hexa_motor_response(self):
        for preset, count in [("QuadX",4),("HexaX",6)]:
            with self.subTest(preset=preset), tempfile.TemporaryDirectory() as work:
                work = Path(work)
                raw = f'schema_version=3\nacquisition="continuous"\nmodel_name="ComposedVehicle"\n[geometry]\npreset="{preset}"\n'
                run = self.run_config(raw, 'emit', '--profile', 'vio', '--output-dir', str(work/'generated'))
                self.assertEqual(run.returncode, 0, run.stderr)
                harness = work/'MotorRun.mo'
                harness.write_text(f'model MotorRun\n  extends ComposedVehicle(pwm=fill(1500,{count}));\nend MotorRun;\n')
                script = work/'simulate.mos'
                files = [work/'generated/sources/ModelicaCPSLibrary/package.mo',work/'generated/ComposedVehicle.mo',harness]
                script.write_text('loadModel(Modelica, {"4.0.0"});\n'+''.join('loadFile('+json.dumps(str(f))+');\n' for f in files)
                    +'simulate(MotorRun, stopTime=0.1, tolerance=1e-9, numberOfIntervals=100, outputFormat="csv");\ngetErrorString();\n')
                run = subprocess.run(['omc',str(script)],cwd=work,text=True,capture_output=True,timeout=120)
                self.assertIn('The simulation finished successfully',run.stdout,run.stdout+run.stderr)
                with (work/'MotorRun_res.csv').open() as handle:
                    last = list(csv.DictReader(handle))[-1]
                self.assertAlmostEqual(float(last['time']),0.1)
                expected = 500*(1-math.exp(-0.1/0.03))
                for i in range(1,count+1):
                    self.assertAlmostEqual(float(last[f'rotorSpeed[{i}]']),expected,delta=1e-4)
                self.assertAlmostEqual(float(last['yaw_deg']),90.0,delta=1e-6)
                self.assertLess(float(last['accel[3]']),0)
                self.assertAlmostEqual(float(last['baro_altitude_m']),-float(last['truth.p_w[3]']),delta=1e-8)
                for key in ['gps[1]','gps[2]','gps[3]','accel[3]','baro_pressure_pa']:
                    self.assertTrue(math.isfinite(float(last[key])),key)

    @unittest.skipUnless(os.environ.get('FIRE_TEST_OMC') == '1', 'set FIRE_TEST_OMC=1')
    def test_composed_eight_channel_adapter_and_sensor_mounts(self):
        for motor_model in ('first_order_speed', 'first_order_voltage_to_speed'):
            with self.subTest(motor=motor_model), tempfile.TemporaryDirectory() as work:
                work = Path(work)
                esc_model = 'average_voltage' if motor_model == 'first_order_voltage_to_speed' else 'pass_through'
                raw = ('schema_version=3\nmodel_name="ChannelsVehicle"\n'
                       '[geometry]\npreset="OctoX"\n'
                       '[initial]\np_start=[1,2,-10]\n'
                       '[landing_gear]\nenabled=false\n'
                       '[interface]\npwm_min=1000\npwm_max=2000\n'
                       f'[models]\nesc="{esc_model}"\nmotor="{motor_model}"\nrotor="quadratic_blade"\n')
                if motor_model == 'first_order_voltage_to_speed':
                    raw += '[motor]\nKv=62.5\n[power]\ndcVoltage=16\n'
                run = self.run_config(raw, 'emit', '--profile', 'vio', '--output-dir', str(work/'generated'))
                self.assertEqual(run.returncode, 0, run.stderr)
                harness = work/'ChannelsRun.mo'
                harness.write_text('''model ChannelsRun
  extends ChannelsVehicle(geometry(R_bImu=[0,-1,0;1,0,0;0,0,1],R_bMag=diagonal({1,-1,-1})),
    plant(gravity_w=zeros(3)),pwm={900,1100,1200,1300,1400,1500,1600,2100});
equation
  when time>=0.3 then
    assert(max(abs(commands.normalizedCommand-{0,0.1,0.2,0.3,0.4,0.5,0.6,1}))<1e-12,"PWM channel clipping changed");
    assert(max(abs(rotorSpeed-1000*(1-exp(-time/0.03))*{0,0.1,0.2,0.3,0.4,0.5,0.6,1}))<0.01,"Eight independent speed responses changed");
    assert(max(abs(values.position_ned-plant.measurements.position))<1e-12,"NED position changed");
    assert(max(abs(accel-{-plant.measurements.acceleration[2],plant.measurements.acceleration[1],plant.measurements.acceleration[3]}))<1e-12,"IMU mount conversion changed");
    assert(max(abs(mag-10000*diagonal({1,-1,-1})*plant.measurements.magneticField))<1e-12,"Mag mount/Gauss conversion changed");
    assert(abs(baro_pressure_pa-101325)<1e-8 and abs(baro_temperature_c-15)<1e-8,"Plant atmospheric supply changed");
  end when;
end ChannelsRun;
''')
                files = [work/'generated/sources/ModelicaCPSLibrary/package.mo',
                         work/'generated/ChannelsVehicle.mo', harness]
                script = work/'simulate.mos'
                script.write_text('loadModel(Modelica,{"4.0.0"});\n'
                    + ''.join('loadFile('+json.dumps(str(f))+');\n' for f in files)
                    + 'simulate(ChannelsRun,stopTime=0.31,tolerance=1e-9);\ngetErrorString();\n')
                run = subprocess.run(['omc', str(script)], cwd=work, text=True, capture_output=True, timeout=120)
                self.assertIn('The simulation finished successfully', run.stdout, run.stdout+run.stderr)

    def test_p1_contracts_and_supply_are_explicit(self):
        for config in ('quad_p1.toml', 'hex_p1.toml'):
            for profile in ('plant', 'vio'):
                with self.subTest(config=config, profile=profile), tempfile.TemporaryDirectory() as work:
                    raw = (ROOT / 'configs' / config).read_text()
                    run = self.run_config(raw, 'emit', '--profile', profile, '--output-dir', work)
                    self.assertEqual(run.returncode, 0, run.stderr)
                    manifest = json.loads(next(Path(work).glob('*.manifest.json')).read_text())
                    self.assertEqual(manifest['physics']['propulsion'], 'P1')
                    contract = manifest['propulsion_contract']
                    self.assertEqual(contract['command'], 'esc_command_v1')
                    self.assertEqual(contract['control_mode'], 'unipolar_voltage_modulation')
                    self.assertEqual(contract['supply'], 'internal_constant' if profile == 'vio' else 'input_default')
                    self.assertFalse(contract['power_accounting'])
                    self.assertNotIn('omegaMax', manifest['config']['motor'])
                    source = next(Path(work).glob('*.mo')).read_text()
                    self.assertIn('Vehicles.Copter.MultirotorWithSensors', source)
                    self.assertNotIn('VoltageDrivenMultirotor', source)
                    self.assertIn('ESC.AverageVoltage escs[', source)
                    self.assertIn('Motors.FirstOrderVoltageToSpeed motors[', source)
                    self.assertIn('Blades.QuadraticBlade rotors[', source)
                    self.assertEqual(manifest['config']['power']['dcVoltage'], 16)
                    self.assertNotIn('commands.demand', source)
                    if profile == 'vio':
                        self.assertIn('escs.escCommand = {commands.normalizedCommand[', source)
                        self.assertIn('plant.rotorForce_r = rotors.force_r', source)
                        self.assertIn('Exports.FMU.CopterInterface', source)
                    self.assertTrue((Path(work) / 'sources/ModelicaCPSLibrary/Physical/Electrical/Drives/ESC/AverageVoltage.mo').is_file())

    def test_p1_rejects_missing_and_cross_profile_parameters(self):
        raw = 'schema_version=3\n[models]\nesc="average_voltage"\nmotor="first_order_voltage_to_speed"\nrotor="quadratic_blade"\n[motor]\nKv=62.5\n[power]\ndcVoltage=16\n'
        cases = [raw.replace('average_voltage', 'pass_through'),
                 raw.replace('first_order_voltage_to_speed', 'first_order_speed'),
                 raw.replace('rotor="quadratic_blade"', 'rotor="voltage_driven"'),
                 raw.replace('Kv=62.5', ''), raw.replace('[power]\ndcVoltage=16', ''),
                 raw.replace('Kv=62.5', 'omegaMax=1000'),
                 raw.replace('Kv=62.5', 'Kv=62.5\ntau=0.03'),
                 raw.replace('Kv=62.5', 'Kv=0'), raw.replace('Kv=62.5', 'Kv=nan'),
                 raw.replace('Kv=62.5', 'Kv=[62.5,62.5]'),
                 raw.replace('Kv=62.5', 'Kv=62.5\ntauOmega=-0.03'),
                 raw.replace('Kv=62.5', 'Kv=62.5\nomega_start=-1'),
                 raw.replace('dcVoltage=16', 'dcVoltage=-1'),
                 raw.replace('dcVoltage=16', 'dcVoltage=inf'),
                 raw.replace('dcVoltage=16', 'dcVoltage=[16,16,16,16]'),
                 raw.replace('dcVoltage=16', 'dcVoltage=true'),
                 'schema_version=3\n[power]\ndcVoltage=16',
                 'schema_version=3\n[motor]\nKv=62.5',
                 'schema_version=3\n[models]\nrotor="electric"']
        with tempfile.TemporaryDirectory() as work:
            output = Path(work) / 'generated'
            for invalid in cases:
                with self.subTest(raw=invalid):
                    run = self.run_config(invalid, 'emit', '--output-dir', str(output))
                    self.assertNotEqual(run.returncode, 0)
                    self.assertFalse(output.exists())
        run = self.run_config(raw.replace('dcVoltage=16', 'dcVoltage=0'))
        self.assertEqual(run.returncode, 0, run.stderr)

    def test_switching_propulsion_invalidates_verification(self):
        with tempfile.TemporaryDirectory() as work:
            p0 = 'schema_version=3\nmodel_name="Switchable"\n'
            run = self.run_config(p0, 'emit', '--profile', 'vio', '--output-dir', work)
            self.assertEqual(run.returncode, 0, run.stderr)
            path = Path(work) / 'Switchable.manifest.json'
            manifest = json.loads(path.read_text())
            self.assertEqual(manifest['physics']['propulsion'], 'P0')
            self.assertEqual(manifest['propulsion_contract']['command'], 'speed_command_v1')
            manifest['verification']['fmu_simulation'] = 'passed'
            path.write_text(json.dumps(manifest))
            p1 = p0 + '[models]\nesc="average_voltage"\nmotor="first_order_voltage_to_speed"\nrotor="quadratic_blade"\n[motor]\nKv=62.5\n[power]\ndcVoltage=16\n'
            run = self.run_config(p1, 'emit', '--profile', 'vio', '--output-dir', work)
            self.assertEqual(run.returncode, 0, run.stderr)
            manifest = json.loads(path.read_text())
            self.assertEqual(manifest['physics']['propulsion'], 'P1')
            self.assertEqual(manifest['verification']['fmu_simulation'], 'not_run')

    @unittest.skipUnless(os.environ.get('FIRE_TEST_OMC') == '1', 'set FIRE_TEST_OMC=1')
    def test_p1_composed_channels_and_voltage_response(self):
        for preset, channels, mapping in [('QuadX', 4, [3,2,4,1]), ('HexaX', 8, [1,2,3,4,6,8])]:
            n = len(mapping)
            gains = [50+5*i for i in range(n)]
            taus = [0.02+0.003*i for i in range(n)]
            starts = [10*(i+1) for i in range(n)]
            for profile in ('plant', 'vio'):
                with self.subTest(preset=preset, profile=profile), tempfile.TemporaryDirectory() as work:
                    work = Path(work)
                    raw = ('schema_version=3\nmodel_name="P1Vehicle"\n'
                           f'[geometry]\npreset="{preset}"\nnActuators={channels}\nactuatorIndex={mapping}\n'
                           '[models]\nesc="average_voltage"\nmotor="first_order_voltage_to_speed"\nrotor="quadratic_blade"\n'
                           f'[motor]\nKv={gains}\ntauOmega={taus}\nomega_start={starts}\n'
                           '[power]\ndcVoltage=8\n[landing_gear]\nenabled=false\n')
                    run = self.run_config(raw, 'emit', '--profile', profile, '--output-dir', str(work/'generated'))
                    self.assertEqual(run.returncode, 0, run.stderr)
                    command = [0.1*(i+1) for i in range(channels)]
                    values = [1100+800*u for u in command] if profile == 'vio' else command
                    port = 'pwm' if profile == 'vio' else 'normalizedCommand'
                    harness = work/'P1Run.mo'
                    harness.write_text('model P1Run\n  P1Vehicle vehicle('+port+'={'+','.join(map(str,values))+'});\nend P1Run;\n')
                    files = [work/'generated/sources/ModelicaCPSLibrary/package.mo', work/'generated/P1Vehicle.mo', harness]
                    script = work/'simulate.mos'
                    script.write_text('loadModel(Modelica,{"4.0.0"});\n'
                        + ''.join('loadFile('+json.dumps(str(f))+');\n' for f in files)
                        + 'simulate(P1Run,stopTime=0.08,tolerance=1e-10,outputFormat="csv");\ngetErrorString();\n')
                    run = subprocess.run(['omc',str(script)],cwd=work,text=True,capture_output=True,timeout=120)
                    self.assertIn('The simulation finished successfully',run.stdout,run.stdout+run.stderr)
                    with (work/'P1Run_res.csv').open() as stream:
                        final = list(csv.DictReader(stream))[-1]
                    for i, channel in enumerate(mapping):
                        target = gains[i]*8*command[channel-1]
                        expected = target+(starts[i]-target)*math.exp(-0.08/taus[i])
                        key = f'vehicle.rotorSpeed[{i+1}]'
                        self.assertAlmostEqual(float(final[key]),expected,delta=1e-4)

    def test_rotor_parameters_are_independent_of_motor_response(self):
        raw = 'schema_version=3\n[rotor]\nkT=[0,1e-5,2e-5,3e-5]\nkQ=0\n'
        run = self.run_config(raw, 'validate', '--json')
        self.assertEqual(run.returncode, 0, run.stderr)
        resolved = json.loads(run.stdout)
        self.assertNotIn('kT', resolved['motor'])
        self.assertEqual(resolved['rotor']['kQ'], [0]*4)
        for invalid in (raw.replace('kQ=0', 'kQ=-1'),
                        raw.replace('kQ=0', 'kQ=nan'),
                        raw.replace('kQ=0', 'kQ=[1,2]'),
                        'schema_version=3\n[motor]\nkT=1e-5',
                        'schema_version=3\n[models]\nesc="unknown"'):
            with self.subTest(raw=invalid), tempfile.TemporaryDirectory() as work:
                output = Path(work)/'generated'
                run = self.run_config(invalid, 'emit', '--output-dir', str(output))
                self.assertNotEqual(run.returncode, 0)
                self.assertFalse(output.exists())

    @unittest.skipUnless(os.environ.get('FIRE_TEST_OMC') == '1', 'set FIRE_TEST_OMC=1')
    def test_composed_coaxial_shared_channels_and_supply_step(self):
        # Four commands drive eight independent motors with unequal response/aero parameters.
        mapping = [1,2,3,4,1,2,3,4]
        command = [0.1,0.2,0.3,0.4]
        gains = [50+5*i for i in range(8)]
        taus = [0.02+0.003*i for i in range(8)]
        starts = [10*(i+1) for i in range(8)]
        thrust = [1e-5*(i+1) for i in range(8)]
        torque = [1e-7*(i+1) for i in range(8)]
        common = (f'[geometry]\npreset="CoaxialX8"\nnActuators=4\nactuatorIndex={mapping}\n'
                  '[landing_gear]\nenabled=false\n'
                  f'[rotor]\nkT={thrust}\nkQ={torque}\n')
        with tempfile.TemporaryDirectory() as work:
            work = Path(work)
            for p1 in (False, True):
                name = 'SupplyP1' if p1 else 'SupplyP0'
                raw = f'schema_version=3\nmodel_name="{name}"\n'+common
                raw += f'[motor]\nomega_start={starts}\n'
                if p1:
                    raw += (f'Kv={gains}\ntauOmega={taus}\n[power]\ndcVoltage=8\n'
                            '[models]\nesc="average_voltage"\nmotor="first_order_voltage_to_speed"\n')
                else:
                    raw += f'omegaMax={[8*g for g in gains]}\ntau={taus}\n'
                run = self.run_config(raw, 'emit', '--profile', 'plant', '--output-dir', str(work/'generated'))
                self.assertEqual(run.returncode, 0, run.stderr)
            harness = work/'SupplyRun.mo'
            harness.write_text("""model SupplyRun
  SupplyP0 p0(normalizedCommand={0.1,0.2,0.3,0.4});
  SupplyP1 p1(normalizedCommand={0.1,0.2,0.3,0.4},dcVoltage=if time<0.06 then 8 else 4);
equation
  when time>=0.05 then
    assert(max(abs(p0.rotorSpeed-p1.rotorSpeed))<1e-6,"Nominal independent motors must agree");
    assert(max(abs(p0.force_b-p1.force_b))<1e-6,"Nominal common vehicle force must agree");
    assert(max(abs(p0.moment_b-p1.moment_b))<1e-6,"Nominal hub/CG torque must agree");
  end when;
end SupplyRun;
""")
            files = [work/'generated/sources/ModelicaCPSLibrary/package.mo',
                     work/'generated/SupplyP0.mo', work/'generated/SupplyP1.mo', harness]
            script = work/'simulate.mos'
            script.write_text('loadModel(Modelica,{"4.0.0"});\n'
                + ''.join('loadFile('+json.dumps(str(f))+');\n' for f in files)
                + 'simulate(SupplyRun,stopTime=0.12,tolerance=1e-10,outputFormat="csv");\ngetErrorString();\n')
            run = subprocess.run(['omc',str(script)],cwd=work,text=True,capture_output=True,timeout=120)
            self.assertIn('The simulation finished successfully',run.stdout,run.stdout+run.stderr)
            with (work/'SupplyRun_res.csv').open() as stream:
                final = list(csv.DictReader(stream))[-1]
            for i, channel in enumerate(mapping):
                target = gains[i]*8*command[channel-1]
                p0 = target+(starts[i]-target)*math.exp(-0.12/taus[i])
                at_step = target+(starts[i]-target)*math.exp(-0.06/taus[i])
                p1 = target/2+(at_step-target/2)*math.exp(-0.06/taus[i])
                for name, expected in [('p0',p0),('p1',p1)]:
                    self.assertAlmostEqual(float(final[f'{name}.rotorSpeed[{i+1}]']),expected,delta=1e-4)
                    self.assertAlmostEqual(float(final[f'{name}.rotors[{i+1}].force_r[3]']),
                                           thrust[i]*expected**2,delta=1e-5)
                    spin = (1 if i%2==0 else -1)*(1 if i<4 else -1)
                    self.assertAlmostEqual(float(final[f'{name}.rotors[{i+1}].moment_r[3]']),
                                           -spin*torque[i]*expected**2,delta=1e-6)

    @unittest.skipUnless(os.environ.get('FIRE_TEST_OMC') == '1', 'set FIRE_TEST_OMC=1')
    def test_plant_fmu_keeps_internal_wrenches_out_of_inputs(self):
        for config in ('quad.toml', 'quad_p1.toml'):
            with self.subTest(config=config), tempfile.TemporaryDirectory() as work:
                work = Path(work)
                run = self.run_config((ROOT/'configs'/config).read_text(), 'emit',
                                      '--profile', 'plant', '--output-dir', str(work/'generated'))
                self.assertEqual(run.returncode, 0, run.stderr)
                model_file = next((work/'generated').glob('*.mo'))
                files = [work/'generated/sources/ModelicaCPSLibrary/package.mo', model_file]
                script = work/'export.mos'
                script.write_text('loadModel(Modelica,{"4.0.0"});\n'
                    + ''.join('loadFile('+json.dumps(str(f))+');\n' for f in files)
                    + f'buildModelFMU({model_file.stem},version="2.0",fmuType="cs",platforms={{"static"}});\ngetErrorString();\n')
                run = subprocess.run(['omc',str(script)],cwd=work,text=True,capture_output=True,timeout=120)
                fmu = work/(model_file.stem+'.fmu')
                self.assertTrue(fmu.is_file(),run.stdout+run.stderr)
                with zipfile.ZipFile(fmu) as archive:
                    model = ET.fromstring(archive.read('modelDescription.xml'))
                inputs = {v.get('name').split('[')[0] for v in model.find('ModelVariables')
                          if v.get('causality') == 'input'}
                expected = {'normalizedCommand','gravity_w','wind_w','externalForce_b',
                            'externalMoment_b','magneticField_w','pressure','temperature'}
                if config == 'quad_p1.toml':
                    expected.add('dcVoltage')
                # OMC may fold default-bound environmental arrays into constants;
                # command inputs must remain public and computed hub wrenches must not.
                self.assertTrue(inputs <= expected, inputs-expected)
                self.assertIn('normalizedCommand', inputs)
                self.assertEqual('dcVoltage' in inputs, config == 'quad_p1.toml')
