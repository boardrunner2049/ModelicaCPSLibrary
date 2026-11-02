"""P3/P4 configuration, topology and native multicopter integration contracts."""
import csv
import json
import os
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
BIN = ROOT / 'tools/fire-compose/target/debug/fire-compose'


class ThreePhaseComposerTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        subprocess.run(['cargo', 'build', '--locked', '--offline', '--manifest-path',
                        str(ROOT / 'tools/fire-compose/Cargo.toml')], check=True)

    def emit(self, raw, work, profile='plant'):
        source = work / 'physics.toml'
        source.write_text(raw)
        return subprocess.run([str(BIN), 'emit', str(source), '--profile', profile,
                               '--output-dir', str(work / 'generated')], capture_output=True, text=True)

    def test_physical_parameters_and_modes_are_validated_before_emission(self):
        raw = (ROOT / 'configs/quad_p4.toml').read_text()
        invalid = [raw.replace(line, '') for line in [
            'RPhase = 0.2', 'LPhase = 0.001', 'ke = 0.01', 'polePairs = 7', 'J = 0.0001',
            'Ron = 0.005', 'Rdiode = 0.005', 'Vdiode = 0.5', 'pwmFrequency = 20000.0',
            'sourceResistance = 0.02', 'dcLinkCapacitance = 0.002', 'dcVoltage = 16.0']]
        invalid += [raw.replace('LPhase = 0.001', 'LPhase = ' + v)
                    for v in ['0', '-1', 'nan', 'inf', 'true', '[0.001, 0.002]']]
        invalid += [raw.replace(a, b) for a, b in [
            ('polePairs = 7', 'polePairs = 2.5'), ('polePairs = 7', 'polePairs = 0'),
            ('trapezoidal = true', 'trapezoidal = 1'), ('Goff = 1e-5', 'Goff = 0'),
            ('deadTime = 0.0', 'deadTime = 0.000025'), ('deadTime = 0.0', 'deadTime = -1'),
            ('currentThreshold = 0.001', 'currentThreshold = 0'),
            ('Vdiode = 0.5', 'Vdiode = -1'), ('sourceResistance = 0.02', 'sourceResistance = 0'),
            ('RPhase = 0.2', 'RPhase = 0.2\nR = 0.2'),
            ('three_phase_pm', 'dynamic_dc_equivalent'),
            ('switching_three_phase', 'average_unidirectional'),
            ('switching_three_phase', 'average_three_phase'),  # PWM-only keys must not be ignored.
            ('omega_start = 0.0', 'omega_start = -1'),
            ('omega_start = 0.0', 'omega_start = 0.0\ni_start = [0,0,0]'),
            ('omega_start = 0.0', 'omega_start = 0.0\ni_start = [[1,0,0],[0,0,0],[0,0,0],[0,0,0]]'),
        ]]
        for bad in invalid:
            with self.subTest(config=bad), tempfile.TemporaryDirectory() as directory:
                work = Path(directory)
                result = self.emit(bad, work)
                self.assertNotEqual(result.returncode, 0, result.stdout)
                self.assertFalse((work / 'generated').exists())

    def test_phase_initialization_and_per_rotor_parameters(self):
        raw = (ROOT / 'configs/quad_p3.toml').read_text()
        raw = raw.replace('polePairs = 7', 'polePairs = [3,5,7,9]').replace(
            'trapezoidal = true', 'trapezoidal = [true,false,true,false]').replace(
            'theta0 = 0.0', 'theta0 = [-0.2,0.0,0.1,0.3]').replace(
            'omega_start = 0.0', 'omega_start = 0.0\ni_start = [[1,-1,0],[0,0,0],[-2,1,1],[0,0,0]]')
        with tempfile.TemporaryDirectory() as directory:
            work = Path(directory)
            result = self.emit(raw, work)
            self.assertEqual(result.returncode, 0, result.stderr)
            manifest = json.loads(next((work / 'generated').glob('*.manifest.json')).read_text())
            self.assertEqual(manifest['config']['motor']['i_start'][2], [-2, 1, 1])
            self.assertEqual(manifest['config']['motor']['polePairs'], [3, 5, 7, 9])

    def test_examples_ports_manifest_and_shared_bus(self):
        for vehicle, n in [('quad', 4), ('hex', 6)]:
            for level in [3, 4]:
                for profile in ['plant', 'vio']:
                    with self.subTest(vehicle=vehicle, level=level, profile=profile), tempfile.TemporaryDirectory() as directory:
                        work = Path(directory)
                        result = self.emit((ROOT / f'configs/{vehicle}_p{level}.toml').read_text(), work, profile)
                        self.assertEqual(result.returncode, 0, result.stderr)
                        manifest = json.loads(next((work / 'generated').glob('*.manifest.json')).read_text())
                        self.assertEqual(manifest['physics']['propulsion'], f'P{level}')
                        self.assertEqual(manifest['propulsion_contract']['current_states_per_rotor'], 2)
                        self.assertEqual(manifest['event_requirements']['pwm_switching'], level == 4)
                        self.assertTrue(manifest['propulsion_contract']['source_can_absorb_energy'])
                        self.assertEqual(manifest['verification']['fmu_simulation'], 'not_run')
                        if profile == 'vio':
                            self.assertEqual(manifest['input_adapter']['command_meaning'], 'high_side_pwm_duty')
                        source = next((work / 'generated').glob('*.mo')).read_text()
                        self.assertEqual(source.count('Systems.Power.DCLink bus'), 1)
                        for i in range(1, n + 1):
                            self.assertIn(f'connect(bus.p, escs[{i}].dc_p)', source)
                            self.assertIn(f'connect(escs[{i}].phase, motors[{i}].phase)', source)
                        self.assertIn(f'phaseCurrent[{n},3]', source)
                        self.assertNotIn('motors.currentZero', source)
                        self.assertNotIn('motors.equivalentVoltage', source)
                        self.assertNotIn('motorCurrent[', source)

    @unittest.skipUnless(os.environ.get('FIRE_TEST_OMC') == '1', 'set FIRE_TEST_OMC=1')
    def test_quad_hexa_profiles_mapping_power_and_cutoff(self):
        for vehicle in ['quad', 'hex']:
            for level in [3, 4]:
                for profile in ['plant', 'vio']:
                    with self.subTest(vehicle=vehicle, level=level, profile=profile), tempfile.TemporaryDirectory() as directory:
                        work = Path(directory)
                        raw = (ROOT / f'configs/{vehicle}_p{level}.toml').read_text() + '\n[landing_gear]\nenabled=false\n'
                        result = self.emit(raw, work, profile)
                        self.assertEqual(result.returncode, 0, result.stderr)
                        generated = work / 'generated'
                        manifest = json.loads(next(generated.glob('*.manifest.json')).read_text())
                        model = manifest['model']
                        mapping = manifest['interface']['actuatorIndex']
                        channels = manifest['interface']['nActuators']
                        commands = [0.25 + 0.04*i for i in range(channels)]
                        values = [1100 + 800*u for u in commands] if profile == 'vio' else commands
                        off = 1100 if profile == 'vio' else 0
                        port = 'pwm' if profile == 'vio' else 'normalizedCommand'
                        harness = work / 'P34VehicleRun.mo'
                        harness.write_text('model P34VehicleRun\n  ' + model + ' vehicle(' + port +
                            '=if time<0.006 then {' + ','.join(map(str, values)) + '} else fill(' +
                            str(off) + ',' + str(channels) + '));\n' +
                            '  Real netEnergy(start=0,fixed=true);\n  Real error;\nequation\n' +
                            '  der(netEnergy)=vehicle.sourcePower-vehicle.busLoss-sum(vehicle.escLoss)'
                            '-sum(vehicle.copperLoss)-sum(vehicle.frictionLoss)-sum(vehicle.loadPower);\n'
                            '  error=netEnergy-(sum(vehicle.storedEnergy)+vehicle.busStoredEnergy-0.256);\n'
                            '  assert(abs(error)<1e-4,"Vehicle bus/motor energy balance");\n'
                            'end P34VehicleRun;\n')
                        mos = work / 'run.mos'
                        paths = [generated / 'sources/ModelicaCPSLibrary/package.mo', generated / (model + '.mo'), harness]
                        mos.write_text('loadModel(Modelica,{"4.0.0"});\n' + ''.join(
                            'loadFile(' + json.dumps(str(p)) + ');\n' for p in paths) +
                            'simulate(P34VehicleRun,stopTime=0.025,tolerance=1e-8,numberOfIntervals=2500,'
                            'outputFormat="csv",variableFilter="vehicle.(phaseCurrent.*|rotorSpeed.*|busVoltage|escs.*control.command|escs.*coast)|error");\ngetErrorString();\n')
                        result = subprocess.run(['omc', str(mos)], cwd=work, capture_output=True, text=True, timeout=120)
                        self.assertIn('The simulation finished successfully', result.stdout, result.stdout + result.stderr)
                        self.assertNotIn('Warning:', result.stdout)
                        with (work / 'P34VehicleRun_res.csv').open() as stream:
                            rows = list(csv.DictReader(stream))
                        powered = min(rows, key=lambda row: abs(float(row['time']) - 0.005))
                        final = rows[-1]
                        for i, channel in enumerate(mapping, 1):
                            self.assertAlmostEqual(float(powered[f'vehicle.escs[{i}].control.command']), commands[channel-1])
                            self.assertGreater(float(powered[f'vehicle.rotorSpeed[{i}]']), 0)
                            self.assertAlmostEqual(sum(float(final[f'vehicle.phaseCurrent[{i},{j}]']) for j in range(1, 4)), 0, delta=1e-7)
                            self.assertEqual(float(final[f'vehicle.escs[{i}].coast']), 1)


if __name__ == '__main__':
    unittest.main()
