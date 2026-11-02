"""P2 parameter contracts and composed electromechanical behavior."""
import csv
import json
import math
import os
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
BIN = ROOT / 'tools/fire-compose/target/debug/fire-compose'


class P2ComposerTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        subprocess.run(['cargo', 'build', '--locked', '--offline', '--manifest-path',
                        str(ROOT/'tools/fire-compose/Cargo.toml')], check=True)

    def emit(self, raw, work, profile='plant'):
        config=work/'vehicle.toml'
        config.write_text(raw)
        return subprocess.run([str(BIN),'emit',str(config),'--profile',profile,
            '--output-dir',str(work/'generated')],capture_output=True,text=True)

    def test_explicit_parameters_and_compatible_ports(self):
        raw=(ROOT/'configs/quad_p2.toml').read_text()
        invalid=[raw.replace(line, '') for line in ['R = 0.2', 'L = 0.001', 'k = 0.02',
                 'J = 0.0001', 'dcVoltage = 16.0']]
        invalid += [raw.replace('L = 0.001', 'L = '+value)
                    for value in ['0', '-1', 'nan', 'inf', 'true', '[0.001,0.002]']]
        invalid += [raw.replace('i_start = 0.0','i_start = -1'),
                    raw.replace('omega_start = 0.0','omega_start = -1'),
                    raw.replace('b = 0.0','b = -0.1'),
                    raw.replace('R = 0.2','R = 0.2\nKv = 50'),
                    raw.replace('J = 0.0001','J = -1'),
                    raw.replace('average_unidirectional','average_voltage'),
                    raw.replace('average_unidirectional','pass_through'),
                    raw.replace('dynamic_dc_equivalent','first_order_voltage_to_speed')]
        for bad in invalid:
            with self.subTest(raw=bad), tempfile.TemporaryDirectory() as directory:
                work=Path(directory)
                run=self.emit(bad,work)
                self.assertNotEqual(run.returncode,0,run.stdout)
                self.assertFalse((work/'generated').exists())
        for profile in ['plant','vio']:
            with self.subTest(profile=profile), tempfile.TemporaryDirectory() as directory:
                work=Path(directory)
                run=self.emit(raw,work,profile)
                self.assertEqual(run.returncode,0,run.stderr)
                manifest=json.loads(next((work/'generated').glob('*.manifest.json')).read_text())
                self.assertEqual(manifest['physics']['propulsion'],'P2')
                self.assertTrue(manifest['propulsion_contract']['load_feedback'])
                self.assertTrue(manifest['propulsion_contract']['power_accounting'])
                self.assertTrue(manifest['event_requirements']['motor_current_zero_crossing'])
                self.assertFalse(manifest['propulsion_contract']['gyroscopic_coupling'])
                source=next((work/'generated').glob('*.mo')).read_text()
                self.assertIn('connect(escs.p, motors.p)',source)
                self.assertIn('plant.rotorMoment_r = reactions.moment_r',source)
                self.assertIn('motors.loadTorque = rotors.loadTorque',source)
                self.assertIn('escs.openCircuitVoltage = motors.backEMF',source)
                self.assertIn('motors.currentZero = escs.coast',source)
                self.assertNotIn('tauOmega=',source)
                self.assertTrue((work/'generated/sources/ModelicaCPSLibrary/Physical/Electrical/Passives/SeriesRL.mo').is_file())

    @unittest.skipUnless(os.environ.get('FIRE_TEST_OMC')=='1','set FIRE_TEST_OMC=1')
    def test_composed_arrays_mapping_equilibrium_and_cutoff(self):
        for preset, channels, mapping in [('QuadX',4,[3,2,4,1]),('HexaX',8,[1,2,3,4,6,8])]:
            n=len(mapping)
            resistance=[0.1+0.02*i for i in range(n)]
            constant=[0.015+0.001*i for i in range(n)]
            inductance=[0.001+0.0001*i for i in range(n)]
            inertia=[0.0001+0.00001*i for i in range(n)]
            load_inertia=[0.00002+0.00001*i for i in range(n)]
            kq=[1e-7+1e-8*i for i in range(n)]
            raw=(f'schema_version=3\nmodel_name="P2Vehicle"\n[geometry]\npreset="{preset}"\n'
                 f'nActuators={channels}\nactuatorIndex={mapping}\n[models]\n'
                 'esc="average_unidirectional"\nmotor="dynamic_dc_equivalent"\n'
                 f'[motor]\nR={resistance}\nL={inductance}\nk={constant}\nJ={inertia}\n'
                 f'[rotor]\nJ={load_inertia}\nkQ={kq}\n[power]\ndcVoltage=16\n'
                 '[landing_gear]\nenabled=false\n')
            commands=[0.05+0.07*i for i in range(channels)]
            for profile in ['plant','vio']:
                with self.subTest(preset=preset,profile=profile), tempfile.TemporaryDirectory() as directory:
                    work=Path(directory)
                    run=self.emit(raw,work,profile)
                    self.assertEqual(run.returncode,0,run.stderr)
                    values=[1100+800*u for u in commands] if profile=='vio' else commands
                    port='pwm' if profile=='vio' else 'normalizedCommand'
                    off=1100 if profile=='vio' else 0
                    harness=work/'P2Run.mo'
                    harness.write_text('model P2Run\n  P2Vehicle vehicle('+port+'=if time<2 then {'+
                        ','.join(map(str,values))+'} else fill('+str(off)+','+str(channels)+'));\nend P2Run;\n')
                    files=[work/'generated/sources/ModelicaCPSLibrary/package.mo',work/'generated/P2Vehicle.mo',harness]
                    mos=work/'run.mos'
                    mos.write_text('loadModel(Modelica,{"4.0.0"});\n'+''.join('loadFile('+json.dumps(str(p))+');\n' for p in files)+
                        'simulate(P2Run,stopTime=2.2,tolerance=1e-9,numberOfIntervals=2200,outputFormat="csv");\ngetErrorString();\n')
                    run=subprocess.run(['omc',str(mos)],cwd=work,capture_output=True,text=True,timeout=120)
                    self.assertIn('The simulation finished successfully',run.stdout,run.stdout+run.stderr)
                    with (work/'P2Run_res.csv').open() as stream:
                        rows=list(csv.DictReader(stream))
                    steady=min(rows,key=lambda r:abs(float(r['time'])-1.99))
                    final=rows[-1]
                    for i,ch in enumerate(mapping):
                        r,k,q=resistance[i],constant[i],kq[i]
                        voltage=16*commands[ch-1]
                        expected=(-k+math.sqrt(k*k+4*r*q*voltage/k))/(2*r*q/k)
                        speed=float(steady[f'vehicle.rotorSpeed[{i+1}]'])
                        self.assertAlmostEqual(speed,expected,delta=0.01)
                        current=float(steady[f'vehicle.motorCurrent[{i+1}]'])
                        self.assertAlmostEqual(k*current,q*speed*speed,delta=1e-5)
                        self.assertAlmostEqual(float(steady[f'vehicle.dcCurrent[{i+1}]']),commands[ch-1]*current,delta=1e-6)
                        self.assertAlmostEqual(float(final[f'vehicle.motorCurrent[{i+1}]']),0,delta=1e-6)
                        self.assertLess(float(final[f'vehicle.rotorSpeed[{i+1}]']),speed)
                        self.assertAlmostEqual(float(final[f'vehicle.plant.rotorMoment_r[{i+1},3]']),0,delta=1e-7)


if __name__=='__main__':
    unittest.main()
