#!/usr/bin/env python3
"""P3/P4 CCM comparison, PWM ripple and native integration-step convergence (not FMI)."""
import argparse
import csv
import json
import math
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[1]


def require(condition, message):
    if not condition:
        raise AssertionError(message)


def run(output, omc):
    output.mkdir(parents=True, exist_ok=True)
    (output / 'report.json').unlink(missing_ok=True)
    script = output / 'build.mos'
    script.write_text('loadModel(Modelica,{"4.0.0"});\n'
        + 'loadFile(' + json.dumps(str(ROOT / 'package.mo')) + ');\n'
        + 'buildModel(ModelicaCPSLibrary.Tests.ThreePhaseAveraging,stopTime=0.05,'
        'numberOfIntervals=50000,tolerance=1e-9,outputFormat="csv",fileNamePrefix="Averaging",'
        'variableFilter="mean.*|switched.phaseCurrent.*|average.energyError|switched.energyError");\ngetErrorString();\n')
    binary = output / 'Averaging'
    binary.unlink(missing_ok=True)
    build = subprocess.run([omc, str(script)], cwd=output, capture_output=True, text=True, timeout=90)
    (output / 'build.log').write_text(build.stdout + build.stderr)
    require(build.returncode == 0 and 'Error:' not in build.stdout and binary.is_file(),
            'Build failed: ' + str(output / 'build.log'))
    cases = []
    for name, step, override in [('coarse', 5e-6, None), ('fine', 2.5e-6, None),
                                  ('off_conductance', 2.5e-6,
                                   'average.esc.Goff=2e-5,switched.esc.Goff=2e-5')]:
        result_file = output / (name + '.csv')
        result_file.unlink(missing_ok=True)
        command = [str(binary), '-maxStepSize=' + str(step), '-stepSize=1e-6', '-r=' + str(result_file)]
        if override:
            command.append('-override=' + override)
        process = subprocess.run(command, cwd=output, capture_output=True, text=True, timeout=120)
        log = process.stdout + process.stderr
        (output / (name + '.log')).write_text(log)
        require(process.returncode == 0 and 'The simulation finished successfully' in log
                and 'LOG_ASSERT' not in log and result_file.exists(), 'Run failed: ' + name)
        with result_file.open() as stream:
            rows = list(csv.DictReader(stream))
        last = rows[-1]
        require(abs(float(last['time']) - 0.05) < 1e-10, 'Incomplete averaging window')
        metrics = {'case': name, 'max_internal_step_s': step, 'pwm_frequency_Hz': 10000,
                   'Goff_S': 2e-5 if override else 1e-5}
        for field in ['meanCurrentAverage', 'meanCurrentSwitched', 'meanPowerAverage', 'meanPowerSwitched']:
            value = float(last[field])
            require(math.isfinite(value), 'Non-finite result: ' + field)
            metrics[field] = value
        for quantity in ['Current', 'Power']:
            a, b = metrics['mean' + quantity + 'Average'], metrics['mean' + quantity + 'Switched']
            metrics[quantity.lower() + '_relative_error'] = abs(a-b) / abs(a)
            require(metrics[quantity.lower() + '_relative_error'] < 0.01, 'CCM means differ by over 1%')
        # Last full PWM period, avoiding the endpoint of the next period.
        samples = [float(row['switched.phaseCurrent[3]']) for row in rows
                   if 0.0498 <= float(row['time']) < 0.0499]
        metrics['current_ripple_peak_to_peak_A'] = max(samples)-min(samples)
        require(metrics['current_ripple_peak_to_peak_A'] > 0.1, 'P4 did not resolve PWM ripple')
        metrics['max_energy_error_J'] = max(abs(float(row[field])) for row in rows
            for field in ['average.energyError', 'switched.energyError'])
        require(metrics['max_energy_error_J'] < 1e-4, 'Energy balance exceeded tolerance')
        cases.append(metrics)
    coarse, fine, leakage = cases
    for field in ['meanCurrentSwitched', 'meanPowerSwitched', 'current_ripple_peak_to_peak_A']:
        require(abs(coarse[field]-fine[field])/abs(fine[field]) < 0.001,
                'Step refinement changed ' + field + ' by over 0.1%')
    for field in ['meanCurrentSwitched', 'meanPowerSwitched']:
        require(abs(leakage[field]-fine[field])/abs(fine[field]) < 0.001,
                'Off-conductance sensitivity exceeds 0.1% in benchmark')
    report = {'target': 'OpenModelica native DASSL; not FMI',
              'compiler': subprocess.check_output([omc, '--version'], text=True).strip(),
              'benchmark': 'Locked shaft, duty 0.6, 16 V resistive source, 10 kHz, deadTime=0',
              'mean_window_s': [0.04, 0.05], 'cases': cases,
              'limits': 'CCM benchmark only; no universal DCM, commutation or FMI certification'}
    (output / 'report.json').write_text(json.dumps(report, indent=2) + '\n')
    print('PASS: P3/P4 CCM means, PWM ripple, step refinement and off-conductance sensitivity')
    print(output / 'report.json')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--omc', default='omc')
    parser.add_argument('--output', type=Path, default=ROOT / 'build/p34-validation')
    args = parser.parse_args()
    run(args.output.resolve(), args.omc)
