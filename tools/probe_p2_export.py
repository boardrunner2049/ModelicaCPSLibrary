#!/usr/bin/env python3
"""Build P2 FMI 2 CS, exercise coast/restart and compare two step sizes to native DASSL."""
import argparse
import csv
import json
from pathlib import Path
import subprocess
import tomllib

from probe_export import Fmi2CS, ROOT, require


def command(t, n):
    if t < 0.1:
        return [0.4+0.03*i for i in range(n)]
    if t < 0.22:
        return [0.0]*n
    if t < 0.28:
        return [0.1]*n
    if t < 0.4:
        return [0.7+0.01*i for i in range(n)]
    return [0.0]*n


def run(output, omc):
    output.mkdir(parents=True, exist_ok=True)
    (output/'report.json').unlink(missing_ok=True)
    subprocess.run(['cargo','build','--locked','--offline','--manifest-path',
                    str(ROOT/'tools/fire-compose/Cargo.toml')],check=True)
    reports={}
    for config_name in ['quad_p2.toml','hex_p2.toml']:
        work=output/config_name.removesuffix('.toml')
        work.mkdir(parents=True, exist_ok=True)
        raw=(ROOT/'configs'/config_name).read_text()
        config=tomllib.loads(raw)
        model=config['model_name']
        channels=config['geometry']['nActuators']
        rotors=len(config['geometry']['actuatorIndex'])
        source=work/'physics.toml'
        source.write_text(raw+'\n[landing_gear]\nenabled=false\n')
        generated=work/'generated'
        subprocess.run([str(ROOT/'tools/fire-compose/target/debug/fire-compose'),'emit',
                        str(source),'--profile','vio','--output-dir',str(generated)],check=True,
                       stdout=subprocess.DEVNULL)
        pwm_min,pwm_span=1100,800
        vectors=['{'+','.join(str(pwm_min+pwm_span*u) for u in command(t,channels))+'}'
                 for t in [0,0.15,0.25,0.3,0.45]]
        harness=work/'P2Reference.mo'
        harness.write_text('model P2Reference\n  '+model+' vehicle(pwm=if time<0.1 then '+vectors[0]+
            ' else if time<0.22 then '+vectors[1]+' else if time<0.28 then '+vectors[2]+
            ' else if time<0.4 then '+vectors[3]+' else '+vectors[4]+');\nend P2Reference;\n')
        mos=work/'build.mos'
        mos.write_text('loadModel(Modelica,{"4.0.0"});\n'+''.join('loadFile('+json.dumps(str(p))+');\n'
            for p in [generated/'sources/ModelicaCPSLibrary/package.mo',generated/(model+'.mo'),harness])+
            'simulate(P2Reference,stopTime=0.5,tolerance=1e-10,numberOfIntervals=10000,outputFormat="csv");\ngetErrorString();\n'+
            f'buildModelFMU({model},version="2.0",fmuType="cs",fileNamePrefix="P2",platforms={{"static"}});\ngetErrorString();\n')
        fmu=work/'P2.fmu'
        fmu.unlink(missing_ok=True)
        (work/'P2Reference_res.csv').unlink(missing_ok=True)
        process=subprocess.run([omc,str(mos)],cwd=work,capture_output=True,text=True,timeout=300)
        log=process.stdout+process.stderr
        (work/'build.log').write_text(log)
        require(process.returncode==0 and 'Error:' not in log and fmu.exists()
                and 'The simulation finished successfully' in log,'P2 export/reference failed: '+str(work/'build.log'))
        with (work/'P2Reference_res.csv').open() as stream:
            reference=list(csv.DictReader(stream))
        sample_times=[0.08,0.1001,0.12,0.21,0.25,0.32,0.45,0.5]
        expected={t:min(reference,key=lambda row:abs(float(row['time'])-t)) for t in sample_times}
        cases=[]
        for dt in [0.0001,0.00005]:
            host=Fmi2CS(fmu,work/f'unpacked-{dt}',channels,n_rotors=rotors)
            for field in ['motorCurrent','dcCurrent','electricalPower','storedEnergy','copperLoss']:
                for i in range(1,rotors+1):
                    name=f'{field}[{i}]'
                    require(name in host.variables and host.variables[name].get('causality')=='output',
                            'Missing P2 diagnostic '+name)
                    host.observables.append(name)
            try:
                host.initialize([pwm_min+pwm_span*u for u in command(0,channels)],0.501)
                samples={}
                ticks={round(t/dt):t for t in sample_times}
                changes={round(t/dt):t for t in [0.1,0.22,0.28,0.4]}
                for tick in range(round(0.5/dt)):
                    if tick in changes:
                        host.set_inputs([pwm_min+pwm_span*u for u in command(changes[tick],channels)])
                    host.step(tick*dt,dt)
                    if tick+1 in ticks:
                        t=ticks[tick+1]
                        samples[t]=host.snapshot()
                speed_error=current_error=0.0
                for t, values in samples.items():
                    for i in range(1,rotors+1):
                        w=values[f'rotorSpeed[{i}]']
                        current=values[f'motorCurrent[{i}]']
                        speed_error=max(speed_error,abs(w-float(expected[t][f'vehicle.rotorSpeed[{i}]'])))
                        current_error=max(current_error,abs(current-float(expected[t][f'vehicle.motorCurrent[{i}]'])))
                        require(current>=-1e-5 and values[f'dcCurrent[{i}]']>=-1e-5,'Reverse FMU current')
                        if t in [0.12,0.21,0.25,0.45,0.5]:
                            require(abs(current)<1e-5 and abs(values[f'electricalPower[{i}]'])<1e-5,
                                    f'FMU failed to coast at {t}: {current}')
                        if t==0.32:
                            require(current>0,'FMU failed to restart')
                cases.append({'step_s':dt,'max_speed_error_rad_s':speed_error,
                              'max_current_error_A':current_error})
            finally:
                host.close()
        coarse,fine=cases
        require(fine['max_speed_error_rad_s']<0.5 and fine['max_current_error_A']<0.1,
                'P2 FMI2 error exceeds tolerance: '+str(cases))
        for key in ['max_speed_error_rad_s','max_current_error_A']:
            require(fine[key]<=0.8*coarse[key]+1e-7,'Step refinement did not converge: '+str(cases))
        reports[config_name]={'channels':channels,'rotors':rotors,'steps':cases}
    report={'target':'OpenModelica FMI 2 Co-Simulation','compiler':subprocess.check_output([omc,'--version'],text=True).strip(),
            'reference':'native DASSL, tolerance 1e-10','metrics':reports}
    (output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    print('PASS: P2 quad/hexa FMI2 current, coast, restart and step convergence')
    print(output/'report.json')


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--omc',default='omc')
    parser.add_argument('--output',type=Path,default=ROOT/'build/p2-export-validation')
    args=parser.parse_args()
    run(args.output.resolve(),args.omc)
