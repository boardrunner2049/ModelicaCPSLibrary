within ModelicaCPSLibrary.Tests;
model VoltagePropulsion "P1 analytic response, P0 equivalence, voltage sag and coast-down"
  Systems.Propulsion.Motors.FirstOrderSpeed p0(tau=0.03,omegaMax=1000,omega_start=70);
  Systems.Propulsion.Motors.FirstOrderVoltageToSpeed nominal(Kv=62.5,tauOmega=0.03,omega_start=70);
  Systems.Propulsion.Motors.FirstOrderVoltageToSpeed sag(Kv=62.5,tauOmega=0.03,omega_start=70);
  Systems.Propulsion.Motors.FirstOrderVoltageToSpeed stopped(Kv=62.5,tauOmega=0.03,omega_start=70);
  Physical.Electrical.Drives.ESC.PassThrough p0Esc;
  Physical.Electrical.Drives.ESC.AverageVoltage escs[3];
  Physical.Mechanical.Aerodynamics.Blades.QuadraticBlade rotors[2](each kT=1e-5,each kQ=1.5e-7);
  Real sagExpected(unit="rad/s");
equation
  p0Esc.escCommand=0.4;
  p0.demand=p0Esc.normalizedCommand;
  escs.escCommand={0.4,0.4,1};
  escs.dcVoltage={16,if time<0.15 then 16 else 8,0};
  nominal.equivalentVoltage=escs[1].equivalentVoltage;
  sag.equivalentVoltage=escs[2].equivalentVoltage;
  stopped.equivalentVoltage=escs[3].equivalentVoltage;
  rotors.omega={p0.omega,nominal.omega};
  rotors.airVelocity_r=zeros(2,3);
  rotors.omegaBody_r=zeros(2,3);
  sagExpected=if time<0.15 then 400+(70-400)*exp(-time/0.03)
    else 200+(400+(70-400)*exp(-0.15/0.03)-200)*exp(-(time-0.15)/0.03);
  assert(abs(nominal.omega-(400+(70-400)*exp(-time/0.03)))<1e-4,"P1 analytic response changed");
  assert(abs(nominal.omega-p0.omega)<1e-7,"P1 nominal speed differs from P0");
  assert(max(abs(rotors[2].force_r-rotors[1].force_r))<1e-7 and max(abs(rotors[2].moment_r-rotors[1].moment_r))<1e-7,
    "P1 nominal wrench differs from P0");
  assert(abs(sag.omega-sagExpected)<1e-4,"Voltage sag response or independent state changed");
  assert(abs(stopped.omega-70*exp(-time/0.03))<1e-4,"Zero voltage coast-down changed");
  annotation(experiment(StopTime=0.5,Tolerance=1e-10,Interval=0.001));
end VoltagePropulsion;
