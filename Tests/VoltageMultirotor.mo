within ModelicaCPSLibrary.Tests;
model VoltageMultirotor "P0/P1 vehicle equivalence with remapped channels and shared contact"
  parameter Vehicles.Copter.Geometry geometry=Vehicles.Copter.Presets.QuadX(
    actuatorIndex={3,2,4,1},spinSign={-1,1,-1,1});
  Vehicles.Copter.MultirotorWithSensors p0(geometry=geometry,p_start={0,0,-0.12});
  Vehicles.Copter.MultirotorWithSensors p1(geometry=geometry,p_start={0,0,-0.12});
  Physical.Electrical.Drives.ESC.PassThrough p0Esc[4];
  Physical.Electrical.Drives.ESC.AverageVoltage p1Esc[4];
  Systems.Propulsion.Motors.FirstOrderSpeed p0Motor[4](each tau=0.03);
  Systems.Propulsion.Motors.FirstOrderVoltageToSpeed p1Motor[4];
  Physical.Mechanical.Aerodynamics.Blades.QuadraticBlade p0Rotor[4](each kQ=1.5e-7,spinSign=geometry.spinSign);
  Physical.Mechanical.Aerodynamics.Blades.QuadraticBlade p1Rotor[4](each kQ=1.5e-7,spinSign=geometry.spinSign);
  parameter Real command[4]={0.1,0.2,0.3,0.4};
equation
  for i in 1:4 loop
    p0Esc[i].escCommand=command[geometry.actuatorIndex[i]];
    p1Esc[i].escCommand=command[geometry.actuatorIndex[i]];
  end for;
  p1Esc.dcVoltage=fill(16,4);
  p0Motor.demand=p0Esc.normalizedCommand;
  p1Motor.equivalentVoltage=p1Esc.equivalentVoltage;
  p0Rotor.omega=p0Motor.omega;
  p1Rotor.omega=p1Motor.omega;
  p0Rotor.airVelocity_r=p0.rotorAirVelocity_r;
  p1Rotor.airVelocity_r=p1.rotorAirVelocity_r;
  p0Rotor.omegaBody_r=p0.rotorOmegaBody_r;
  p1Rotor.omegaBody_r=p1.rotorOmegaBody_r;
  p0.rotorForce_r=p0Rotor.force_r;
  p1.rotorForce_r=p1Rotor.force_r;
  p0.rotorMoment_r=p0Rotor.moment_r;
  p1.rotorMoment_r=p1Rotor.moment_r;
  assert(max(abs(p0Motor.omega-p1Motor.omega))<1e-6,"Vehicle rotor mapping changed");
  assert(max(abs(p0.force_b-p1.force_b))<1e-4,"P1 total force changed at nominal voltage");
  assert(max(abs(p0.moment_b-p1.moment_b))<1e-4,"P1 hub/CG moment changed");
  assert(max(abs(p0.truth.p_w-p1.truth.p_w))<1e-6,"P1 body motion changed");
  assert(max(abs(p0.measurements.acceleration-p1.measurements.acceleration))<1e-4,"P1 sensing changed");
  annotation(experiment(StopTime=0.2,Tolerance=1e-10,Interval=0.001));
end VoltageMultirotor;
