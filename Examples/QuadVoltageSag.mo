within ModelicaCPSLibrary.Examples;
model QuadVoltageSag "P1 supply drop reduces speed and thrust at a fixed ESC command"
  parameter Real hoverCommand=sqrt(1.5*9.80665/(4*1e-5))/1000;
  Vehicles.Copter.MultirotorWithSensors vehicle(
    geometry=Vehicles.Copter.Presets.QuadX(),landingGear(enabled=false));
  output Real dcVoltage(unit="V");
  output Real altitude(unit="m");
  Physical.Electrical.Drives.ESC.AverageVoltage escs[vehicle.geometry.nRotors];
  Systems.Propulsion.Motors.FirstOrderVoltageToSpeed motors[vehicle.geometry.nRotors](each Kv=62.5,
    each tauOmega=0.03,each omega_start=1000*hoverCommand);
  Physical.Mechanical.Aerodynamics.Blades.QuadraticBlade rotors[vehicle.geometry.nRotors](
    each kT=1e-5,each kQ=1.5e-7,spinSign=vehicle.geometry.spinSign);
equation
  rotors.omega=motors.omega;
  rotors.airVelocity_r=vehicle.rotorAirVelocity_r;
  rotors.omegaBody_r=vehicle.rotorOmegaBody_r;
  vehicle.rotorForce_r=rotors.force_r;
  vehicle.rotorMoment_r=rotors.moment_r;
  dcVoltage=if time<1 then 16 else 12;
  escs.dcVoltage=fill(dcVoltage,vehicle.geometry.nRotors);
  escs.escCommand=fill(hoverCommand,vehicle.geometry.nRotors);
  motors.equivalentVoltage=escs.equivalentVoltage;
  altitude=-vehicle.truth.p_w[3];
  annotation(experiment(StopTime=2,Tolerance=1e-8,Interval=0.002));
end QuadVoltageSag;
