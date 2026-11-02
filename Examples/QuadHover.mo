within ModelicaCPSLibrary.Examples;
model QuadHover "Open-loop force balance with initialized rotor speed; no flight controller"
  parameter Vehicles.Copter.Geometry geometry=Vehicles.Copter.Presets.QuadX();
  Vehicles.Copter.MultirotorWithSensors vehicle(geometry=geometry);
  Physical.Electrical.Drives.ESC.PassThrough escs[vehicle.geometry.nRotors];
  Systems.Propulsion.Motors.FirstOrderSpeed motors[vehicle.geometry.nRotors](each omegaMax=1000,each tau=0.03,
    omega_start={sqrt(vehicle.chassis.mass*9.80665/(vehicle.geometry.nRotors*rotors[i].kT)) for i in 1:vehicle.geometry.nRotors});
  Physical.Mechanical.Aerodynamics.Blades.QuadraticBlade rotors[vehicle.geometry.nRotors](
    each kT=1e-5,each kQ=1.5e-7,spinSign=vehicle.geometry.spinSign);
equation
  rotors.omega=motors.omega;
  rotors.airVelocity_r=vehicle.rotorAirVelocity_r;
  rotors.omegaBody_r=vehicle.rotorOmegaBody_r;
  vehicle.rotorForce_r=rotors.force_r;
  vehicle.rotorMoment_r=rotors.moment_r;
  motors.demand=escs.normalizedCommand;
  escs.escCommand={sqrt(vehicle.chassis.mass*9.80665/(geometry.nRotors*rotors[i].kT))/motors[i].omegaMax for i in 1:geometry.nRotors};
  annotation(experiment(StopTime=2,Tolerance=1e-7,Interval=0.005));
end QuadHover;
