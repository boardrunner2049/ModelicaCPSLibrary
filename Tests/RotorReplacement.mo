within ModelicaCPSLibrary.Tests;
model RotorReplacement "Independent rotor replacement preserves the vehicle hub-wrench boundary"
  model ReducedThrustRotor
    extends Physical.Mechanical.Aerodynamics.Blades.QuadraticBlade(kT=0.5e-5);
  end ReducedThrustRotor;
  Vehicles.Copter.MultirotorPlant vehicle(gravity_w=zeros(3));
  ReducedThrustRotor rotors[vehicle.geometry.nRotors](spinSign=vehicle.geometry.spinSign);
equation
  rotors.omega=fill(100,vehicle.geometry.nRotors);
  rotors.airVelocity_r=vehicle.rotorAirVelocity_r;
  rotors.omegaBody_r=vehicle.rotorOmegaBody_r;
  vehicle.rotorForce_r=rotors.force_r;
  vehicle.rotorMoment_r=rotors.moment_r;
  assert(abs(vehicle.force_b[3]+0.2)<1e-10,"Replacement blade coefficient did not reach the common vehicle");
  annotation(experiment(StopTime=0.05,Tolerance=1e-9,Interval=0.001));
end RotorReplacement;
