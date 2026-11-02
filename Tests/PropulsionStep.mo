within ModelicaCPSLibrary.Tests;
model PropulsionStep "Analytical speed step, opposite yaw torques, and demand limits"
  ModelicaCPSLibrary.Physical.Mechanical.Aerodynamics.Blades.QuadraticBlade ccw(
    kT = 1e-3, kQ = 2e-5, spinSign = 1);
  ModelicaCPSLibrary.Physical.Mechanical.Aerodynamics.Blades.QuadraticBlade cw(
    kT = 1e-3, kQ = 2e-5, spinSign = -1);
  ModelicaCPSLibrary.Systems.Propulsion.Motors.FirstOrderSpeed saturated(
    omegaMax = 100, tau = 0.05);
  ModelicaCPSLibrary.Systems.Propulsion.Motors.FirstOrderSpeed stopped(
    omegaMax = 100, tau = 0.05);
  Physical.Electrical.Drives.ESC.PassThrough esc;
  Systems.Propulsion.Motors.FirstOrderSpeed motor(omegaMax=100,tau=0.05);
  Real expectedSpeed;
equation
  esc.escCommand=0.5;
  motor.demand=esc.normalizedCommand;
  ccw.omega=motor.omega;
  cw.omega=motor.omega;
  ccw.airVelocity_r = zeros(3);
  cw.airVelocity_r = zeros(3);
  ccw.omegaBody_r = zeros(3);
  cw.omegaBody_r = zeros(3);
  saturated.demand = 2;
  stopped.demand = -1;
  expectedSpeed = 50 * (1 - exp(-time / 0.05));
  assert(abs(ccw.omega - expectedSpeed) < 1e-5 and
    abs(cw.omega - expectedSpeed) < 1e-5, "Rotor speed step response is incorrect");
  assert(abs(ccw.force_r[3] - 1e-3 * expectedSpeed ^ 2) < 1e-5,
    "Thrust must follow actual rotor speed, not command");
  assert(abs(ccw.moment_r[3] + 2e-5 * expectedSpeed ^ 2) < 1e-6 and
    abs(cw.moment_r[3] - 2e-5 * expectedSpeed ^ 2) < 1e-6,
    "Aerodynamic yaw torque must oppose spin direction");
  assert(max(abs(ccw.moment_r + cw.moment_r)) < 1e-9,
    "Oppositely spinning identical rotors must cancel aerodynamic yaw torque");
  assert(abs(saturated.omega - 2 * expectedSpeed) < 2e-5 and abs(stopped.omega) < 1e-10,
    "Normalized speed demand must be limited to [0,1]");
  annotation(experiment(StopTime = 0.5, Tolerance = 1e-10, Interval = 0.005));
end PropulsionStep;
