within ModelicaCPSLibrary.Physical.Mechanical.Aerodynamics.Blades;
model QuadraticBlade "Static thrust and aerodynamic reaction moment at the rotor hub"
  parameter Real kT(unit = "N.s2") = 1e-5 "Thrust divided by squared angular speed";
  parameter Real kQ(unit = "N.m.s2") = 1e-7 "Torque divided by squared angular speed";
  parameter Integer spinSign = 1 "Positive rotation uses right-hand rule about rotor +z";
  input Modelica.Units.SI.AngularVelocity omega "Nonnegative rotor speed magnitude";
  input Modelica.Units.SI.Velocity airVelocity_r[3]
    "Hub velocity relative to air; unused by this static model";
  input Modelica.Units.SI.AngularVelocity omegaBody_r[3]
    "Carrier angular velocity; unused by this static model";
  output Modelica.Units.SI.Force force_r[3];
  output Modelica.Units.SI.Torque loadTorque "Nonnegative resisting shaft torque magnitude";
  output Modelica.Units.SI.Torque moment_r[3]
    "Aerodynamic hub moment; does not include moment arm or rotor inertia reaction";
initial equation
  assert(kT >= 0 and kQ >= 0, "Quadratic blade coefficients must be nonnegative");
  assert(abs(spinSign) == 1, "Rotor spinSign must be +1 or -1");
equation
  assert(omega >= -1e-8, "QuadraticBlade requires nonnegative speed; reverse rotation is unsupported");
  force_r = {0, 0, kT * omega ^ 2};
  loadTorque = kQ * omega ^ 2;
  moment_r = {0, 0, -spinSign * loadTorque};
end QuadraticBlade;
