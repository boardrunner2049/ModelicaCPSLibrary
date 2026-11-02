within ModelicaCPSLibrary.Systems.Propulsion;
model HubReaction "Aerodynamic hub moment plus axial spin-acceleration reaction"
  parameter Modelica.Units.SI.Inertia J;
  parameter Modelica.Units.SI.Inertia JLoad = 0;
  parameter Integer spinSign = 1;
  input Modelica.Units.SI.Torque aerodynamicMoment_r[3];
  input Modelica.Units.SI.AngularAcceleration acceleration;
  output Modelica.Units.SI.Torque moment_r[3];
initial equation
  assert(J > 0 and JLoad >= 0 and abs(spinSign) == 1, "Invalid hub reaction parameters");
equation
  moment_r = aerodynamicMoment_r-{0,0,spinSign*(J+JLoad)*acceleration};
  annotation(Documentation(info="<html><p>Fixed-carrier axial approximation. Do not also add the motor housing reaction. The vehicle applies the mount rotation and CG moment arm once. No gyroscopic or carrier angular-acceleration feedback is included.</p></html>"));
end HubReaction;
