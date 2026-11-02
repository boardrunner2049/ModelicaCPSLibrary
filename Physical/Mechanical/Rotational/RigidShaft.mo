within ModelicaCPSLibrary.Physical.Mechanical.Rotational;
model RigidShaft "Rigid shaft inertia with viscous friction to its support"
  parameter Modelica.Units.SI.Inertia J;
  parameter Modelica.Units.SI.RotationalDampingConstant b = 0;
  parameter Modelica.Units.SI.AngularVelocity omega_start = 0;
  Modelica.Mechanics.Rotational.Interfaces.Flange_a drive;
  Modelica.Mechanics.Rotational.Interfaces.Flange_b load;
  Modelica.Mechanics.Rotational.Interfaces.Support support;
  Modelica.Mechanics.Rotational.Components.Inertia inertia(
    J=J,phi(start=0,fixed=true),w(start=omega_start,fixed=true));
  Modelica.Mechanics.Rotational.Components.Damper friction(d=b);
  output Modelica.Units.SI.AngularVelocity omega;
  output Modelica.Units.SI.AngularAcceleration acceleration;
  output Modelica.Units.SI.Power frictionLoss;
  output Modelica.Units.SI.Energy kineticEnergy;
initial equation
  assert(J > 0 and b >= 0, "RigidShaft requires J > 0 and b >= 0");
equation
  connect(drive,inertia.flange_a);
  connect(load,inertia.flange_b);
  connect(inertia.flange_a,friction.flange_a);
  connect(friction.flange_b,support);
  omega = inertia.w;
  acceleration = inertia.a;
  frictionLoss = friction.lossPower;
  kineticEnergy = J*omega*omega/2;
end RigidShaft;
