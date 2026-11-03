// Harness wrapper for ModelicaCPSLibrary.Physical.Mechanical.Rotational.RigidShaft.
// RigidShaft has no default for J and three acausal flanges (drive, load,
// support); exported alone it is refused ("parameter J has neither a binding
// nor an initial value"). This wrapper gives it J = 2e-3 kg.m2,
// b = 0.01 N.m.s/rad (mechanical time constant J/b = 0.2 s), grounds the
// support on a fixed housing and applies two prescribed torques: a drive
// torque on the drive flange (accelerating positive rotation) and a load
// torque on the load flange (opposing positive rotation).
model FireHarness_RigidShaft
  input Real driveTorque(unit = "N.m", start = 0) "Torque applied to the drive flange";
  input Real loadTorque(unit = "N.m", start = 0) "Torque opposing positive rotation, at the load flange";
  ModelicaCPSLibrary.Physical.Mechanical.Rotational.RigidShaft shaft(J = 2e-3, b = 0.01, omega_start = 0);
  Modelica.Mechanics.Rotational.Sources.Torque drive(useSupport = true);
  Modelica.Mechanics.Rotational.Sources.Torque load(useSupport = true);
  Modelica.Mechanics.Rotational.Components.Fixed housing;
  output Real omega(unit = "rad/s") = shaft.omega;
  output Real acceleration(unit = "rad/s2") = shaft.acceleration;
  output Real frictionLoss(unit = "W") = shaft.frictionLoss;
  output Real kineticEnergy(unit = "J") = shaft.kineticEnergy;
equation
  drive.tau = driveTorque;
  load.tau = -loadTorque;
  connect(drive.flange, shaft.drive);
  connect(load.flange, shaft.load);
  connect(shaft.support, housing.flange);
  connect(drive.support, housing.flange);
  connect(load.support, housing.flange);
end FireHarness_RigidShaft;
