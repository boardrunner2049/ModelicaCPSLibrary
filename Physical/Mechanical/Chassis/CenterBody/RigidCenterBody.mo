within ModelicaCPSLibrary.Physical.Mechanical.Chassis.CenterBody;

record RigidCenterBody "Central frame mass; motion is owned by the vehicle rigid body"
  extends ModelicaCPSLibrary.Physical.Mechanical.MassProperties(
    mass = 1,
    inertia = diagonal({0.01, 0.01, 0.01}));
end RigidCenterBody;
