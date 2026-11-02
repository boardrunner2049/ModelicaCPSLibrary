within ModelicaCPSLibrary.Physical.Mechanical.Chassis;

model ChassisAssembly "Select an aggregate budget or assemble rigid component mass properties"
  parameter Boolean useAssembledMass = false
    "False uses aggregate exclusively; true uses constituents exclusively";
  parameter ModelicaCPSLibrary.Physical.Mechanical.MassProperties aggregate(
    mass = 1, inertia = diagonal({0.01, 0.01, 0.01}));
  parameter CenterBody.RigidCenterBody core;
  parameter Integer nArms(min = 0) = 0;
  parameter Arms.RigidArm arms[nArms];
  parameter Integer nPayloads(min = 0) = 0;
  parameter Payloads.FixedPayload payloads[nPayloads];
  parameter Integer nAdditionalParts(min = 0) = 0
    "Motor, battery, sensors, and other masses not already included above";
  parameter ModelicaCPSLibrary.Physical.Mechanical.MassProperties additionalParts[nAdditionalParts];

  final parameter ModelicaCPSLibrary.Physical.Mechanical.MassProperties properties =
    if useAssembledMass then
      Functions.combineMassProperties(cat(1, {core}, arms, payloads, additionalParts))
    else Functions.combineMassProperties({aggregate});
  final parameter Real mass(unit = "kg") = properties.mass;
  final parameter Real cg_C[3](each unit = "m") = properties.r_C;
  final parameter Real inertia[3, 3](each unit = "kg.m2") = properties.inertia
    "Combined inertia about cg_C, resolved in chassis/body axes";
initial equation
  assert(mass > 0, "The selected chassis mass budget must have positive total mass");
  annotation(Documentation(info = "<html><p>All parts are fixed to chassis axes C. Subtract cg_C from every rotor and sensor mount position before passing it to the rigid body assembly. This model has no motion states and applies no gravity. Add every physical part exactly once; aggregate mode never adds the constituent records. fire-compose checks duplicate nonempty componentId values in the configuration before emission and preserves them in the manifest. Direct Modelica callers own duplicate-part bookkeeping.</p></html>"));
end ChassisAssembly;
