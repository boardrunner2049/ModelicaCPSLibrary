within ModelicaCPSLibrary.Tests;

model ChassisMassProperties "Independent full-tensor mass composition and aggregate-mode checks"
  ModelicaCPSLibrary.Physical.Mechanical.Chassis.ChassisAssembly assembled(
    useAssembledMass = true,
    core(mass = 2, r_C = {1, -2, 3}, inertia = diagonal({2, 3, 4})),
    nArms = 1,
    arms = {ModelicaCPSLibrary.Physical.Mechanical.Chassis.Arms.RigidArm(
      mass = 1, r_C = {3, -1, 4},
      R_Cj = [0, -1, 0; 1, 0, 0; 0, 0, 1],
      inertia = diagonal({0.1, 0.2, 0.3}))},
    nPayloads = 1,
    payloads = {ModelicaCPSLibrary.Physical.Mechanical.Chassis.Payloads.FixedPayload(
      mass = 1, r_C = {-1, -3, 2},
      inertia = [0.4, 0.05, 0; 0.05, 0.5, 0; 0, 0, 0.6])});
  ModelicaCPSLibrary.Physical.Mechanical.Chassis.ChassisAssembly aggregateOnly(
    useAssembledMass = false,
    aggregate(mass = 7, r_C = {4, 5, 6}, inertia = diagonal({1, 2, 3})),
    core(mass = 900), nPayloads = 1,
    payloads = {ModelicaCPSLibrary.Physical.Mechanical.Chassis.Payloads.FixedPayload(mass = 800)});
  ModelicaCPSLibrary.Physical.Mechanical.Chassis.ChassisAssembly coreOnly(
    useAssembledMass = true, nArms = 0, nPayloads = 0, nAdditionalParts = 0,
    core(mass = 2, r_C = {0.2, 0.3, 0.4}, inertia = diagonal({0.1, 0.2, 0.3})));
  parameter Real expectedInertia[3, 3] = [6.6, -3.95, -4; -3.95, 13.6, -2; -4, -2, 14.9];
  output Real totalMass = assembled.mass;
initial equation
  assert(abs(assembled.mass - 4) < 1e-12, "Assembled mass was not counted once per part");
  assert(sum(abs(assembled.cg_C - {1, -2, 3})) < 1e-12, "Incorrect combined centre of mass");
  for j in 1:3 loop
    for k in 1:3 loop
      assert(abs(assembled.inertia[j, k] - expectedInertia[j, k]) < 1e-10,
        "Incorrect rotated/parallel-axis full inertia tensor");
    end for;
  end for;
  assert(abs(aggregateOnly.mass - 7) < 1e-12,
    "Aggregate mode must not add constituent mass a second time");
  assert(sum(abs(aggregateOnly.cg_C - {4, 5, 6})) < 1e-12,
    "Aggregate centre of mass changed");
  assert(abs(coreOnly.mass - 2) < 1e-12 and sum(abs(coreOnly.cg_C - {0.2, 0.3, 0.4})) < 1e-12,
    "Empty arm/payload/additional arrays must retain the core properties");
  assert(abs(coreOnly.inertia[1, 1] - 0.1) < 1e-12 and abs(coreOnly.inertia[2, 2] - 0.2) < 1e-12,
    "Core inertia was translated despite being the only part");
  annotation(experiment(StopTime = 0.01));
end ChassisMassProperties;
