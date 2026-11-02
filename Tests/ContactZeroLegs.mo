within ModelicaCPSLibrary.Tests;

model ContactZeroLegs "A vehicle may omit landing contacts independently of rotor count"
  ModelicaCPSLibrary.Physical.Mechanical.Chassis.LandingGear.LandingGearAssembly gear(nLegs = 0);
equation
  gear.p_w = zeros(3);
  gear.v_b = zeros(3);
  gear.omega_b = zeros(3);
  gear.R_wb = identity(3);
  gear.terrainPoint_w = zeros(3);
  gear.terrainNormal_w = {0, 0, -1};
  gear.terrainVelocity_w = zeros(3);
  assert(sum(abs(gear.force_b)) + sum(abs(gear.moment_b)) < 1e-12,
    "A zero-leg assembly must return zero force and moment");
  annotation(experiment(StopTime = 0.01));
end ContactZeroLegs;
