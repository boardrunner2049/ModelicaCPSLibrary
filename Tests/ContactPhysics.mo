within ModelicaCPSLibrary.Tests;

model ContactPhysics "Inclined contact, friction work, angular tip velocity, and tensile-force rejection"
  ModelicaCPSLibrary.Physical.Mechanical.Chassis.LandingGear.CompliantPointLeg inclined(
    rLeg_b = {0, 0, 0}, stiffness = 1000, damping = 20,
    tangentialDamping = 2, frictionCoefficient = 0.3);
  ModelicaCPSLibrary.Physical.Mechanical.Chassis.LandingGear.CompliantPointLeg pulling(
    rLeg_b = {0, 0, 0}, stiffness = 1000, damping = 20);
  ModelicaCPSLibrary.Physical.Mechanical.Chassis.LandingGear.CompliantPointLeg offset(
    rLeg_b = {1, 0, 0}, stiffness = 1000, damping = 20);
  ModelicaCPSLibrary.Physical.Mechanical.Chassis.LandingGear.CompliantPointLeg separating(
    rLeg_b = {0, 0, 0}, stiffness = 1000, damping = 20);
equation
  inclined.p_w = {0, -0.006, 0.008};
  inclined.v_b = {2, 0, 0};
  inclined.omega_b = zeros(3);
  inclined.R_wb = identity(3);
  inclined.terrainPoint_w = zeros(3);
  inclined.terrainNormal_w = {0, 0.6, -0.8};
  inclined.terrainVelocity_w = zeros(3);

  pulling.p_w = {0, 0, 0.01};
  pulling.v_b = {2, 0, -1};
  pulling.omega_b = zeros(3);
  pulling.R_wb = identity(3);
  pulling.terrainPoint_w = zeros(3);
  pulling.terrainNormal_w = {0, 0, -1};
  pulling.terrainVelocity_w = zeros(3);

  offset.p_w = {0, 0, 0.01};
  offset.v_b = {0, 0, -1};
  offset.omega_b = {0, -1, 0};
  offset.R_wb = identity(3);
  offset.terrainPoint_w = zeros(3);
  offset.terrainNormal_w = {0, 0, -1};
  offset.terrainVelocity_w = zeros(3);

  separating.p_w = {0, 0, 0.01 - 0.2 * time};
  separating.v_b = {0, 0, -0.2};
  separating.omega_b = zeros(3);
  separating.R_wb = identity(3);
  separating.terrainPoint_w = zeros(3);
  separating.terrainNormal_w = {0, 0, -1};
  separating.terrainVelocity_w = zeros(3);

  assert(abs(inclined.gap + 0.01) < 1e-12 and abs(inclined.normalForce - 10) < 1e-9,
    "Inclined-plane normal penetration/force is incorrect");
  assert(sum(abs(inclined.force_b - {-3, 6, -8})) < 1e-9,
    "Tangential friction must be capped by mu times normal force");
  assert(abs(inclined.tangentialPower + 6) < 1e-9,
    "Tangential friction must dissipate relative-motion energy");
  assert(abs(pulling.normalForce) < 1e-12 and sum(abs(pulling.force_b)) < 1e-12,
    "An unloading damper must not create tensile contact or unladen friction");
  assert(abs(offset.normalForce - 10) < 1e-9 and sum(abs(offset.moment_b - {0, 10, 0})) < 1e-9,
    "Angular tip velocity and CG contact moment must both be included");
  when time >= 0.06 then
    assert(not separating.contact and sum(abs(separating.force_b)) < 1e-12,
      "Contact must switch off when the leg leaves the plane");
  end when;
  annotation(experiment(StopTime = 0.1, Tolerance = 1e-8));
end ContactPhysics;
