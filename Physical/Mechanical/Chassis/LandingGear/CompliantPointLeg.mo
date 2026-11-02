within ModelicaCPSLibrary.Physical.Mechanical.Chassis.LandingGear;

model CompliantPointLeg "Fixed leg tip with unilateral spring-damper ground contact"
  parameter Real rLeg_b[3](each unit = "m") = {0, 0, 0.1}
    "Uncompressed tip offset from vehicle CG in body FRD axes";
  parameter Real stiffness(min = 0, unit = "N/m") = 3000;
  parameter Real damping(min = 0, unit = "N.s/m") = 150;
  parameter Real tangentialDamping(min = 0, unit = "N.s/m") = 25;
  parameter Real frictionCoefficient(min = 0) = 0.6;

  input Real p_w[3](each unit = "m");
  input Real v_b[3](each unit = "m/s");
  input Real omega_b[3](each unit = "rad/s");
  input Real R_wb[3, 3] "Body to world rotation";
  input Real terrainPoint_w[3](each unit = "m");
  input Real terrainNormal_w[3] "Unit surface normal toward free space";
  input Real terrainVelocity_w[3](each unit = "m/s");

  output Real force_b[3](each unit = "N");
  output Real moment_b[3](each unit = "N.m");
  output Boolean contact;
  output Real gap(unit = "m");
  output Real normalForce(unit = "N");
  output Real force_w[3](each unit = "N");
  output Real tipPosition_w[3](each unit = "m");
  output Real tipVelocity_w[3](each unit = "m/s");
  output Real tangentialPower(unit = "W");

  ModelicaCPSLibrary.Physical.Mechanical.Contact.ContactMode mode;
  ModelicaCPSLibrary.Physical.Mechanical.Contact.UnilateralSpringDamper normalLaw(
    stiffness = stiffness, damping = damping);
  ModelicaCPSLibrary.Physical.Mechanical.Contact.TangentialFriction frictionLaw(
    tangentialDamping = tangentialDamping, frictionCoefficient = frictionCoefficient);
protected
  Real relativeVelocity_w[3](each unit = "m/s");
  Real normalVelocity(unit = "m/s");
equation
  assert(abs(terrainNormal_w * terrainNormal_w - 1) < 1e-8,
    "Terrain normal supplied to a leg must be a unit vector");
  tipPosition_w = p_w + R_wb * rLeg_b;
  tipVelocity_w = R_wb * (v_b + cross(omega_b, rLeg_b));
  relativeVelocity_w = tipVelocity_w - terrainVelocity_w;
  gap = terrainNormal_w * (tipPosition_w - terrainPoint_w);
  normalVelocity = terrainNormal_w * relativeVelocity_w;

  mode.gap = gap;
  contact = mode.contact;
  normalLaw.active = contact;
  normalLaw.gap = gap;
  normalLaw.normalVelocity = normalVelocity;
  normalForce = normalLaw.normalForce;
  frictionLaw.active = contact;
  frictionLaw.normalForce = normalForce;
  frictionLaw.velocityTangential_w = relativeVelocity_w - normalVelocity * terrainNormal_w;
  tangentialPower = frictionLaw.power;

  force_w = normalForce * terrainNormal_w + frictionLaw.force_w;
  force_b = transpose(R_wb) * force_w;
  moment_b = cross(rLeg_b, force_b);
  annotation(Documentation(info = "<html><p>The vehicle rigid body supplies the sprung mass. This massless leg has no separate tip-mass state and does not reset vehicle velocity at touchdown. Contact relations generate state events. Friction is dissipative viscous friction capped by the Coulomb load limit; it is not a static-friction constraint.</p></html>"));
end CompliantPointLeg;
