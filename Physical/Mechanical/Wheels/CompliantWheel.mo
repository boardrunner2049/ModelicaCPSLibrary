within ModelicaCPSLibrary.Physical.Mechanical.Wheels;

model CompliantWheel "Point-contact wheel: compliant normal load, load-proportional slip forces, friction circle"
  parameter Real rHub_b[3](each unit = "m") = {0, 0, 0.05}
    "Wheel hub (centre) offset from vehicle CG in body FRD; fixed to the body";
  parameter Real radius(min = 0, unit = "m") = 0.05 "Loaded rolling radius";
  parameter Real stiffness(min = 0, unit = "N/m") = 2000
    "Combined tyre and suspension stiffness along the terrain normal";
  parameter Real damping(min = 0, unit = "N.s/m") = 50;
  parameter Real frictionCoefficient(min = 0) = 0.9 "Friction-circle limit, times normal load";
  parameter Real longitudinalStiffness(min = 0) = 15
    "Longitudinal force per unit slip, per unit normal load (driven wheels)";
  parameter Real corneringStiffness(min = 0, unit = "1/rad") = 10
    "Lateral force per radian of slip angle, per unit normal load";
  parameter Real rollingResistance(min = 0) = 0.02
    "Rolling-resistance coefficient: opposing force per unit normal load";
  parameter Real relaxationSpeed(min = 1e-6, unit = "m/s") = 0.5
    "Slip denominators use sqrt(vx^2 + relaxationSpeed^2): viscous below, slip-based above";
  parameter Boolean driven = false "True: longitudinal force from surfaceSpeed slip";

  input Real steerAngle(unit = "rad") "Wheel heading about body z; positive turns the wheel right";
  input Real surfaceSpeed(unit = "m/s") "Tread speed omega*radius of a driven wheel; unused otherwise";
  input Real p_w[3](each unit = "m");
  input Real v_b[3](each unit = "m/s");
  input Real omega_b[3](each unit = "rad/s");
  input Real R_wb[3, 3] "Body to world rotation";
  input Real terrainPoint_w[3](each unit = "m");
  input Real terrainNormal_w[3] "Unit surface normal toward free space";
  input Real terrainVelocity_w[3](each unit = "m/s");

  output Real force_b[3](each unit = "N") "Total contact force on the body";
  output Real moment_b[3](each unit = "N.m") "About the vehicle CG";
  output Boolean contact;
  output Real gap(unit = "m");
  output Real normalForce(unit = "N");
  output Real longitudinalForce(unit = "N") "Along the wheel heading in the terrain plane";
  output Real lateralForce(unit = "N") "Positive to the wheel's right";
  output Real longitudinalSpeed(unit = "m/s") "Hub speed along the wheel heading";
  output Real lateralSpeed(unit = "m/s") "Hub speed to the wheel's right";

  ModelicaCPSLibrary.Physical.Mechanical.Contact.ContactMode mode;
  ModelicaCPSLibrary.Physical.Mechanical.Contact.UnilateralSpringDamper normalLaw(
    stiffness = stiffness, damping = damping);
protected
  Real contactPoint_b[3](each unit = "m");
  Real hubVelocity_w[3](each unit = "m/s");
  Real heading_w[3];
  Real headingInPlane_w[3];
  Real forward_w[3] "Unit wheel heading in the terrain plane";
  Real right_w[3] "Unit lateral axis in the terrain plane";
  Real slipScale(unit = "m/s");
  Real longitudinalDemand(unit = "N");
  Real lateralDemand(unit = "N");
  Real demand(unit = "N");
  Real limitRatio;
  Real force_w[3](each unit = "N");
equation
  assert(abs(terrainNormal_w * terrainNormal_w - 1) < 1e-8,
    "Terrain normal supplied to a wheel must be a unit vector");
  // Contact point: the lowest point of a wheel disc on a plane, the hub
  // offset by one radius against the terrain normal (camber neglected).
  contactPoint_b = rHub_b - radius * transpose(R_wb) * terrainNormal_w;
  gap = terrainNormal_w * (p_w + R_wb * contactPoint_b - terrainPoint_w);
  hubVelocity_w = R_wb * (v_b + cross(omega_b, rHub_b)) - terrainVelocity_w;

  mode.gap = gap;
  contact = mode.contact;
  normalLaw.active = contact;
  normalLaw.gap = gap;
  normalLaw.normalVelocity = terrainNormal_w * hubVelocity_w;
  normalForce = normalLaw.normalForce;

  heading_w = R_wb * {cos(steerAngle), sin(steerAngle), 0};
  headingInPlane_w = heading_w - (heading_w * terrainNormal_w) * terrainNormal_w;
  forward_w = headingInPlane_w / sqrt(max(headingInPlane_w * headingInPlane_w, 1e-12));
  right_w = cross(forward_w, terrainNormal_w);
  longitudinalSpeed = forward_w * hubVelocity_w;
  lateralSpeed = right_w * hubVelocity_w;

  // Load-proportional slip forces. Slip ratio and slip angle are normalised
  // by sqrt(vx^2 + v0^2): at speed this is the usual slip model, and near
  // rest it becomes viscous, so the model stays smooth through zero speed.
  slipScale = sqrt(longitudinalSpeed * longitudinalSpeed + relaxationSpeed * relaxationSpeed);
  longitudinalDemand = normalForce * (if driven then
      longitudinalStiffness * (surfaceSpeed - longitudinalSpeed) / slipScale
    else -rollingResistance * longitudinalSpeed / slipScale);
  lateralDemand = -normalForce * corneringStiffness * lateralSpeed / slipScale;
  demand = sqrt(longitudinalDemand * longitudinalDemand + lateralDemand * lateralDemand);
  // Both branches may be evaluated: keep the unused one finite (hosts may trap 0/0).
  limitRatio = noEvent(if demand > frictionCoefficient * normalForce and demand > 0
    then frictionCoefficient * normalForce / max(demand, 1e-12) else 1);
  longitudinalForce = limitRatio * longitudinalDemand;
  lateralForce = limitRatio * lateralDemand;

  force_w = normalForce * terrainNormal_w + longitudinalForce * forward_w + lateralForce * right_w;
  force_b = transpose(R_wb) * force_w;
  moment_b = cross(contactPoint_b, force_b);
  annotation(Documentation(info = "<html><p>The vehicle rigid body supplies the sprung mass; the
wheel adds no mass and no states. A driven wheel's spin is a state of the drivetrain that supplies
<code>surfaceSpeed</code> and receives <code>longitudinalForce*radius</code> as its load torque. A
free wheel has rolling resistance only. Forces act at the contact point; the normal law is the
library's unilateral spring-damper and its contact relation generates state events. Camber,
self-aligning torque and load-dependent stiffness are neglected.</p></html>"));
end CompliantWheel;
