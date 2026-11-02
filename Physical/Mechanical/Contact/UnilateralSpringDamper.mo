within ModelicaCPSLibrary.Physical.Mechanical.Contact;

model UnilateralSpringDamper "Penalty contact; normal force cannot pull the body toward the surface"
  parameter Real stiffness(min = 0, unit = "N/m") = 3000;
  parameter Real damping(min = 0, unit = "N.s/m") = 150;
  input Boolean active;
  input Real gap(unit = "m") "Signed surface gap, positive outside";
  input Real normalVelocity(unit = "m/s") "Positive when separating from the surface";
  output Real penetration(unit = "m");
  output Real normalForce(unit = "N");
initial equation
  assert(stiffness >= 0 and damping >= 0, "Contact stiffness and damping must be nonnegative");
equation
  penetration = max(0, -gap);
  normalForce = if active and gap <= 0 then
    max(0, stiffness * penetration - damping * normalVelocity) else 0;
end UnilateralSpringDamper;
