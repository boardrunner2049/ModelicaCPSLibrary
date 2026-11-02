within ModelicaCPSLibrary.Physical.Mechanical.Contact;

model TangentialFriction "Dissipative viscous friction limited by the normal load"
  parameter Real tangentialDamping(min = 0, unit = "N.s/m") = 25;
  parameter Real frictionCoefficient(min = 0) = 0.6;
  parameter Real regularizationSpeed(min = 1e-12, unit = "m/s") = 1e-6;
  input Boolean active;
  input Real normalForce(unit = "N");
  input Real velocityTangential_w[3](each unit = "m/s");
  output Real force_w[3](each unit = "N");
  output Real power(unit = "W") "Nonpositive power against relative surface motion";
protected
  Real speed(unit = "m/s");
  Real forceMagnitude(unit = "N");
initial equation
  assert(tangentialDamping >= 0 and frictionCoefficient >= 0 and regularizationSpeed > 0,
    "Friction coefficients must be nonnegative and regularization speed positive");
equation
  // Preserve the exact norm while avoiding a singular generated derivative
  // at rest; active/normalForce still retain their contact-switching events.
  speed = if noEvent(velocityTangential_w * velocityTangential_w > 0) then
    sqrt(velocityTangential_w * velocityTangential_w) else 0;
  forceMagnitude = if active and normalForce > 0 then
    min(tangentialDamping * speed, frictionCoefficient * normalForce) else 0;
  force_w = -forceMagnitude * velocityTangential_w / max(speed, regularizationSpeed);
  power = force_w * velocityTangential_w;
end TangentialFriction;
