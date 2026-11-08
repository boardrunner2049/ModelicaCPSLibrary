within ModelicaCPSLibrary.Utilities.Math;
function rateLimitedLag "Rate of a first-order lag toward a target, with a deadband and a rate limit (servo)"
  input Real target;
  input Real current;
  input Real timeConstant "Lag time constant (> 0)";
  input Real maxRate "Largest magnitude of the rate (> 0)";
  input Real deadband = 0 "No motion while |target - current| <= deadband";
  output Real rate;
protected
  Real error = target - current;
algorithm
  rate := noEvent(if abs(error) <= deadband then 0
    else min(maxRate, max(-maxRate, (error - sign(error)*deadband)/timeConstant)));
end rateLimitedLag;
