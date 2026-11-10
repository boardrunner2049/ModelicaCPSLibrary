within ModelicaCPSLibrary.Systems.Peripherals;
model PrecisionLanding "K0: line-of-sight angles to a landing target from a camera looking along device +z (angle x = atan(x/z), angle y = atan(y/z), device axes) and the distance; valid inside the field of view and range; continuous"
  extends PartialPeripheral;
  parameter Real targetPosition_w[3](each unit="m") = zeros(3) "Target, NED from the origin";
  parameter Real fieldOfViewX(unit="rad") = 1.047 "Full field of view about device y (x direction)";
  parameter Real fieldOfViewY(unit="rad") = 0.785 "Full field of view about device x (y direction)";
  parameter Real maxRange(unit="m") = 15 "Farthest detection";
  output Real angle[2](each unit="rad");
  output Real distance(unit="m");
  output Boolean valid;
  output Real validity = if valid then 1 else 0 "1 if valid, else 0";
protected
  Real los_s[3](each unit="m") "Target from the camera, device axes";
equation
  los_s = transpose(R_bs)*transpose(truth.R_wb)*(targetPosition_w - mountPosition_w);
  distance = sqrt(los_s*los_s);
  angle = noEvent(if los_s[3] > 1e-6 then {atan(los_s[1]/los_s[3]), atan(los_s[2]/los_s[3])} else {0, 0});
  valid = los_s[3] > 1e-6 and abs(angle[1]) <= fieldOfViewX/2 and abs(angle[2]) <= fieldOfViewY/2
    and distance <= maxRange;
end PrecisionLanding;
