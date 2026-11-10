within ModelicaCPSLibrary.Systems.Payloads;
model Winch "A load on a winch line paid out and wound in at a rate set by a servo (ArduPilot WINCH_TYPE 1: rate = maxRate (pulse - 1500)/500, positive out)"
  extends PartialSlungLoad;
  parameter Real maxRate(unit="m/s") = 0.5 "Line speed at full command";
  parameter Real maxLength(unit="m") = 10 "Line on the drum";
  parameter Real length_start(unit="m") = 0.2;
  parameter Real deadband(unit="us") = 25 "Around 1500 us";
  input Real pulse(unit="us") = 1500 "Servo command";
  output Real paidOut(unit="m") "Line paid out";
protected
  Real length(unit="m", start=length_start, fixed=true);
  Real rate(unit="m/s");
equation
  rate = if abs(pulse - 1500) < deadband then 0 else maxRate*max(-1, min(1, (pulse - 1500)/500));
  der(length) = if (length <= 0 and rate < 0) or (length >= maxLength and rate > 0) then 0 else rate;
  rest = max(length, 0.01);
  attached = true;
  paidOut = length;
end Winch;
