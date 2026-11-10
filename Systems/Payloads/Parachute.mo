within ModelicaCPSLibrary.Systems.Payloads;
model Parachute "A canopy released by a servo (ArduPilot CHUTE_TYPE 10: released above releasePulse), deploying after a delay and inflating, then drag 1/2 rho Cd A |v| v at the attachment"
  extends PartialPayload;
  parameter Real diameter(unit="m") = 1.2 "Canopy, inflated [datasheet]";
  parameter Real dragCoefficient = 1.5 "[datasheet]";
  parameter Real deployDelay(unit="s") = 0.3 "Release to canopy out [datasheet]";
  parameter Real inflationTime(unit="s") = 1.0 "[assumed]";
  parameter Real releasePulse(unit="us") = 1200 "Above it: released (CHUTE_SERVO_ON 1300, OFF 1100) [ArduPilot]";
  parameter Real pressure0(unit="Pa") = 101325 "[environment]";
  parameter Real temperature0(unit="K") = 288.15 "[environment]";
  input Real pulse(unit="us") = 1100 "Servo command";
  output Boolean released;
  output Real inflation "Canopy area fraction, 0..1";
  output Real drag(unit="N");
protected
  discrete Boolean out(start=false, fixed=true);
  discrete Real releaseTime(start=-1, fixed=true);
  Real filled(start=0, fixed=true);
  Real pressure(unit="Pa");
  Real airTemperature(unit="K");
  Real speed(unit="m/s");
  Real drag_w[3](each unit="N");
equation
  when pulse > releasePulse and not pre(out) then
    out = true;
    releaseTime = time;
  end when;
  released = out;
  der(filled) = if out and time >= releaseTime + deployDelay and filled < 1 then 1/inflationTime else 0;
  inflation = min(1, filled);
  (pressure, airTemperature) = Worlds.Environment.troposphere(-attach_w[3], pressure0, temperature0);
  speed = sqrt(attachVelocity_w*attachVelocity_w + 1e-9);
  drag_w = -0.5*pressure/(287.05287*airTemperature)*dragCoefficient*inflation*Modelica.Constants.pi
    *diameter^2/4*speed*attachVelocity_w;
  drag = sqrt(drag_w*drag_w);
  force_b = transpose(truth.R_wb)*drag_w;
  moment_b = cross(position_b, force_b);
end Parachute;
