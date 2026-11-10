within ModelicaCPSLibrary.Systems.Payloads;
model LandingGear "Retractable gear moved by a servo (ArduPilot LGR_ENABLE: deployed at the maximum pulse, retracted at the minimum) at a finite speed; its drag area while deployed"
  extends PartialPayload;
  parameter Real retractTime(unit="s") = 2 "Full travel";
  parameter Real dragArea(unit="m2") = 0.01 "Cd A of the deployed gear";
  parameter Real pwmMin(unit="us") = 1000;
  parameter Real pwmMax(unit="us") = 2000;
  input Real pulse(unit="us") = 2000 "Servo command";
  output Real deployed "1 deployed, 0 retracted";
protected
  Real position(start=1, fixed=true);
  Real target;
  Real drag_w[3](each unit="N");
equation
  target = max(0, min(1, (pulse - pwmMin)/(pwmMax - pwmMin)));
  der(position) = max(-1/retractTime, min(1/retractTime, 10*(target - position)));
  deployed = max(0, min(1, position));
  drag_w = -0.5*1.225*dragArea*deployed*sqrt(attachVelocity_w*attachVelocity_w + 1e-9)*attachVelocity_w;
  force_b = transpose(truth.R_wb)*drag_w;
  moment_b = cross(position_b, force_b);
end LandingGear;
