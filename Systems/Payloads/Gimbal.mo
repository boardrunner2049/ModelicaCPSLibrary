within ModelicaCPSLibrary.Systems.Payloads;
model Gimbal "A three-axis servo gimbal (ArduPilot MNT1_TYPE 1: tilt, roll, pan servos): each angle follows its commanded angle at a limited rate; the camera's ground point"
  extends PartialPayload;
  parameter Real angleMin[3](each unit="rad") = {-1.5708, -0.5236, -2.7925} "Tilt, roll, pan at the minimum pulse";
  parameter Real angleMax[3](each unit="rad") = {0.4363, 0.5236, 2.7925} "At the maximum pulse";
  parameter Real maxRate[3](each unit="rad/s") = {1.5708, 1.5708, 2.0944};
  parameter Real timeConstant(unit="s") = 0.05 "Of the stabilization loop";
  parameter Real pwmMin(unit="us") = 1000;
  parameter Real pwmMax(unit="us") = 2000;
  parameter Real groundDown(unit="m") = 0;
  input Real pulse[3](each unit="us") = fill(1500, 3) "Tilt, roll, pan servo commands";
  output Real angle[3](each unit="rad") "Tilt, roll, pan of the camera relative to the body";
  output Real groundPoint_w[3](each unit="m") "Where the camera's boresight meets the ground (zeros if above the horizon)";
protected
  Real a[3](each start=0, each fixed=true);
  Real command[3](each unit="rad");
  Real boresight_w[3];
equation
  for i in 1:3 loop
    command[i] = angleMin[i] + (angleMax[i] - angleMin[i])*max(0, min(1, (pulse[i] - pwmMin)/(pwmMax - pwmMin)));
    der(a[i]) = max(-maxRate[i], min(maxRate[i], (command[i] - a[i])/timeConstant));
  end for;
  angle = a;
  // boresight: body x turned by pan (about z) then tilt (about y); roll does not move it
  boresight_w = truth.R_wb*{cos(a[1])*cos(a[3]), cos(a[1])*sin(a[3]), -sin(a[1])};
  groundPoint_w = noEvent(if boresight_w[3] > 1e-3 then attach_w + (groundDown - attach_w[3])/boresight_w[3]*boresight_w else zeros(3));
  force_b = zeros(3);
  moment_b = zeros(3);
end Gimbal;
