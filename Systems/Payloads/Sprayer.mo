within ModelicaCPSLibrary.Systems.Payloads;
model Sprayer "A liquid tank emptied by a pump whose flow follows its servo (ArduPilot SPRAY_ENABLE: pump function 22); the liquid's weight loads the vehicle at the tank"
  extends PartialPayload;
  parameter Real tankVolume(unit="l") = 5 "Capacity [datasheet]";
  parameter Real volume_start(unit="l") = 5 "Liquid at the start [configuration]";
  parameter Real maxFlow(unit="l/min") = 5 "Pump at full command [datasheet]";
  parameter Real density(unit="kg/l") = 1.0 "[physics]";
  parameter Real pwmMin(unit="us") = 1000 "Pump off [datasheet]";
  parameter Real pwmMax(unit="us") = 2000 "Pump full [datasheet]";
  input Real pulse(unit="us") = 1000 "Pump servo command";
  output Real volume(unit="l");
  output Real flowRate(unit="l/min");
  output Real level "Fraction of the tank";
protected
  Real liquid(unit="l", start=volume_start, fixed=true);
equation
  flowRate = if liquid > 0 then maxFlow*max(0, min(1, (pulse - pwmMin)/(pwmMax - pwmMin))) else 0;
  der(liquid) = -flowRate/60;
  volume = max(liquid, 0);
  level = volume/tankVolume;
  force_b = transpose(truth.R_wb)*{0, 0, density*volume*gravity};
  moment_b = cross(position_b, force_b);
end Sprayer;
