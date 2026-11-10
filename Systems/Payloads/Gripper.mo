within ModelicaCPSLibrary.Systems.Payloads;
model Gripper "A load held on a short hook line, released and grabbed by a servo (ArduPilot GRIP_TYPE 1: release below releasePulse, grab above grabPulse near the load, neutral holds)"
  extends PartialSlungLoad;
  parameter Real hookLength(unit="m") = 0.1 "Unstretched hook line [assumed]";
  parameter Real releasePulse(unit="us") = 1250 "Below it: release [ArduPilot]";
  parameter Real grabPulse(unit="us") = 1750 "Above it: grab, if the load is within two hook lengths [ArduPilot]";
  input Real pulse(unit="us") = 1500 "Servo command";
  output Boolean holding;
  output Real holdingLevel "1 holding, else 0";
protected
  discrete Boolean held(start=true, fixed=true);
equation
  rest = hookLength;
  when pulse < releasePulse then
    held = false;
  elsewhen pulse > grabPulse and lineLength < 2*hookLength then
    held = true;
  end when;
  holding = held;
  holdingLevel = if held then 1 else 0;
  attached = held;
end Gripper;
