within ModelicaCPSLibrary.Physical.Electrical.Drives.ESC;
model PassThrough "Stateless normalized command forwarding"
  input Real escCommand(unit="1") "Normalized command in [0,1]";
  output Real normalizedCommand(unit="1");
equation
  assert(escCommand >= 0 and escCommand <= 1,
    "PassThrough expects a normalized command in [0,1]; normalize PWM at the adapter");
  normalizedCommand = escCommand;
  annotation(Documentation(info="<html><p>Algebraic command boundary for a speed-response motor. No delay, switching or electrical power model. PWM normalization belongs to Adapters.VirtualIO.PwmInput.</p></html>"));
end PassThrough;
