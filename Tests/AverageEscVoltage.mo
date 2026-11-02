within ModelicaCPSLibrary.Tests;
model AverageEscVoltage "P1 modulation clamp, supply changes and explicit PWM adapter"
  Physical.Electrical.Drives.ESC.AverageVoltage esc[5];
  Adapters.VirtualIO.PwmInput commands(nChannels=5,
    pwmMin={1000,1100,1200,1300,1400},pwmMax={2000,1900,1800,1700,1600});
  parameter Real command[5]={-0.5,0,0.25,1,2};
  Real supply(unit="V");
equation
  commands.pulseWidth_us = {500,1100,1350,1700,1800};
  supply = if time < 0.1 then 16 else if time < 0.2 then 8 else 0;
  for i in 1:5 loop
    esc[i].escCommand = command[i];
    esc[i].dcVoltage = supply;
  end for;
  assert(max(abs(esc.modulation-{0,0,0.25,1,1}))<1e-12,"ESC clamp changed");
  assert(max(abs(commands.normalizedCommand-esc.modulation))<1e-12,"PWM channel calibration changed");
  assert(max(abs(esc.equivalentVoltage-supply*{0,0,0.25,1,1}))<1e-12,"ESC supply scaling changed");
  annotation(experiment(StopTime=0.3,Tolerance=1e-9,Interval=0.001));
end AverageEscVoltage;
