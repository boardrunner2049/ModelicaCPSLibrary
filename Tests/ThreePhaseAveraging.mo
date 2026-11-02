within ModelicaCPSLibrary.Tests;
model ThreePhaseAveraging "Locked-shaft P3/P4 CCM mean-current and loss benchmark"
  Support.LockedThreePhase average(command=0.6);
  Support.LockedThreePhase switched(command=0.6,
    redeclare Physical.Electrical.Drives.ESC.SwitchingThreePhase esc(pwmFrequency=10000));
  Real meanCurrentAverage(start=0,fixed=true);
  Real meanCurrentSwitched(start=0,fixed=true);
  Real meanPowerAverage(start=0,fixed=true);
  Real meanPowerSwitched(start=0,fixed=true);
  final parameter Real expectedCurrent = (0.6*16-0.4*0.5)/(2*0.2+0.005+0.005+0.02*0.6^2);
equation
  der(meanCurrentAverage) = if time >= 0.04 then average.phaseCurrent[3]/0.01 else 0;
  der(meanCurrentSwitched) = if time >= 0.04 then switched.phaseCurrent[3]/0.01 else 0;
  der(meanPowerAverage) = if time >= 0.04 then average.esc.electricalPower/0.01 else 0;
  der(meanPowerSwitched) = if time >= 0.04 then switched.esc.electricalPower/0.01 else 0;
  when terminal() then
    assert(abs(meanCurrentAverage-expectedCurrent) < 0.01,"P3 locked-rotor electrical equilibrium failed");
    assert(abs(meanCurrentSwitched-meanCurrentAverage)/meanCurrentAverage < 0.01,
      "P4 mean current differs from P3 by over 1% in the declared CCM benchmark");
    assert(abs(meanPowerSwitched-meanPowerAverage)/meanPowerAverage < 0.01,
      "P4 mean power differs from P3 by over 1% in the declared CCM benchmark");
  end when;
  annotation(experiment(StopTime=0.05,Interval=0.000005,Tolerance=1e-9));
end ThreePhaseAveraging;
