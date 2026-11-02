within ModelicaCPSLibrary.Tests;
model ThreePhaseCoast "P3/P4 signed currents, coast/restart, energy and finite dead time"
  Support.ThreePhaseDrive average;
  Support.ThreePhaseDrive switched(redeclare Physical.Electrical.Drives.ESC.SwitchingThreePhase
    esc(pwmFrequency=10000,deadTime=1e-6));
equation
  average.command = if time < 0.03 then 0.6 else if time < 0.05 then 0 else 0.7;
  switched.command = average.command;
  when time >= 0.0301 then
    assert(average.esc.recirculating and switched.esc.recirculating,"Cutoff must retain a recirculation path");
    assert(max(abs(average.phaseCurrent)) > 1 and max(abs(switched.phaseCurrent)) > 1,
      "Cutoff erased winding current");
  end when;
  when time >= 0.049 then
    assert(average.esc.coast and switched.esc.coast,"Residual current did not release to Hi-Z");
    assert(max(abs(average.phaseCurrent)) < 0.001 and max(abs(switched.phaseCurrent)) < 0.001,
      "Coast residual exceeds declared off-conductance tolerance");
  end when;
  when time >= 0.079 then
    assert(average.speed > 60 and switched.speed > 60,"Forward startup/restart failed");
    assert(min(average.phaseCurrent) < -1 and max(average.phaseCurrent) > 1,
      "Forward rotation must still have signed phase currents");
  end when;
  annotation(experiment(StopTime=0.08,Interval=0.00001,Tolerance=1e-8));
end ThreePhaseCoast;
