within ModelicaCPSLibrary.Tests;
model ThreePhaseRegeneration "Gate-off diodes return energy from an initially spinning motor"
  Support.ThreePhaseDrive drive(omega_start=1600,kQ=0,command=0,
    redeclare Physical.Electrical.Drives.ESC.SwitchingThreePhase esc(pwmFrequency=10000));
equation
  when time >= 0.001 then
    assert(drive.esc.coast,"Initially disabled inverter must be Hi-Z");
    assert(drive.esc.dcCurrent < -1,"Gate-off body diodes must permit energy return");
    assert(drive.bus.voltage > 16,"Returned current must raise the finite-impedance DC bus");
    assert(drive.speed < 1600,"Regeneration must remove mechanical energy");
  end when;
  annotation(experiment(StopTime=0.002,Interval=0.000002,Tolerance=1e-9));
end ThreePhaseRegeneration;
