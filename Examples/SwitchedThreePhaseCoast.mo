within ModelicaCPSLibrary.Examples;
model SwitchedThreePhaseCoast "P4 uses the same motor and load, replacing only the ESC"
  extends ThreePhaseCoast(redeclare Physical.Electrical.Drives.ESC.SwitchingThreePhase
    esc(pwmFrequency=10000,deadTime=1e-6));
  annotation(experiment(StopTime=0.08,Interval=0.000005,Tolerance=1e-8));
end SwitchedThreePhaseCoast;
