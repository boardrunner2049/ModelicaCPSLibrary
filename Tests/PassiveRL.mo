within ModelicaCPSLibrary.Tests;
model PassiveRL "Signed RL transient and passive energy balance"
  Physical.Electrical.Passives.SeriesRL branch(R=2,L=0.02,i_start=-1);
  Modelica.Electrical.Analog.Sources.SignalVoltage source;
  Modelica.Electrical.Analog.Basic.Ground ground;
  Real expected(unit="A");
  Real energy(start=0.01,fixed=true,unit="J");
equation
  source.v=if time<0.1 then -4 else 4;
  connect(source.p,branch.p);
  connect(branch.n,source.n);
  connect(source.n,ground.p);
  expected=if time<0.1 then -2+exp(-time/0.01)
    else 2+(-4+exp(-10))*exp(-(time-0.1)/0.01);
  der(energy)=branch.v*branch.i-branch.copperLoss;
  assert(abs(branch.i-expected)<1e-5,"Bidirectional RL response changed");
  assert(abs(energy-branch.magneticEnergy)<1e-7,"RL energy balance failed");
  annotation(experiment(StopTime=0.2,Tolerance=1e-10,Interval=0.0001));
end PassiveRL;
