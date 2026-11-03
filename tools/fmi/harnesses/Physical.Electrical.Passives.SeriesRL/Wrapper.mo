// Harness wrapper for ModelicaCPSLibrary.Physical.Electrical.Passives.SeriesRL.
// SeriesRL is a two-terminal acausal component: exported alone its pins are
// open (structurally singular) and R, L have no values. This wrapper puts two
// instances in series across an ideal controlled voltage source (ground at
// the far end):
//   - rl:  R = 2 Ohm, L = 10 mH, signed initial current i_start = -1 A (fixed);
//   - rl2: R = 1 Ohm, L = 20 mH, fixedCurrentStart = false with a deliberately
//          inconsistent start guess i_start = 0.7 A: like the dependent phase
//          of a floating star, its initial current must come from the network
//          (series KCL: -1 A), not from i_start.
// The two pieces have different L/R (5 ms and 20 ms), so the voltage split
// between them depends on the current, not just on the source voltage.
// The loop is first order with tau = (L1+L2)/(R1+R2) = 10 ms.
// The wrapper integrates the source energy and the reported copper losses so
// the harness can check the energy balance
//   E_source - E_loss = sum(magneticEnergy) - (L1+L2)*i_start^2/2.
model FireHarness_SeriesRL
  input Real voltage(unit = "V") "Source voltage applied across the series pair";
  ModelicaCPSLibrary.Physical.Electrical.Passives.SeriesRL rl(R = 2, L = 0.01, i_start = -1);
  ModelicaCPSLibrary.Physical.Electrical.Passives.SeriesRL rl2(R = 1, L = 0.02, i_start = 0.7,
    fixedCurrentStart = false);
  Modelica.Electrical.Analog.Sources.SignalVoltage source;
  Modelica.Electrical.Analog.Basic.Ground ground;
  Modelica.Units.SI.Energy sourceEnergy(start = 0, fixed = true) "Integral of v*i delivered by the source";
  Modelica.Units.SI.Energy lossEnergy(start = 0, fixed = true) "Integral of the reported copper losses";
  output Modelica.Units.SI.Current current = rl.i;
  output Modelica.Units.SI.Voltage terminalVoltage = rl.v "Voltage across the first piece only";
  output Modelica.Units.SI.Energy energyError "Source energy minus dissipated and stored energy change";
equation
  source.v = voltage;
  connect(source.p, rl.p);
  connect(rl.n, rl2.p);
  connect(rl2.n, ground.p);
  connect(source.n, ground.p);
  der(sourceEnergy) = voltage*rl.i;
  der(lossEnergy) = rl.copperLoss + rl2.copperLoss;
  energyError = sourceEnergy - lossEnergy - (rl.magneticEnergy + rl2.magneticEnergy - 0.03*(-1)^2/2);
end FireHarness_SeriesRL;
