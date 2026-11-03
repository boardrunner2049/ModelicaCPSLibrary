// Harness wrapper for ModelicaCPSLibrary.Physical.Electrical.Drives.Bridges.SwitchingHalfBridge.
// The switched leg has no defaults for Ron, Rdiode, Vdiode and three acausal pins
// (dc_p, dc_n, phase); exported alone it is refused ("parameter Ron has
// neither a binding nor an initial value"). The wrapper sets Ron = 10 mOhm,
// Rdiode = 20 mOhm, Vdiode = 0.7 V (Goff keeps its 1e-5 S default), feeds
// the leg from an ideal 16 V DC bus (dc_n grounded) and loads the phase pin
// with R = 1 Ohm + L = 0.5 mH in series with a controlled back-EMF to ground,
// so the harness can drive real PWM gate patterns (asynchronous with diode
// freewheel, synchronous complementary), low-side braking and diode
// regeneration, and compare cycle averages with the mean model.
model FireHarness_SwitchingHalfBridge
  input Boolean highGate(start = false) "High-side switch gate";
  input Boolean lowGate(start = false) "Low-side switch gate";
  input Real backEMF(unit = "V", start = 0) "Load back-EMF (phase load return to ground)";
  ModelicaCPSLibrary.Physical.Electrical.Drives.Bridges.SwitchingHalfBridge leg(Ron = 0.01, Rdiode = 0.02, Vdiode = 0.7);
  Modelica.Electrical.Analog.Sources.ConstantVoltage bus(V = 16);
  Modelica.Electrical.Analog.Basic.Resistor loadR(R = 1);
  Modelica.Electrical.Analog.Basic.Inductor loadL(L = 0.5e-3, i(start = 0, fixed = true));
  Modelica.Electrical.Analog.Sources.SignalVoltage emf;
  Modelica.Electrical.Analog.Basic.Ground ground;
  output Real phaseCurrent(unit = "A") = loadL.i "Current from the phase pin into the load";
  output Real phaseVoltage(unit = "V") = leg.phase.v "Phase pin voltage w.r.t. dc_n";
  output Real dcCurrent(unit = "A") = -bus.i "Current delivered by the bus into dc_p";
equation
  leg.highGate = highGate;
  leg.lowGate = lowGate;
  emf.v = backEMF;
  connect(bus.p, leg.dc_p);
  connect(bus.n, leg.dc_n);
  connect(bus.n, ground.p);
  connect(leg.phase, loadR.p);
  connect(loadR.n, loadL.p);
  connect(loadL.n, emf.p);
  connect(emf.n, ground.p);
end FireHarness_SwitchingHalfBridge;
