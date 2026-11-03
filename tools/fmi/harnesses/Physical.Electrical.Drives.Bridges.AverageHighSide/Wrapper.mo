// Harness wrapper for ModelicaCPSLibrary.Physical.Electrical.Drives.Bridges.AverageHighSide.
// The bridge leg is an acausal three-pin component (dc_p, dc_n, phase) with
// parameters without values, so it cannot be exported alone. This wrapper:
//   - gives it Ron = 10 mOhm, Rdiode = 20 mOhm, Vdiode = 0.5 V;
//   - feeds dc_p/dc_n from an ideal 16 V bus whose negative rail dc_n is held
//     1 V BELOW ground by a second source (negRail), so dc_n.v = -1 V and the
//     leg's dc_n-referenced voltages differ from absolute potentials;
//   - loads the phase pin with a series R-L (0.5 Ohm, 1 mH) to ground, plus a
//     10 Ohm bleed resistor in parallel.
//   - a second, identical leg `idleBridge` with enabled = false is wired in
//     parallel (same rails, phase and modulation): it must carry no current
//     on any pin even when forward-biased (exercises the disabled branch). The averaged high side only sources
//     current (it blocks reverse current; real legs rely on external body
//     diodes), so the bleed gives the inductor a path when the leg is
//     disabled or its mean voltage is below the phase voltage.
// The loads are written as explicit OnePort equations rather than MSL
// Basic.Resistor/Inductor, because the MSL Resistor's heat-port assertion is
// refused by the Rumoca FMI C profile (see the harness report).
// `enabled` is a parameter, not an input: the Rumoca FMI C profile refuses a
// discrete equation that depends on an input (bridge.enabled = enabled), so a
// run-time enable cannot be driven. The leg's blocking and freewheel paths are
// exercised through the modulation instead (d = 0 gives a mean voltage of
// -Vdiode, the asynchronous low-side diode freewheel).
// Observers: bridgeLoss = power absorbed through all three pins (the
// component's internal conduction loss) and kclResidual = sum of pin currents.
model FireHarness_AverageHighSide
  model SeriesLoad "Series R-L, v = R*i + L*der(i)"
    extends Modelica.Electrical.Analog.Interfaces.OnePort;
    parameter Modelica.Units.SI.Resistance R;
    parameter Modelica.Units.SI.Inductance L;
  equation
    v = R*i + L*der(i);
  end SeriesLoad;

  model Bleed "Linear resistor, v = R*i"
    extends Modelica.Electrical.Analog.Interfaces.OnePort;
    parameter Modelica.Units.SI.Resistance R;
  equation
    v = R*i;
  end Bleed;

  input Real modulation "High-side duty cycle d in [0, 1]";
  parameter Boolean enabled = true "Leg enable (a parameter: see header)";
  ModelicaCPSLibrary.Physical.Electrical.Drives.Bridges.AverageHighSide bridge(
    Ron = 0.01, Rdiode = 0.02, Vdiode = 0.5);
  Modelica.Electrical.Analog.Sources.ConstantVoltage bus(V = 16);
  ModelicaCPSLibrary.Physical.Electrical.Drives.Bridges.AverageHighSide idleBridge(
    Ron = 0.01, Rdiode = 0.02, Vdiode = 0.5);
  Modelica.Electrical.Analog.Sources.ConstantVoltage negRail(V = 1) "ground -> dc_n, dc_n at -1 V";
  Modelica.Electrical.Analog.Basic.Ground ground;
  SeriesLoad load(R = 0.5, L = 0.001, i(start = 0, fixed = true));
  Bleed bleed(R = 10);
  output Modelica.Units.SI.Current legCurrent = bridge.current;
  output Modelica.Units.SI.Current loadCurrent = load.i;
  output Modelica.Units.SI.Voltage phaseVoltage = bridge.phase.v - bridge.dc_n.v;
  output Modelica.Units.SI.Power busPower = bus.v*bridge.dc_p.i;
  output Modelica.Units.SI.Power bridgeLoss = bridge.dc_p.v*bridge.dc_p.i + bridge.dc_n.v*bridge.dc_n.i
    + bridge.phase.v*bridge.phase.i;
  output Modelica.Units.SI.Voltage dcNegVoltage = bridge.dc_n.v;
  output Modelica.Units.SI.Current idleCurrent = idleBridge.current;
  output Modelica.Units.SI.Voltage idleForwardBias = idleBridge.meanVoltage - (idleBridge.phase.v - idleBridge.dc_n.v);
  output Modelica.Units.SI.Current kclResidual = bridge.dc_p.i + bridge.dc_n.i + bridge.phase.i;
equation
  bridge.modulation = modulation;
  bridge.enabled = enabled;
  connect(bus.p, bridge.dc_p);
  connect(bus.n, bridge.dc_n);
  connect(ground.p, negRail.p);
  connect(negRail.n, bus.n);
  idleBridge.modulation = modulation;
  idleBridge.enabled = false;
  connect(bus.p, idleBridge.dc_p);
  connect(bus.n, idleBridge.dc_n);
  connect(bridge.phase, idleBridge.phase);
  connect(bridge.phase, load.p);
  connect(load.n, ground.p);
  connect(bridge.phase, bleed.p);
  connect(bleed.n, ground.p);
end FireHarness_AverageHighSide;
