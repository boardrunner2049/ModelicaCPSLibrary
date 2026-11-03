// Harness wrapper for ModelicaCPSLibrary.Physical.Electrical.Drives.ESC.AverageThreePhase.
// The ESC has no defaults for Ron, Rdiode, Vdiode and its dc_p/dc_n/phase[3]
// pins are acausal; exported alone it is refused ("parameter Ron has neither
// a binding nor an initial value"). The wrapper uses the library's own
// three-phase bench values (Tests.Support.ThreePhaseDrive: Ron = Rdiode =
// 5 mOhm, Vdiode = 0.5 V; Goff and currentThreshold keep their defaults),
// feeds it from an ideal 16 V bus (dc_n grounded; the source can absorb
// regenerated current) and loads the three phases with a floating star of
// R = 0.2 Ohm + L = 0.1 mH + a controlled EMF per phase. The electrical
// angle is a harness input, so commutation is exercised without a rotor
// model. Two inductor currents are fixed at start values {i0, -i0} (i0 is a
// tunable wrapper parameter, default 0: a winding current flowing out of
// phase 1 and back into phase 2); the third follows from star-point KCL.
model FireHarness_AverageThreePhase
  input Real escCommand(start = 0) "ESC throttle command";
  input Real electricalAngle(unit = "rad", start = 0) "Rotor electrical angle seen by the six-step logic";
  input Real emf[3](each unit = "V", each start = 0) "Per-phase EMF in series with each load branch (load side + , star side -)";
  parameter Real i0(unit = "A") = 0 "Initial winding current out of phase 1, back into phase 2";
  ModelicaCPSLibrary.Physical.Electrical.Drives.ESC.AverageThreePhase esc(Ron = 0.005, Rdiode = 0.005, Vdiode = 0.5);
  Modelica.Electrical.Analog.Sources.ConstantVoltage bus(V = 16);
  Modelica.Electrical.Analog.Basic.Ground ground;
  Modelica.Electrical.Analog.Basic.Resistor loadR[3](each R = 0.2);
  Modelica.Electrical.Analog.Basic.Inductor loadL[3](each L = 0.1e-3, i(start = {i0, -i0, 0}, fixed = {true, true, false}));
  Modelica.Electrical.Analog.Sources.SignalVoltage source[3];
  output Real phaseCurrent[3](each unit = "A") = esc.phaseCurrent "Current out of each ESC phase pin into the load";
  output Real dcCurrent(unit = "A") = esc.dcCurrent "Current drawn from the bus into dc_p";
  output Real electricalPower(unit = "W") = esc.electricalPower;
  output Real motorPower(unit = "W") = esc.motorPower;
  output Real lossPower(unit = "W") = esc.lossPower;
  output Boolean coast = esc.coast;
  output Boolean recirculating = esc.recirculating;
  output Integer sector = esc.control.sector;
  output Real starVoltage(unit = "V") = source[1].n.v "Floating star-point potential w.r.t. dc_n";
equation
  esc.escCommand = escCommand;
  esc.electricalAngle = electricalAngle;
  connect(bus.p, esc.dc_p);
  connect(bus.n, esc.dc_n);
  connect(bus.n, ground.p);
  for j in 1:3 loop
    source[j].v = emf[j];
    connect(esc.phase[j], loadR[j].p);
    connect(loadR[j].n, loadL[j].p);
    connect(loadL[j].n, source[j].p);
    connect(source[j].n, source[1].n);
  end for;
end FireHarness_AverageThreePhase;
