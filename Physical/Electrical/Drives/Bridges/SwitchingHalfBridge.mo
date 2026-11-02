within ModelicaCPSLibrary.Physical.Electrical.Drives.Bridges;
model SwitchingHalfBridge "Two finite-on-resistance switches with antiparallel body diodes"
  extends PartialHalfBridge;
  input Boolean highGate;
  Modelica.Electrical.Analog.Ideal.IdealClosingSwitch highSwitch(Ron=Ron,Goff=Goff);
equation
  connect(highSwitch.p,dc_p);
  connect(highSwitch.n,phase);
  highSwitch.control = highGate;
  assert(not (highGate and lowGate),"Half-bridge shoot-through gate command");
end SwitchingHalfBridge;
