within ModelicaCPSLibrary.Physical.Electrical.Drives.Bridges;
model AverageHalfBridge "Mean high-side drive with physical low-side and diode paths"
  extends PartialHalfBridge;
  input Real modulation;
  input Boolean highSelected;
  AverageHighSide high(Ron=Ron,Rdiode=Rdiode,Vdiode=Vdiode);
equation
  connect(high.dc_p,dc_p);
  connect(high.dc_n,dc_n);
  connect(high.phase,phase);
  high.modulation = modulation;
  high.enabled = highSelected;
end AverageHalfBridge;
