within ModelicaCPSLibrary.Physical.Electrical.Drives.Bridges;
partial model PartialHalfBridge "Common low-side switch and both body diodes"
  parameter Modelica.Units.SI.Resistance Ron;
  parameter Modelica.Units.SI.Resistance Rdiode;
  parameter Modelica.Units.SI.Voltage Vdiode;
  parameter Modelica.Units.SI.Conductance Goff = 1e-5 "Finite off conductance; included in power balance";
  Modelica.Electrical.Analog.Interfaces.PositivePin dc_p;
  Modelica.Electrical.Analog.Interfaces.NegativePin dc_n;
  Modelica.Electrical.Analog.Interfaces.PositivePin phase;
  input Boolean lowGate;
  Modelica.Electrical.Analog.Ideal.IdealClosingSwitch lowSwitch(Ron=Ron,Goff=Goff);
  Modelica.Electrical.Analog.Ideal.IdealDiode highDiode(Ron=Rdiode,Goff=Goff,Vknee=Vdiode);
  Modelica.Electrical.Analog.Ideal.IdealDiode lowDiode(Ron=Rdiode,Goff=Goff,Vknee=Vdiode);
initial equation
  assert(Ron > 0 and Rdiode > 0 and Vdiode >= 0 and Goff > 0,
    "Bridge requires positive Ron,Rdiode,Goff and nonnegative Vdiode");
equation
  connect(lowSwitch.p,phase);
  connect(lowSwitch.n,dc_n);
  connect(highDiode.p,phase);
  connect(highDiode.n,dc_p);
  connect(lowDiode.p,dc_n);
  connect(lowDiode.n,phase);
  lowSwitch.control = lowGate;
end PartialHalfBridge;
