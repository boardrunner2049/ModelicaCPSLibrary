within ModelicaCPSLibrary.Physical.Electrical.Passives;
model SeriesRL "Bidirectional linear series resistance and inductance"
  Modelica.Electrical.Analog.Interfaces.PositivePin p;
  Modelica.Electrical.Analog.Interfaces.NegativePin n;
  Modelica.Units.SI.Voltage v;
  Modelica.Units.SI.Current i;
  parameter Modelica.Units.SI.Resistance R;
  parameter Modelica.Units.SI.Inductance L;
  parameter Modelica.Units.SI.Current i_start = 0;
  parameter Boolean fixedCurrentStart = true "False permits dependent current initialization in a floating star";
  Modelica.Electrical.Analog.Basic.Resistor resistor(R=R);
  Modelica.Electrical.Analog.Basic.Inductor inductor(L=L,i(start=i_start,fixed=fixedCurrentStart));
  output Modelica.Units.SI.Power copperLoss;
  output Modelica.Units.SI.Energy magneticEnergy;
initial equation
  assert(R >= 0 and L > 0, "SeriesRL requires R >= 0 and L > 0");
equation
  connect(p,resistor.p);
  connect(resistor.n,inductor.p);
  connect(inductor.n,n);
  v = p.v-n.v;
  i = p.i;
  copperLoss = R*i*i;
  magneticEnergy = L*i*i/2;
  annotation(Documentation(info="<html><p>Two-terminal passive network: v=R*i+L*der(i). Both current signs are supported, including signed initial current. No motor, back-EMF, current limiting, grounding or drive-mode assumptions. Loss is reported without a thermal state.</p></html>"));
end SeriesRL;
