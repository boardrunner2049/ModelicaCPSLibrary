within ModelicaCPSLibrary.Systems.Power;
model DCLink "Shared ideal source, series resistance and DC-link capacitance"
  parameter Modelica.Units.SI.Resistance sourceResistance;
  parameter Modelica.Units.SI.Capacitance dcLinkCapacitance;
  parameter Modelica.Units.SI.Voltage voltage_start;
  input Modelica.Units.SI.Voltage sourceVoltage;
  Modelica.Electrical.Analog.Interfaces.PositivePin p;
  Modelica.Electrical.Analog.Interfaces.NegativePin n;
  Modelica.Electrical.Analog.Sources.SignalVoltage source;
  Modelica.Electrical.Analog.Basic.Resistor resistance(R=sourceResistance);
  Modelica.Electrical.Analog.Basic.Capacitor capacitor(C=dcLinkCapacitance,v(start=voltage_start,fixed=true));
  Modelica.Electrical.Analog.Basic.Ground ground;
  output Modelica.Units.SI.Voltage voltage;
  output Modelica.Units.SI.Current loadCurrent;
  output Modelica.Units.SI.Current sourceCurrent;
  output Modelica.Units.SI.Power sourcePower;
  output Modelica.Units.SI.Power lossPower;
  output Modelica.Units.SI.Energy storedEnergy;
initial equation
  assert(sourceResistance > 0 and dcLinkCapacitance > 0 and voltage_start >= 0,
    "DCLink requires positive sourceResistance,dcLinkCapacitance and nonnegative voltage_start");
equation
  connect(source.p,resistance.p);
  connect(resistance.n,p);
  connect(capacitor.p,p);
  connect(capacitor.n,n);
  connect(source.n,n);
  connect(n,ground.p);
  source.v = sourceVoltage;
  voltage = p.v-n.v;
  loadCurrent = -p.i;
  sourceCurrent = resistance.i;
  sourcePower = sourceVoltage*sourceCurrent;
  lossPower = sourceResistance*sourceCurrent*sourceCurrent;
  storedEnergy = dcLinkCapacitance*voltage*voltage/2;
  annotation(Documentation(info="<html><p>One shared bus per vehicle, not one battery per motor. The ideal source can absorb returned energy. This is not an SOC, battery chemistry, charge-limit or BMS model. sourceVoltage is the upstream source, not a prescribed bus voltage.</p></html>"));
end DCLink;
