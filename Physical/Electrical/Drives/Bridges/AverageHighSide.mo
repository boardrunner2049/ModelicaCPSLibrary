within ModelicaCPSLibrary.Physical.Electrical.Drives.Bridges;
model AverageHighSide "Power-conserving CCM high-side PWM reduction with asynchronous freewheel loss"
  parameter Modelica.Units.SI.Resistance Ron;
  parameter Modelica.Units.SI.Resistance Rdiode;
  parameter Modelica.Units.SI.Voltage Vdiode;
  Modelica.Electrical.Analog.Interfaces.PositivePin dc_p;
  Modelica.Electrical.Analog.Interfaces.NegativePin dc_n;
  Modelica.Electrical.Analog.Interfaces.PositivePin phase;
  input Real modulation;
  input Boolean enabled;
  Modelica.Units.SI.Current current;
  Modelica.Units.SI.Voltage meanVoltage;
  Modelica.Units.SI.Resistance meanResistance;
equation
  meanVoltage = modulation*(dc_p.v-dc_n.v)-(1-modulation)*Vdiode;
  meanResistance = modulation*Ron+(1-modulation)*Rdiode;
  current = if enabled then noEvent(max(0,(meanVoltage-(phase.v-dc_n.v))/meanResistance)) else 0;
  dc_p.i = modulation*current;
  dc_n.i = (1-modulation)*current;
  phase.i = -current;
  annotation(Documentation(info="<html><p>Asynchronous high-side PWM, continuous-conduction mean: v=d*Vdc-(1-d)*Vdiode-[d*Ron+(1-d)*Rdiode]*i and Idc=d*i. Current blocking at zero is an algebraic approximation, not PWM-cycle DCM reconstruction. External body diodes handle reverse current. No claim of switching-average agreement during discontinuous current, commutation or large bus ripple.</p></html>"));
end AverageHighSide;
