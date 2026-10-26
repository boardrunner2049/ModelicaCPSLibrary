within fire_modelica_models.Physical.Sensors.Electronics;
model CapacitiveReadout "Differential comb capacitance, carrier-averaged C/V, finite bandwidth and rails"
  parameter Real C0(unit="F");
  parameter Real length(unit="m");
  parameter Real carrierVoltage(unit="V");
  parameter Real Cf(unit="F");
  parameter Real tau(unit="s");
  parameter Real rail(unit="V");
  input Real displacement(unit="m");
  output Real capacitancePlus(unit="F");
  output Real capacitanceMinus(unit="F");
  output Real voltage(unit="V", nominal=0.002*carrierVoltage*C0/Cf);
  output Real measuredDisplacement(unit="m");
  output Boolean saturated;
  final parameter Real gain=2*carrierVoltage*C0/(length*Cf) "V/m";
protected
  Real idealVoltage(unit="V");
initial equation
  assert(C0>0 and length>0 and carrierVoltage>0 and Cf>0 and tau>0 and rail>0,"Invalid C/V readout");
  voltage=min(rail,max(-rail,idealVoltage));
equation
  capacitancePlus=C0*(1+displacement/length);
  capacitanceMinus=C0*(1-displacement/length);
  idealVoltage=-carrierVoltage*(capacitancePlus-capacitanceMinus)/Cf;
  tau*der(voltage)+voltage=min(rail,max(-rail,idealVoltage));
  measuredDisplacement=-voltage/gain;
  saturated=noEvent(abs(idealVoltage)>=rail);
  assert(noEvent(abs(displacement)<length),"Readout electrode stroke exceeded");
end CapacitiveReadout;
