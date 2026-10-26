within fire_modelica_models.Physical.Sensors.Electronics;
model CombDrive "Differential comb actuator: F = sum(V^2*dC/dx)/2"
  parameter Real C0(unit="F");
  parameter Real length(unit="m");
  parameter Real bias(unit="V");
  input Real displacement(unit="m");
  input Real command(unit="V");
  output Real force(unit="N");
  output Real capacitancePlus(unit="F");
  output Real capacitanceMinus(unit="F");
  output Real energy(unit="J");
initial equation
  assert(C0>0 and length>0 and bias>0,"Invalid comb electrode parameters");
equation
  capacitancePlus=C0*(1+displacement/length);
  capacitanceMinus=C0*(1-displacement/length);
  force=0.5*C0/length*((bias+command)^2-(bias-command)^2);
  energy=0.5*(capacitancePlus*(bias+command)^2+capacitanceMinus*(bias-command)^2);
  assert(noEvent(abs(displacement)<length),"Comb overlap exceeded; electrode model invalid");
end CombDrive;
