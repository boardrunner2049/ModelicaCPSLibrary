within fire_modelica_models.Physical.Sensors.MEMS;
model ResonantCoupling "Identified pressure-driven mechanical modes; no vendor-specific gyro demodulator"
  parameter Real frequency[6](each unit="Hz");
  parameter Real damping[6] "Damping ratio";
  parameter Real pressureGain[6] "Modal acceleration per Pa [m/(s2.Pa)]";
  parameter Real displacementGain[6] "Readout channel units/m";
  parameter Real velocityGain[6] = zeros(6) "Readout channel units/(m/s)";
  parameter Real quadraticGain[6] = zeros(6) "Readout channel units/m2";
  input Real pressure(unit="Pa");
  output Real error[6] "Equivalent acceleration xyz [m/s2], then rate xyz [rad/s]";
  Real x[6](each unit="m",each start=0,each fixed=true);
  Real v[6](each unit="m/s",each start=0,each fixed=true);
protected
  final parameter Real w[6] = 2*Modelica.Constants.pi*frequency;
initial equation
  assert(min(frequency)>0 and min(damping)>0,"Acoustic mode frequency and damping must be positive");
equation
  der(x)=v;
  der(v)+2*damping .* w .* v+w .* w .* x=pressureGain*pressure;
  error=displacementGain .* x+velocityGain .* v+quadraticGain .* x .* x;
end ResonantCoupling;
