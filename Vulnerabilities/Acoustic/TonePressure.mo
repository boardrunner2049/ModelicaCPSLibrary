within fire_modelica_models.Vulnerabilities.Acoustic;
model TonePressure "Local instantaneous pressure from RMS amplitude; phase is at the sensor"
  parameter Real frequency(unit="Hz")=1000;
  parameter Real phase(unit="rad")=0;
  input Real rmsPressure(unit="Pa")=0;
  input Real envelope=1;
  output Real pressure(unit="Pa");
initial equation
  assert(frequency>0,"Acoustic tone frequency must be positive");
equation
  assert(rmsPressure>=0 and envelope>=0,"Negative RMS pressure or envelope");
  pressure=sqrt(2)*rmsPressure*envelope*cos(2*Modelica.Constants.pi*frequency*time+phase);
end TonePressure;
