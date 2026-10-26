within fire_modelica_models.Vulnerabilities.Acoustic;
model SoundPowerToPressure "Free-field isotropic acoustic power to RMS pressure, not electrical speaker power"
  parameter Real density(unit="kg/m3")=1.21;
  parameter Real soundSpeed(unit="m/s")=343;
  input Real acousticPower(unit="W")=0;
  input Real distance(unit="m")=1;
  output Real rmsPressure(unit="Pa");
initial equation
  assert(density>0 and soundSpeed>0,"Invalid acoustic medium");
equation
  assert(acousticPower>=0 and distance>0,"Acoustic power must be nonnegative and distance positive");
  rmsPressure=sqrt(density*soundSpeed*acousticPower/(4*Modelica.Constants.pi*distance^2));
end SoundPowerToPressure;
