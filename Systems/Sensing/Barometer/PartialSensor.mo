within ModelicaCPSLibrary.Systems.Sensing.Barometer;
partial model PartialSensor "Barometer value contract: pressure, temperature and altitude at the sensor"
  parameter Real pressureBias(unit="Pa") = 0;
  parameter Real temperatureBias(unit="K") = 0;
  parameter Real altitudeBias(unit="m") = 0;
  input Real ambientPressure(unit="Pa") "At the sensor";
  input Real ambientTemperature(unit="K") "At the sensor";
  input Real altitude_w(unit="m") "Geometric altitude above the local origin";
  input Real climbRate_w(unit="m/s");
  output Real pressure(unit="Pa");
  output Real temperature(unit="K");
  output Real altitude(unit="m");
  output Real climbRate(unit="m/s");
end PartialSensor;
