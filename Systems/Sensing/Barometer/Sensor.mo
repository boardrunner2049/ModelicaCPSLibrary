within ModelicaCPSLibrary.Systems.Sensing.Barometer;

model Sensor "Ambient pressure/temperature and geometric altitude; no pressure-height inversion"
  parameter Real pressureBias(unit="Pa") = 0;
  parameter Real temperatureBias(unit="K") = 0;
  parameter Real altitudeBias(unit="m") = 0;
  input Real ambientPressure(unit="Pa");
  input Real ambientTemperature(unit="K");
  input Real altitude_w(unit="m");
  input Real climbRate_w(unit="m/s");
  output Real pressure(unit="Pa");
  output Real temperature(unit="K");
  output Real altitude(unit="m");
  output Real climbRate(unit="m/s");

equation
  pressure = ambientPressure + pressureBias;
  temperature = ambientTemperature + temperatureBias;
  altitude = altitude_w + altitudeBias;
  climbRate = climbRate_w;
end Sensor;
