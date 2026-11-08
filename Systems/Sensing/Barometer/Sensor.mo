within ModelicaCPSLibrary.Systems.Sensing.Barometer;

model Sensor "B0: ambient pressure/temperature and geometric altitude, continuous; no pressure-height inversion"
  extends PartialSensor;
equation
  pressure = ambientPressure + pressureBias;
  temperature = ambientTemperature + temperatureBias;
  altitude = altitude_w + altitudeBias;
  climbRate = climbRate_w;
end Sensor;
