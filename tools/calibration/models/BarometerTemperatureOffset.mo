model BarometerTemperatureOffset "B1 plus a temperature-dependent pressure offset: the richer generator of the model-mismatch test"
  parameter Real temperatureCoefficient(unit="Pa/K") = 0.5 "DPS310 Table 7: pressure temperature sensitivity, 1000 hPa, 25..65 °C";
  parameter Real referenceTemperature(unit="K") = 298.15 "The datasheet's 25 °C";
  parameter Real samplePeriod(unit="s") = 0.02;
  parameter Real startTime(unit="s") = 0;
  parameter Integer seed = 1;
  parameter Integer streamId = 301;
  parameter Real pressureBias(unit="Pa") = 0;
  parameter Real temperatureBias(unit="K") = 0;
  parameter Real pressureNoiseStd(unit="Pa") = 0;
  parameter Real pressureDriftStd(unit="Pa") = 0;
  parameter Real pressureDriftTau(unit="s") = 300;
  parameter Real pressureQuantum(unit="Pa") = 0;
  parameter Real temperatureNoiseStd(unit="K") = 0;
  parameter Real temperatureQuantum(unit="K") = 0;
  input Real ambientPressure(unit="Pa");
  input Real ambientTemperature(unit="K");
  input Real altitude_w(unit="m");
  input Real climbRate_w(unit="m/s");
  output Real pressure(unit="Pa");
  output Real temperature(unit="K");
  output Real altitude(unit="m");
  output Real sampleTime(unit="s");
  output Real deliveryTime(unit="s");
  output Integer sequence;
  output Boolean valid;
  output Real pressureDrift(unit="Pa");
  ModelicaCPSLibrary.Systems.Sensing.Barometer.SampledSensor sensor(latencySamples=0,
    samplePeriod=samplePeriod, startTime=startTime, seed=seed, streamId=streamId,
    pressureBias=pressureBias, temperatureBias=temperatureBias, pressureNoiseStd=pressureNoiseStd,
    pressureDriftStd=pressureDriftStd, pressureDriftTau=pressureDriftTau, pressureQuantum=pressureQuantum,
    temperatureNoiseStd=temperatureNoiseStd, temperatureQuantum=temperatureQuantum);
equation
  // An additive offset: equivalent to adding it to the reading.
  sensor.ambientPressure = ambientPressure + temperatureCoefficient*(ambientTemperature - referenceTemperature);
  sensor.ambientTemperature = ambientTemperature;
  sensor.altitude_w = altitude_w;
  sensor.climbRate_w = climbRate_w;
  pressure = sensor.pressure;
  temperature = sensor.temperature;
  altitude = sensor.altitude;
  sampleTime = sensor.sampleTime;
  deliveryTime = sensor.deliveryTime;
  sequence = sensor.sequence;
  valid = sensor.valid;
  pressureDrift = sensor.pressureDrift;
end BarometerTemperatureOffset;
