within ModelicaCPSLibrary.Systems.Sensing.Barometer;
partial model PartialSensor "Barometer value contract: pressure, temperature and altitude at the sensor"
  parameter Real pressureBias(unit="Pa") = 0 "Constant offset of this unit; the datasheet's absolute accuracy bounds it, a calibration measures it [datasheet]";
  parameter Real temperatureBias(unit="K") = 0 "Constant temperature offset of this unit; bounded by the temperature accuracy [datasheet]";
  parameter Real altitudeBias(unit="m") = 0 "Offset of the convenience altitude output [configuration]";
  input Real ambientPressure(unit="Pa") "At the sensor";
  input Real ambientTemperature(unit="K") "At the sensor";
  input Real altitude_w(unit="m") "Geometric altitude above the local origin";
  input Real climbRate_w(unit="m/s");
  output Real pressure(unit="Pa");
  output Real temperature(unit="K");
  output Real altitude(unit="m");
  output Real climbRate(unit="m/s");
  output Real sampleTime(unit="s") "Acquisition time of the delivered reading; -1 before the first";
  output Real deliveryTime(unit="s") "Time the delivered reading became available; -1 before the first";
  output Integer sequence "Delivered readings so far; it increments exactly when a new reading is delivered";
  output Boolean valid "A reading has been delivered and is valid";
  annotation(Documentation(info="<html><p>Timing contract, in simulation time. A consumer sees a new
reading when <code>sequence</code> increments; between deliveries the outputs hold the last reading.
<code>sampleTime</code> is when the delivered reading was acquired, <code>deliveryTime</code> when it
became available (acquisition plus latency). Before the first delivery <code>valid</code> is false,
<code>sequence</code> is 0 and both times are -1. A continuous level (B0) is valid throughout, with
both times equal to <code>time</code> and <code>sequence</code> 0: it has no discrete readings.
Simulation start (or an FMU reset) restarts the clock, the queue and the noise streams.</p></html>"));
end PartialSensor;
