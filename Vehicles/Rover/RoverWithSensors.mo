within ModelicaCPSLibrary.Vehicles.Rover;

model RoverWithSensors "Rover with the default ideal sensor suite; no protocol/firmware driver"
  extends RoverPlant;
  parameter Real imuPosition_b[3](each unit = "m") = zeros(3) "From the CG";
  parameter Real R_bImu[3, 3] = identity(3) "IMU frame to body";
  parameter Real R_bMag[3, 3] = identity(3) "Magnetometer frame to body";
  parameter Real gnssPosition_b[3](each unit = "m") = zeros(3) "Antenna, from the CG";
  parameter Real barometerPosition_b[3](each unit = "m") = zeros(3) "From the CG";
  input Real magneticField_w[3](each unit = "T") = {20.07e-6, -1.52e-6, 48.24e-6}
    "Earth field at the local origin, NED (WMM at the default origin, West Lafayette)";
  input Real pressure(unit = "Pa") = 101325 "Ambient pressure";
  input Real temperature(unit = "K") = 288.15;
  Systems.Sensing.SensorSuite sensors(rImu_b = imuPosition_b, R_bImu = R_bImu, R_bMag = R_bMag,
    rGnss_b = gnssPosition_b, rBarometer_b = barometerPosition_b);
  output Interfaces.SensorMeasurements measurements;
  output Real barometerSampleTime(unit="s") = sensors.barometerSampleTime "Barometer timing, see Systems.Sensing.Barometer.PartialSensor";
  output Real barometerDeliveryTime(unit="s") = sensors.barometerDeliveryTime;
  output Integer barometerSequence = sensors.barometerSequence;
  output Boolean barometerValid = sensors.barometerValid;
equation
  sensors.p_w = body.p_w;
  sensors.v_w = body.v_w;
  sensors.R_wb = body.R_wb;
  sensors.a_w = body.a_w;
  sensors.omega_b = body.omega_b;
  sensors.alpha_b = body.alpha_b;
  sensors.gravity_w = gravity_w;
  sensors.magneticField_w = magneticField_w;
  sensors.pressure = pressure;
  sensors.temperature = temperature;
  measurements = sensors.measurements;
end RoverWithSensors;
