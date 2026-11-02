within ModelicaCPSLibrary.Adapters.VirtualIO;
model SensorValues "Stateless Sensor-local to FRD; GNSS already NED value conversion"
  parameter Vehicles.Copter.Geometry geometry;
  input Interfaces.SensorMeasurements measurements;
  output Real acceleration_frd[3];
  output Real gyro_frd[3];
  output Real magneticField_frd[3] "Tesla";
  output Real position_ned[3];
  output Real velocity_ned[3];
equation
  acceleration_frd=geometry.R_bImu*measurements.acceleration;
  gyro_frd=geometry.R_bImu*measurements.gyro;
  magneticField_frd=geometry.R_bMag*measurements.magneticField;
  position_ned=measurements.position;
  velocity_ned=measurements.velocity;
end SensorValues;
