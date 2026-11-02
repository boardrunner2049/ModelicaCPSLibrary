within ModelicaCPSLibrary.Interfaces;

record SensorMeasurements "Continuous ideal measurements, without attitude truth"
  Real acceleration[3](each unit="m/s2") "Specific force in IMU sensor frame";
  Real gyro[3](each unit="rad/s") "Angular velocity in IMU sensor frame";
  Real magneticField[3](each unit="T") "Field in magnetometer sensor frame";
  Real position[3](each unit="m") "GNSS antenna position in local NED";
  Real velocity[3](each unit="m/s") "GNSS antenna velocity in local NED";
  Real pressure(unit="Pa") "Ambient pressure with sensor bias";
  Real temperature(unit="K");
  Real altitude(unit="m") "Up-positive geometric altitude with bias";
  Real climbRate(unit="m/s") "Low-fidelity geometric vertical velocity";
end SensorMeasurements;
