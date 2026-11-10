within ModelicaCPSLibrary.Systems.Sensing;

model SensorSuite "Four-sensor assembly with independently replaceable IMU, magnetometer, GNSS and barometer"
  parameter Real rImu_b[3] = zeros(3) "IMU displacement from total CG [m]";
  parameter Real R_bImu[3,3] = identity(3) "IMU frame to body";
  parameter Real R_bMag[3,3] = identity(3) "Magnetometer frame to body";
  parameter Real rGnss_b[3] = zeros(3) "Antenna displacement from total CG [m]";
  parameter Real rBarometer_b[3] = zeros(3) "Barometer displacement from total CG [m]";
  parameter Real accelBias[3] = zeros(3);
  parameter Real gyroBias[3] = zeros(3);
  parameter Real magBias[3] = zeros(3);
  parameter Real positionBias[3] = zeros(3);
  parameter Real velocityBias[3] = zeros(3);
  parameter Real pressureBias = 0;
  parameter Real temperatureBias = 0;
  parameter Real altitudeBias = 0;
  parameter Boolean altitudeDependentAtmosphere = true
    "Barometer ambient pressure and temperature follow the ISA troposphere from pressure and temperature at the local origin";
  input Real p_w[3](each unit="m");
  input Real v_w[3](each unit="m/s");
  input Real R_wb[3,3](each unit="1");
  input Real a_w[3](each unit="m/s2");
  input Real omega_b[3](each unit="rad/s");
  input Real alpha_b[3](each unit="rad/s2");
  input Real gravity_w[3](each unit="m/s2");
  input Real magneticField_w[3](each unit="T");
  input Real pressure(unit="Pa") "Ambient pressure at the local origin (NED z = 0) [Pa]";
  input Real temperature(unit="K") "Ambient temperature at the local origin [K]";
  output ModelicaCPSLibrary.Interfaces.SensorMeasurements measurements;
  output Real barometerSampleTime(unit="s") "Acquisition time of the delivered barometer reading; -1 before the first";
  output Real barometerDeliveryTime(unit="s") "Delivery time of that reading; -1 before the first";
  output Integer barometerSequence "Delivered barometer readings; increments on each new one";
  output Boolean barometerValid "A valid barometer reading has been delivered";

  replaceable model ImuModel = IMU.Sensor constrainedby IMU.PartialSensor;
  ImuModel imu( r_b=rImu_b, R_bs=R_bImu,
    accelBias=accelBias, gyroBias=gyroBias);
  replaceable model MagnetometerModel = Magnetometer.Sensor constrainedby Magnetometer.PartialSensor;
  MagnetometerModel magnetometer( R_bs=R_bMag, bias=magBias);
  replaceable model GnssModel = GNSS.Sensor constrainedby GNSS.PartialSensor;
  GnssModel gnss( positionBias=positionBias, velocityBias=velocityBias);
  replaceable model BarometerModel = Barometer.Sensor constrainedby Barometer.PartialSensor;
  BarometerModel barometer( pressureBias=pressureBias,
    temperatureBias=temperatureBias, altitudeBias=altitudeBias);
protected
  Real barometerPosition_w[3];
  Real barometerVelocity_w[3];
equation
  imu.R_wb = R_wb;
  imu.a_w = a_w;
  imu.gravity_w = gravity_w;
  imu.omega_b = omega_b;
  imu.alpha_b = alpha_b;
  magnetometer.R_wb = R_wb;
  magnetometer.magneticField_w = magneticField_w;
  gnss.position_w = p_w + R_wb * rGnss_b;
  gnss.velocity_w = v_w + R_wb * cross(omega_b, rGnss_b);
  barometerPosition_w = p_w + R_wb * rBarometer_b;
  barometerVelocity_w = v_w + R_wb * cross(omega_b, rBarometer_b);
  if altitudeDependentAtmosphere then
    (barometer.ambientPressure, barometer.ambientTemperature) =
      Worlds.Environment.troposphere(-barometerPosition_w[3], pressure, temperature);
  else
    barometer.ambientPressure = pressure;
    barometer.ambientTemperature = temperature;
  end if;
  barometer.altitude_w = -barometerPosition_w[3];
  barometer.climbRate_w = -barometerVelocity_w[3];
  measurements.acceleration = imu.acceleration;
  measurements.gyro = imu.gyro;
  measurements.magneticField = magnetometer.magneticField;
  measurements.position = gnss.position;
  measurements.velocity = gnss.velocity;
  measurements.pressure = barometer.pressure;
  measurements.temperature = barometer.temperature;
  measurements.altitude = barometer.altitude;
  measurements.climbRate = barometer.climbRate;
  barometerSampleTime = barometer.sampleTime;
  barometerDeliveryTime = barometer.deliveryTime;
  barometerSequence = barometer.sequence;
  barometerValid = barometer.valid;
end SensorSuite;
