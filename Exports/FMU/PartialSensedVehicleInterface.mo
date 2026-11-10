within ModelicaCPSLibrary.Exports.FMU;

partial model PartialSensedVehicleInterface "Vehicle interface v1 state and sensor outputs from a truth record and sensor-suite measurements"
  extends VehicleInterface.Base;
  extends VehicleInterface.KinematicState;
  extends VehicleInterface.Imu;
  extends VehicleInterface.Magnetometer;
  extends VehicleInterface.Gnss;
  extends VehicleInterface.Barometer;
  extends VehicleInterface.BarometerTiming;
  extends VehicleInterface.ExternalWrench;
  parameter Real R_bImu[3, 3] = identity(3) "IMU frame to body, as mounted in the plant";
  parameter Real R_bMag[3, 3] = identity(3) "Magnetometer frame to body, as mounted in the plant";
  output Interfaces.VehicleState truth "Simulation diagnostics; separate from sensor feedback";
  Interfaces.SensorMeasurements measurements "Sensor-frame values from the plant's sensor suite";
equation
  state.position_ned = truth.p_w;
  state.velocity_ned = truth.v_w;
  state.quaternion_wb = truth.q_wb;
  state.angular_velocity_frd = truth.omega_b;
  accel = R_bImu * measurements.acceleration;
  gyro = R_bImu * measurements.gyro;
  mag = 10000 * R_bMag * measurements.magneticField;
  gps[1] = lat0 + measurements.position[1] / earth_radius_m * 180 / Modelica.Constants.pi;
  gps[2] = lon0 + measurements.position[2] / (earth_radius_m * cos(lat0 * Modelica.Constants.pi / 180))
    * 180 / Modelica.Constants.pi;
  gps[3] = ground_alt_wgs84 - measurements.position[3];
  vel_ned = measurements.velocity;
  yaw_deg = atan2(truth.R_wb[2, 1], truth.R_wb[1, 1]) * 180 / Modelica.Constants.pi;
  baro_altitude_m = measurements.altitude;
  baro_pressure_pa = measurements.pressure;
  baro_temperature_c = measurements.temperature - 273.15;
  baro_climb_rate_mps = measurements.climbRate;
end PartialSensedVehicleInterface;
