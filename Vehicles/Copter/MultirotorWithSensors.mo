within ModelicaCPSLibrary.Vehicles.Copter;
model MultirotorWithSensors "Default ideal sensors with independently replaceable IMU; no protocol/firmware driver"
  extends MultirotorPlant;
  input Real magneticField_w[3](each unit="T") = {20.07e-6, -1.52e-6, 48.24e-6}
    "Earth field at the local origin, NED (WMM at the default origin, West Lafayette)";
  input Real pressure(unit="Pa") = 101325 "Constant ambient pressure baseline [Pa]";
  input Real temperature(unit="K") = 288.15 "K";
  Systems.Sensing.SensorSuite sensors(
    rImu_b=geometry.imuPosition_C-chassis.cg_C,R_bImu=geometry.R_bImu,R_bMag=geometry.R_bMag,
    rGnss_b=geometry.gnssPosition_C-chassis.cg_C,rBarometer_b=geometry.barometerPosition_C-chassis.cg_C);
  output Interfaces.SensorMeasurements measurements;
  output Real barometerSampleTime(unit="s") = sensors.barometerSampleTime "Barometer timing, see Systems.Sensing.Barometer.PartialSensor";
  output Real barometerDeliveryTime(unit="s") = sensors.barometerDeliveryTime;
  output Integer barometerSequence = sensors.barometerSequence;
  output Boolean barometerValid = sensors.barometerValid;
initial equation
  assert(max(abs(transpose(geometry.R_bImu)*geometry.R_bImu-identity(3)))<1e-9,"IMU mount must be a proper rotation");
    // and abs(Modelica.Math.Matrices.det(geometry.R_bImu)-1)<1e-9
  assert(max(abs(transpose(geometry.R_bMag)*geometry.R_bMag-identity(3)))<1e-9,"Rotor mount must be a proper rotation");
    // and abs(Modelica.Math.Matrices.det(geometry.R_bMag)-1)<1e-9
equation
  sensors.p_w=body.p_w;
  sensors.v_w=body.v_w;
  sensors.R_wb=body.R_wb;
  sensors.a_w=body.a_w;
  sensors.omega_b=body.omega_b;
  sensors.alpha_b=body.alpha_b;
  sensors.gravity_w=gravity_w;
  sensors.magneticField_w=magneticField_w;
  sensors.pressure=pressure;
  sensors.temperature=temperature;
  measurements=sensors.measurements;
end MultirotorWithSensors;
