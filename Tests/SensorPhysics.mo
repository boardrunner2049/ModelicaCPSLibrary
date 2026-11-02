within ModelicaCPSLibrary.Tests;

model SensorPhysics "Specific force, rigid lever arm, sensor rotation, and bias checks"
  ModelicaCPSLibrary.Systems.Sensing.IMU.Sensor freefall;
  ModelicaCPSLibrary.Systems.Sensing.IMU.Sensor supported;
  ModelicaCPSLibrary.Systems.Sensing.IMU.Sensor lever(
    r_b={1,0,0}, R_bs=[0,-1,0; 1,0,0; 0,0,1],
    accelBias={0.1,0.2,0.3}, gyroBias={0.1,0.2,0.3});
equation
  freefall.R_wb = identity(3);
  freefall.a_w = {0,0,9.81};
  freefall.gravity_w = {0,0,9.81};
  freefall.omega_b = zeros(3);
  freefall.alpha_b = zeros(3);
  supported.R_wb = identity(3);
  supported.a_w = zeros(3);
  supported.gravity_w = {0,0,9.81};
  supported.omega_b = zeros(3);
  supported.alpha_b = zeros(3);
  lever.R_wb = identity(3);
  lever.a_w = zeros(3);
  lever.gravity_w = {0,0,9.81};
  lever.omega_b = {1,2,3};
  lever.alpha_b = {0,0,3};
  when time >= 0.005 then
    assert(sum(abs(freefall.acceleration)) < 1e-10,
      "CG accelerometer must read zero in freefall");
    assert(sum(abs(supported.acceleration - {0,0,-9.81})) < 1e-10,
      "Supported FRD accelerometer must read negative g on z");
    assert(sum(abs(lever.acceleration - {5.1,13.2,-6.51})) < 1e-10,
      "Lever arm tangential/centripetal acceleration, mounting rotation, or bias is incorrect");
    assert(sum(abs(lever.gyro - {2.1,-0.8,3.3})) < 1e-10,
      "Gyroscope mounting rotation or bias is incorrect");
  end when;
  annotation(experiment(StopTime=0.025, Interval=0.001, Tolerance=1e-8));
end SensorPhysics;
