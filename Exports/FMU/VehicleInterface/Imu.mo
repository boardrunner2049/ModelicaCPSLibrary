within ModelicaCPSLibrary.Exports.FMU.VehicleInterface;
partial model Imu "imu_v1: latest IMU reading at the communication point"
  output Real accel[3](each unit="m/s2") "Body FRD specific force [m/s2]";
  output Real gyro[3](each unit="rad/s") "Body FRD angular rate [rad/s]";
end Imu;
