within fire_modelica_models.Systems.Sensing.IMU;
partial model PartialSensor "Rigid-mount IMU value contract; sensor-local SI outputs"
  parameter Real r_b[3](each unit="m") = zeros(3);
  parameter Real R_bs[3,3] = identity(3) "v_b = R_bs*v_s";
  parameter Real accelBias[3](each unit="m/s2") = zeros(3);
  parameter Real gyroBias[3](each unit="rad/s") = zeros(3);
  input Real R_wb[3,3];
  input Real a_w[3](each unit="m/s2");
  input Real gravity_w[3](each unit="m/s2");
  input Real omega_b[3](each unit="rad/s");
  input Real alpha_b[3](each unit="rad/s2");
  output Real acceleration[3](each unit="m/s2");
  output Real gyro[3](each unit="rad/s");
end PartialSensor;
