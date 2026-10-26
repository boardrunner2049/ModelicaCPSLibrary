within fire_modelica_models.Systems.Sensing.IMU;

model Sensor "Continuous specific force and rate for a sensor rigidly mounted on the body"
  extends PartialSensor;

protected
  Real specificForce_b[3](each unit="m/s2");
initial equation
  assert(max(abs(transpose(R_bs)*R_bs-identity(3)))<1e-9,"Sensor mount must be a proper rotation");
    // and abs(Modelica.Math.Matrices.det(R_bs)-1)<1e-9
equation
  specificForce_b = transpose(R_wb)*(a_w-gravity_w)
    + cross(alpha_b,r_b) + cross(omega_b,cross(omega_b,r_b));
  acceleration = transpose(R_bs)*specificForce_b + accelBias;
  gyro = transpose(R_bs)*omega_b + gyroBias;
end Sensor;
