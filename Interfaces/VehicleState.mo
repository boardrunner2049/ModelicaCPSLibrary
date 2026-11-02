within ModelicaCPSLibrary.Interfaces;

record VehicleState "Continuous rigid-body state; independent of sensor measurements"
  Real p_w[3](each unit="m") "CG position in NED world";
  Real v_w[3](each unit="m/s") "CG velocity in NED world";
  Real v_b[3](each unit="m/s") "CG velocity resolved in FRD body";
  Real q_wb[4] "Scalar-first Hamilton quaternion, body to world";
  Real R_wb[3,3] "Rotation from body to world";
  Real omega_b[3](each unit="rad/s") "Body angular velocity";
  Real a_w[3](each unit="m/s2") "CG inertial acceleration in world";
  Real alpha_b[3](each unit="rad/s2") "Body angular acceleration";
end VehicleState;
