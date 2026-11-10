within ModelicaCPSLibrary.Systems.Peripherals;
model ExternalAhrs "E0: an external AHRS/INS reporting the vehicle's attitude (roll, pitch, yaw and quaternion, body FRD to NED), body rates and specific force at the mount, and the mount's NED position and velocity; truth, continuous"
  extends PartialPeripheral;
  parameter Real gravity(unit="m/s2") = 9.80665;
  output Real attitude[3](each unit="rad") "Roll, pitch, yaw";
  output Real quaternion[4] "Scalar-first, body FRD to NED";
  output Real rate[3](each unit="rad/s") "Body FRD";
  output Real accel[3](each unit="m/s2") "Specific force, body FRD";
  output Real position[3](each unit="m") "NED from the origin";
  output Real velocity[3](each unit="m/s") "NED";
  output Boolean valid = true;
  output Real validity = 1 "1 if valid, else 0";
equation
  attitude = eulerAngles(truth.R_wb);
  quaternion = truth.q_wb;
  rate = truth.omega_b;
  accel = transpose(truth.R_wb)*(truth.a_w - {0, 0, gravity})
    + cross(truth.alpha_b, position_b) + cross(truth.omega_b, cross(truth.omega_b, position_b));
  position = mountPosition_w;
  velocity = truth.R_wb*mountVelocity_b;
end ExternalAhrs;
