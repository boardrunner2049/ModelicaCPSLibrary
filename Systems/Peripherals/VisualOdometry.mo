within ModelicaCPSLibrary.Systems.Peripherals;
model VisualOdometry "V0: pose and velocity of the mount in an odometry frame aligned with NED at the origin (roll, pitch, yaw of the vehicle); truth, continuous"
  extends PartialPeripheral;
  output Real position[3](each unit="m") "NED";
  output Real velocity[3](each unit="m/s") "NED";
  output Real attitude[3](each unit="rad") "Roll, pitch, yaw of the vehicle";
  output Real quaternion[4];
  output Boolean valid = true;
  output Real validity = 1 "1 if valid, else 0";
equation
  position = mountPosition_w;
  velocity = truth.R_wb*mountVelocity_b;
  attitude = eulerAngles(truth.R_wb);
  quaternion = truth.q_wb;
end VisualOdometry;
