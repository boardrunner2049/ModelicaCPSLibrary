within ModelicaCPSLibrary.Systems.Peripherals;
function eulerAngles "Roll, pitch, yaw (ZYX) of a body-to-NED rotation matrix"
  input Real R[3,3];
  output Real euler[3](each unit="rad");
algorithm
  euler := {atan2(R[3,2], R[3,3]), -asin(max(-1, min(1, R[3,1]))), atan2(R[2,1], R[1,1])};
end eulerAngles;
