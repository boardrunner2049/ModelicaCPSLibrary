within ModelicaCPSLibrary.Systems.Peripherals;
function eulerQuaternion "Scalar-first body-to-NED quaternion of roll, pitch, yaw (ZYX)"
  input Real euler[3](each unit="rad");
  output Real q[4];
protected
  Real cr = cos(euler[1]/2);
  Real sr = sin(euler[1]/2);
  Real cp = cos(euler[2]/2);
  Real sp = sin(euler[2]/2);
  Real cy = cos(euler[3]/2);
  Real sy = sin(euler[3]/2);
algorithm
  q := {cr*cp*cy + sr*sp*sy, sr*cp*cy - cr*sp*sy, cr*sp*cy + sr*cp*sy, cr*cp*sy - sr*sp*cy};
end eulerQuaternion;
