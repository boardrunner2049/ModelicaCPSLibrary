within ModelicaCPSLibrary.Utilities.Math;

function euler321ToQuaternion
  input Real euler[3] "Roll, pitch, yaw [rad]";
  output Real q[4] "Unit quaternion {w, x, y, z}, body to world";

protected
  Real cr;
  Real sr;
  Real cp;
  Real sp;
  Real cy;
  Real sy;

algorithm
  cr := cos(0.5 * euler[1]);
  sr := sin(0.5 * euler[1]);
  cp := cos(0.5 * euler[2]);
  sp := sin(0.5 * euler[2]);
  cy := cos(0.5 * euler[3]);
  sy := sin(0.5 * euler[3]);

  q[1] := cy * cp * cr + sy * sp * sr;
  q[2] := cy * cp * sr - sy * sp * cr;
  q[3] := sy * cp * sr + cy * sp * cr;
  q[4] := sy * cp * cr - cy * sp * sr;
  q := quaternionNormalize(q);
end euler321ToQuaternion;
