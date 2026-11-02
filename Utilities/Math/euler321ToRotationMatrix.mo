within ModelicaCPSLibrary.Utilities.Math;

function euler321ToRotationMatrix
  input Real euler[3] "Roll, pitch, yaw [rad]";
  output Real R[3, 3] "Rotation matrix from body frame to world frame";

protected
  Real cr;
  Real sr;
  Real cp;
  Real sp;
  Real cy;
  Real sy;

algorithm
  cr := cos(euler[1]);
  sr := sin(euler[1]);
  cp := cos(euler[2]);
  sp := sin(euler[2]);
  cy := cos(euler[3]);
  sy := sin(euler[3]);

  R[1, 1] := cy * cp;
  R[1, 2] := cy * sp * sr - sy * cr;
  R[1, 3] := cy * sp * cr + sy * sr;
  R[2, 1] := sy * cp;
  R[2, 2] := sy * sp * sr + cy * cr;
  R[2, 3] := sy * sp * cr - cy * sr;
  R[3, 1] := -sp;
  R[3, 2] := cp * sr;
  R[3, 3] := cp * cr;
end euler321ToRotationMatrix;
