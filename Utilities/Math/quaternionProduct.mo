within ModelicaCPSLibrary.Utilities.Math;

function quaternionProduct
  input Real q1[4] "Left quaternion {w, x, y, z}";
  input Real q2[4] "Right quaternion {w, x, y, z}";
  output Real q[4] "Hamilton product q1*q2";

algorithm
  q[1] := q1[1] * q2[1] - q1[2] * q2[2] - q1[3] * q2[3] - q1[4] * q2[4];
  q[2] := q1[1] * q2[2] + q1[2] * q2[1] + q1[3] * q2[4] - q1[4] * q2[3];
  q[3] := q1[1] * q2[3] - q1[2] * q2[4] + q1[3] * q2[1] + q1[4] * q2[2];
  q[4] := q1[1] * q2[4] + q1[2] * q2[3] - q1[3] * q2[2] + q1[4] * q2[1];
end quaternionProduct;
