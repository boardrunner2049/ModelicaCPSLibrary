within ModelicaCPSLibrary.Utilities.Math;

function quaternionToRotationMatrix
  input Real q[4] "Quaternion {w, x, y, z}, body to world";
  output Real R[3, 3] "Rotation matrix from body frame to world frame";

algorithm
  for i in 1:3 loop
    for j in 1:3 loop
      R[i,j] := quaternionRotationElement(q,i,j);
    end for;
  end for;
end quaternionToRotationMatrix;
