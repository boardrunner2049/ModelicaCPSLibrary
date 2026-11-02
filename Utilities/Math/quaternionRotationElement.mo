within ModelicaCPSLibrary.Utilities.Math;

function quaternionRotationElement "Scalar entry of the normalized Hamilton body-to-world rotation"
  input Real q[4] "Scalar-first Hamilton quaternion, body to world";
  input Integer i "Row, 1 through 3";
  input Integer j "Column, 1 through 3";
  output Real value;
protected
  Real n;
  Real w;
  Real x;
  Real y;
  Real z;
algorithm
  // Preserve quaternionNormalize's epsilon behavior, including q = zero.
  // Scalar arithmetic also avoids array-return derivative code in OMC FMI.
  assert(i >= 1 and i <= 3 and j >= 1 and j <= 3, "Rotation index must be 1 through 3");
  n := max(sqrt(q[1]*q[1] + q[2]*q[2] + q[3]*q[3] + q[4]*q[4]),
    ModelicaCPSLibrary.Utilities.Constants.eps);
  w := q[1]/n;
  x := q[2]/n;
  y := q[3]/n;
  z := q[4]/n;
  if i == 1 then
    value := if j == 1 then 1-2*(y*y+z*z)
      elseif j == 2 then 2*(x*y-z*w) else 2*(x*z+y*w);
  elseif i == 2 then
    value := if j == 1 then 2*(x*y+z*w)
      elseif j == 2 then 1-2*(x*x+z*z) else 2*(y*z-x*w);
  else
    value := if j == 1 then 2*(x*z-y*w)
      elseif j == 2 then 2*(y*z+x*w) else 1-2*(x*x+y*y);
  end if;
end quaternionRotationElement;
