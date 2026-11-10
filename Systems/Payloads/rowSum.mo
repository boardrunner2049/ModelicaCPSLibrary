within ModelicaCPSLibrary.Systems.Payloads;
function rowSum "Sum of the rows of an n x 3 array"
  input Real x[:, 3];
  output Real y[3];
algorithm
  y := zeros(3);
  for i in 1:size(x, 1) loop
    y := y + x[i, :];
  end for;
end rowSum;
