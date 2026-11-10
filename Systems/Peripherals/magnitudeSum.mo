within ModelicaCPSLibrary.Systems.Peripherals;
function magnitudeSum "Sum of the magnitudes of the elements"
  input Real x[:];
  output Real y;
algorithm
  y := 0;
  for i in 1:size(x, 1) loop
    y := y + abs(x[i]);
  end for;
end magnitudeSum;
