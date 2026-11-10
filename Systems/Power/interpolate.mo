within ModelicaCPSLibrary.Systems.Power;
function interpolate "Piecewise-linear y(x) through the points (xs, ys), xs increasing; held beyond the ends"
  input Real x;
  input Real xs[:];
  input Real ys[size(xs, 1)];
  output Real y;
algorithm
  y := ys[1];
  if x >= xs[size(xs, 1)] then
    y := ys[size(xs, 1)];
  elseif x > xs[1] then
    for i in 2:size(xs, 1) loop
      if x <= xs[i] and x > xs[i - 1] then
        y := ys[i - 1] + (ys[i] - ys[i - 1])*(x - xs[i - 1])/(xs[i] - xs[i - 1]);
      end if;
    end for;
  end if;
end interpolate;
