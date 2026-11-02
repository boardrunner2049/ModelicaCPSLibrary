within ModelicaCPSLibrary.Utilities.Math;
function regularPolygon "Positions about +z, first point at phase (radians)"
  input Integer n;
  input Real radius;
  input Real height = 0;
  input Real phase = 0;
  output Real positions[n,3];
algorithm
  for i in 1:n loop
    positions[i,:] := {radius*cos(phase + 2*Modelica.Constants.pi*(i-1)/n),
      radius*sin(phase + 2*Modelica.Constants.pi*(i-1)/n), height};
  end for;
end regularPolygon;
