within ModelicaCPSLibrary.Physical.Electrical.Electromechanical;
function trapezoid "Unit peak back-EMF; positive plateau at 30 to 150 electrical degrees"
  input Modelica.Units.SI.Angle theta;
  output Real y;
protected
  Real x;
algorithm
  x := mod(theta,2*Modelica.Constants.pi)/(Modelica.Constants.pi/3);
  y := if x < 0.5 then 2*x else if x < 2.5 then 1 else if x < 3.5 then 6-2*x
    else if x < 5.5 then -1 else 2*x-12;
end trapezoid;
