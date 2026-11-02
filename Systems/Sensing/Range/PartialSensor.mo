within ModelicaCPSLibrary.Systems.Sensing.Range;
partial model PartialSensor "Unit world ray and plane n*x=h; caller owns mount transform"
  parameter Real minRange(unit="m")=0;
  parameter Real maxRange(unit="m")=100;
  parameter Real parallelTolerance=1e-9 "Absolute cosine to plane normal";
  input Real position_w[3](each unit="m");
  input Real direction_w[3]={0,0,1} "Unit world NED ray; default points Down";
  input Real planeNormal_w[3]={0,0,1} "Nonzero; need not be normalized";
  input Real planeOffset(unit="m")=0 "n*x=h; scale with the normal";
  output Real distance(unit="m") "Zero when invalid; always check valid/status";
  output Boolean valid;
  output Integer status;
initial equation
  assert(minRange>=0 and maxRange>minRange,"Invalid range limits");
  assert(parallelTolerance>0 and parallelTolerance<1,"Invalid parallel tolerance");
equation
  assert(abs(direction_w*direction_w-1)<1e-8,"Range direction must be a unit vector");
  assert(planeNormal_w*planeNormal_w>Modelica.Constants.small,"Plane normal must be nonzero");
end PartialSensor;
