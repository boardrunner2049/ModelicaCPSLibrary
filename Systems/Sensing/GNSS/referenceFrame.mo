within ModelicaCPSLibrary.Systems.Sensing.GNSS;
function referenceFrame "WGS84 origin and NED-to-ECEF rotation"
  input Reference reference;
  output Real origin[3];
  output Real R_ew[3,3];
protected
  Real N;
  constant Real e2=6.6943799901413165e-3;
algorithm
  assert(abs(reference.latitude)<=Modelica.Constants.pi/2,"Invalid geodetic latitude");
  N:=6378137/sqrt(1-e2*sin(reference.latitude)^2);
  origin:={(N+reference.altitude)*cos(reference.latitude)*cos(reference.longitude),
    (N+reference.altitude)*cos(reference.latitude)*sin(reference.longitude),
    (N*(1-e2)+reference.altitude)*sin(reference.latitude)};
  R_ew:=[-sin(reference.latitude)*cos(reference.longitude),-sin(reference.longitude),-cos(reference.latitude)*cos(reference.longitude);
    -sin(reference.latitude)*sin(reference.longitude),cos(reference.longitude),-cos(reference.latitude)*sin(reference.longitude);
    cos(reference.latitude),0,-sin(reference.latitude)];
end referenceFrame;
