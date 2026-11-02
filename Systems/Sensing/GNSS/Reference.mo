within ModelicaCPSLibrary.Systems.Sensing.GNSS;
record Reference "WGS84 geodetic origin of the fixed local NED frame"
  parameter Real latitude(unit="rad")=0;
  parameter Real longitude(unit="rad")=0;
  parameter Real altitude(unit="m")=0 "Ellipsoidal height, not mean sea level";
end Reference;
