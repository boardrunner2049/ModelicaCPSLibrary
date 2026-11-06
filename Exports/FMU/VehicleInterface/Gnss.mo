within ModelicaCPSLibrary.Exports.FMU.VehicleInterface;
partial model Gnss "gnss_v1: latest GNSS fix"
  parameter Real earth_radius_m = 6378137 "Spherical Earth radius of the local geodetic conversion [m]";
  output Real gps[3] "Latitude [deg], longitude [deg], altitude above the WGS84 ellipsoid [m]";
  output Real vel_ned[3](each unit="m/s") "Velocity in NED [m/s]";
  output Real yaw_deg(unit="deg") "Heading clockwise from north [deg]";
end Gnss;
