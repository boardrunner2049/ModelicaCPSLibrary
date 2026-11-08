within ModelicaCPSLibrary.Exports.FMU.VehicleInterface;
partial model Magnetometer "magnetometer_v1: latest magnetometer reading"
  output Real mag[3] "Body FRD magnetic field [Gauss]";
  parameter Real earth_field_north_T(unit = "T") = 20.07e-6
    "Earth magnetic field at the geodetic origin, north; a host sets it for its location (default: WMM at the default origin)";
  parameter Real earth_field_east_T(unit = "T") = -1.52e-6 "Earth field at the origin, east";
  parameter Real earth_field_down_T(unit = "T") = 48.24e-6 "Earth field at the origin, down";
end Magnetometer;
