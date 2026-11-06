within ModelicaCPSLibrary.Exports.FMU.VehicleInterface;
partial model KinematicState "kinematic_state_v1: physical truth at the combined CG, not a measurement"
  parameter Real lat0 = 40.414929 "Geodetic origin latitude [deg]";
  parameter Real lon0 = -86.932387 "Geodetic origin longitude [deg]";
  parameter Real ground_alt_wgs84 = 149 "Geodetic origin altitude above the WGS84 ellipsoid [m]; NED down is zero here";
  output Interfaces.KinematicState state "Kinematic truth; see Interfaces.KinematicState";
end KinematicState;
