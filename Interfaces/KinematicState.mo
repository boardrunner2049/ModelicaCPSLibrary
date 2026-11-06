within ModelicaCPSLibrary.Interfaces;

record KinematicState "Vehicle kinematic truth at the combined CG (vehicle interface kinematic_state_v1)"
  Real position_ned[3](each unit="m") "CG position in local NED, relative to the geodetic origin (lat0, lon0, ground_alt_wgs84)";
  Real velocity_ned[3](each unit="m/s") "CG velocity in local NED";
  Real quaternion_wb[4](each unit="1") "Unit scalar-first Hamilton quaternion (w,x,y,z) rotating body FRD vectors into NED";
  Real angular_velocity_frd[3](each unit="rad/s") "Body angular velocity resolved in body FRD";
end KinematicState;
