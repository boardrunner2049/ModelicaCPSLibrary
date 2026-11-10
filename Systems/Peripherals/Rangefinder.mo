within ModelicaCPSLibrary.Systems.Peripherals;
model Rangefinder "L0: slant range along the boresight (device +z) to the ground plane, continuous"
  extends PartialPeripheral;
  parameter Real minRange(unit="m") = 0.1;
  parameter Real maxRange(unit="m") = 12;
  parameter Real groundDown(unit="m") = 0 "Ground plane, NED down coordinate";
  output Real distance(unit="m") "Zero when invalid";
  output Boolean valid;
  output Real reported(unit="m") "As a device reports it: the distance; 0 below the minimum range, maxRange + 1 beyond the maximum, -1 for no return or a device error";
  output Real validity = if valid then 1 else 0 "1 if valid, else 0";
protected
  Real boresight_w[3] "Unit boresight, NED";
  Sensing.Range.Sensor ray(minRange=minRange,maxRange=maxRange,position_w=mountPosition_w,
    direction_w=boresight_w,planeNormal_w={0,0,1},planeOffset=groundDown);
equation
  boresight_w=truth.R_wb*R_bs[:,3]/sqrt(R_bs[:,3]*transpose(truth.R_wb)*truth.R_wb*R_bs[:,3]);
  distance=ray.distance;
  valid=ray.valid;
  reported=if ray.valid then ray.distance else if ray.status==Sensing.Range.TooClose then 0
    else if ray.status==Sensing.Range.TooFar then maxRange+1 else -1;
end Rangefinder;
