within ModelicaCPSLibrary.Systems.Sensing.Range;
model Sensor "L0: ideal continuous ray/plane intersection with explicit return status"
  extends PartialSensor;
  output Real geometricRange(unit="m") "Diagnostic signed ray parameter; zero for parallel rays";
protected
  Real denominator;
  Boolean parallelRay;
equation
  denominator=planeNormal_w*direction_w;
  parallelRay=abs(denominator)<=parallelTolerance*sqrt(planeNormal_w*planeNormal_w);
  geometricRange=if parallelRay then 0 else (planeOffset-planeNormal_w*position_w)/denominator;
  status=if parallelRay or geometricRange<0 then NoReturn else
    if geometricRange<minRange then TooClose else if geometricRange>maxRange then TooFar else Valid;
  valid=status==Valid;
  distance=if valid then geometricRange else 0;
end Sensor;
