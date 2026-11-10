within ModelicaCPSLibrary.Systems.Peripherals;
model SampledRangefinder "L1: sampled slant range with bias, range-proportional noise, quantization, limits and latency"
  extends PartialPeripheral;
  parameter Real minRange(unit="m") = 0.1 "[datasheet]";
  parameter Real maxRange(unit="m") = 12 "[datasheet]";
  parameter Real groundDown(unit="m") = 0 "Ground plane, NED down coordinate [environment]";
  parameter Real samplePeriod(unit="s") = 0.02 "[datasheet]";
  parameter Real startTime(unit="s") = 0 "[configuration]";
  parameter Integer latencySamples(min=0) = 0 "[datasheet]";
  parameter Integer seed = 1 "[configuration]";
  parameter Integer streamId = 311 "[configuration]";
  parameter Real bias(unit="m") = 0 "[datasheet]";
  parameter Real noiseStd(unit="m") = 0 "Per-sample standard deviation at zero range [datasheet]";
  parameter Real noiseSlope = 0 "Added standard deviation per metre of range [datasheet]";
  parameter Real quantum(unit="m") = 0 "Resolution; 0 for none [datasheet]";
  output Real distance(unit="m") "Zero when invalid";
  output Boolean valid;
  output Real reported(unit="m") "As a device reports it: the distance; 0 below the minimum range, maxRange + 1 beyond the maximum, -1 for no return or a device error";
  output Real validity = if valid then 1 else 0 "1 if valid, else 0";
  output Integer sequence;
protected
  Real boresight_w[3] "Unit boresight, NED";
  Sensing.Range.SampledSensor ray(minRange=minRange,maxRange=maxRange,position_w=mountPosition_w,
    direction_w=boresight_w,planeNormal_w={0,0,1},planeOffset=groundDown,
    samplePeriod=samplePeriod,startTime=startTime,latencySamples=latencySamples,seed=seed,
    streamId=streamId,bias=bias,noiseStd=noiseStd,noiseSlope=noiseSlope,quantum=quantum);
equation
  boresight_w=truth.R_wb*R_bs[:,3]/sqrt(R_bs[:,3]*transpose(truth.R_wb)*truth.R_wb*R_bs[:,3]);
  distance=ray.distance;
  valid=ray.valid;
  reported=if ray.valid then ray.distance else if ray.status==Sensing.Range.TooClose then 0
    else if ray.status==Sensing.Range.TooFar then maxRange+1 else -1;
  sequence=ray.sequence;
end SampledRangefinder;
