within ModelicaCPSLibrary.Systems.Peripherals;
model SampledPrecisionLanding "K1: K0 per frame: angle noise and pixel resolution, distance noise, latency; invalid frames outside the view"
  extends PartialPeripheral;
  parameter Real targetPosition_w[3](each unit="m") = zeros(3) "[configuration]";
  parameter Real fieldOfViewX(unit="rad") = 1.047 "[datasheet]";
  parameter Real fieldOfViewY(unit="rad") = 0.785 "[datasheet]";
  parameter Real maxRange(unit="m") = 15 "[datasheet]";
  parameter Real samplePeriod(unit="s") = 0.02 "Frame period [datasheet]";
  parameter Real startTime(unit="s") = 0 "[configuration]";
  parameter Integer latencySamples(min=0) = 0 "[datasheet]";
  parameter Integer seed = 1 "[configuration]";
  parameter Integer streamId = 401 "[configuration]";
  parameter Real angleNoiseStd(unit="rad") = 0 "[datasheet]";
  parameter Real angleQuantum(unit="rad") = 0 "One pixel [datasheet]";
  parameter Real distanceNoiseStd(unit="m") = 0 "[datasheet]";
  output Real angle[2](each unit="rad");
  output Real distance(unit="m");
  output Boolean valid;
  output Real validity = if valid then 1 else 0 "1 if valid, else 0";
protected
  PrecisionLanding ideal(position_b=position_b,R_bs=R_bs,targetPosition_w=targetPosition_w,
    fieldOfViewX=fieldOfViewX,fieldOfViewY=fieldOfViewY,maxRange=maxRange);
  Sampler sampler(available=ideal.valid,n=3,samplePeriod=samplePeriod,startTime=startTime,latencySamples=latencySamples,
    seed=seed,streamId=streamId,noiseStd={angleNoiseStd,angleNoiseStd,distanceNoiseStd},
    quantum={angleQuantum,angleQuantum,0});
equation
  ideal.truth = truth;
  sampler.u = cat(1, ideal.angle, {ideal.distance});
  angle = sampler.y[1:2];
  distance = sampler.y[3];
  valid = sampler.valid;
end SampledPrecisionLanding;
