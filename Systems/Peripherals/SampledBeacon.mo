within ModelicaCPSLibrary.Systems.Peripherals;
model SampledBeacon "B1: B0 at the update rate with horizontal and vertical noise, resolution and latency"
  extends PartialPeripheral;
  parameter Real samplePeriod(unit="s") = 0.125;
  parameter Real startTime(unit="s") = 0;
  parameter Integer latencySamples(min=0) = 0;
  parameter Integer seed = 1;
  parameter Integer streamId = 391;
  parameter Real horizontalNoiseStd(unit="m") = 0;
  parameter Real verticalNoiseStd(unit="m") = 0;
  parameter Real quantum(unit="m") = 0;
  output Real position[3](each unit="m");
  output Boolean valid;
  output Real validity = if valid then 1 else 0 "1 if valid, else 0";
protected
  Beacon ideal(position_b=position_b,R_bs=R_bs);
  Sampler sampler(n=3,samplePeriod=samplePeriod,startTime=startTime,latencySamples=latencySamples,
    seed=seed,streamId=streamId,noiseStd={horizontalNoiseStd,horizontalNoiseStd,verticalNoiseStd},
    quantum=fill(quantum,3));
equation
  ideal.truth = truth;
  sampler.u = ideal.position;
  position = sampler.y;
  valid = sampler.valid;
end SampledBeacon;
