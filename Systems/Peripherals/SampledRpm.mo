within ModelicaCPSLibrary.Systems.Peripherals;
model SampledRpm "R1: R0 counted over a gate: resolution of one pulse per gate, noise, latency"
  parameter Integer n(min=1) = 1;
  parameter Real samplePeriod(unit="s") = 0.1 "Gate time";
  parameter Real startTime(unit="s") = 0;
  parameter Integer latencySamples(min=0) = 0;
  parameter Integer seed = 1;
  parameter Integer streamId = 341;
  parameter Real pulsesPerRevolution = 1;
  parameter Real noiseStd = 0 "rpm";
  parameter Real scale = 0 "Scale-factor error";
  parameter Real maxRpm = 1e30 "Clipped";
  input Real speed[n](each unit="rad/s");
  output Real rpm[n];
  output Boolean valid;
  output Real validity = if valid then 1 else 0 "1 if valid, else 0";
protected
  Rpm ideal(n=n);
  Sampler sampler(n=n,samplePeriod=samplePeriod,startTime=startTime,latencySamples=latencySamples,
    seed=seed,streamId=streamId,scale=fill(scale,n),noiseStd=fill(noiseStd,n),
    quantum=fill(60/(pulsesPerRevolution*samplePeriod),n),lower=zeros(n),upper=fill(maxRpm,n));
equation
  ideal.speed=speed;
  sampler.u=ideal.rpm;
  rpm=sampler.y;
  valid=sampler.valid;
end SampledRpm;
