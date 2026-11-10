within ModelicaCPSLibrary.Systems.Peripherals;
model SampledTemperature "T1: T0 through a first-order thermal lag, sampled with offset, noise and resolution"
  extends PartialPeripheral;
  parameter Real temperature0(unit="K") = 288.15 "[environment]";
  parameter Real selfHeating(unit="K") = 0 "[datasheet]";
  parameter Real timeConstant(unit="s") = 1 "Thermal response [datasheet]";
  parameter Real samplePeriod(unit="s") = 0.1 "[datasheet]";
  parameter Real startTime(unit="s") = 0 "[configuration]";
  parameter Integer latencySamples(min=0) = 0 "[datasheet]";
  parameter Integer seed = 1 "[configuration]";
  parameter Integer streamId = 361 "[configuration]";
  parameter Real bias(unit="K") = 0 "[datasheet]";
  parameter Real noiseStd(unit="K") = 0 "[datasheet]";
  parameter Real quantum(unit="K") = 0 "[datasheet]";
  output Real temperature(unit="K");
  output Boolean valid;
  output Real validity = if valid then 1 else 0 "1 if valid, else 0";
protected
  Temperature ideal(position_b=position_b,R_bs=R_bs,temperature0=temperature0,selfHeating=selfHeating);
  Real element(unit="K", start=temperature0 + selfHeating, fixed=true) "Sensing element";
  Sampler sampler(n=1,samplePeriod=samplePeriod,startTime=startTime,latencySamples=latencySamples,
    seed=seed,streamId=streamId,bias={bias},noiseStd={noiseStd},quantum={quantum});
equation
  ideal.truth = truth;
  der(element) = (ideal.temperature - element)/timeConstant;
  sampler.u = {element};
  temperature = sampler.y[1];
  valid = sampler.valid;
end SampledTemperature;
