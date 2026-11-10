within ModelicaCPSLibrary.Systems.Peripherals;
model SampledPowerModule "W1: voltage and current sampled through the module's sensing: gain error, offset, noise, resolution, range"
  extends PartialPowerModule;
  parameter Real samplePeriod(unit="s") = 0.1 "[datasheet]";
  parameter Real startTime(unit="s") = 0 "[configuration]";
  parameter Integer latencySamples(min=0) = 0 "[datasheet]";
  parameter Integer seed = 1 "[configuration]";
  parameter Integer streamId = 351 "[configuration]";
  parameter Real voltageScale = 0 "Gain error [datasheet]";
  parameter Real voltageBias(unit="V") = 0 "[datasheet]";
  parameter Real voltageNoiseStd(unit="V") = 0 "[datasheet]";
  parameter Real voltageQuantum(unit="V") = 0 "[datasheet]";
  parameter Real maxVoltage(unit="V") = 1e30 "[datasheet]";
  parameter Real currentScale = 0 "Gain error [datasheet]";
  parameter Real currentBias(unit="A") = 0 "[datasheet]";
  parameter Real currentNoiseStd(unit="A") = 0 "[datasheet]";
  parameter Real currentQuantum(unit="A") = 0 "[datasheet]";
  parameter Real maxCurrent(unit="A") = 1e30 "[datasheet]";
protected
  Sampler sampler(n=2,samplePeriod=samplePeriod,startTime=startTime,latencySamples=latencySamples,
    seed=seed,streamId=streamId,scale={voltageScale,currentScale},bias={voltageBias,currentBias},
    noiseStd={voltageNoiseStd,currentNoiseStd},quantum={voltageQuantum,currentQuantum},
    lower={0,-maxCurrent},upper={maxVoltage,maxCurrent});
equation
  sampler.u = {battery.voltage, packCurrent};
  voltage = sampler.y[1];
  current = sampler.y[2];
  valid = sampler.valid;
end SampledPowerModule;
