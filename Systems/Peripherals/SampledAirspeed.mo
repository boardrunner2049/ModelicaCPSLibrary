within ModelicaCPSLibrary.Systems.Peripherals;
model SampledAirspeed "A1: A0 sampled; differential pressure with offset, drift, noise, resolution and full scale; temperature with offset, noise and resolution"
  extends PartialPeripheral;
  parameter Real pressure0(unit="Pa") = 101325 "Ambient pressure at the origin [environment]";
  parameter Real temperature0(unit="K") = 288.15 "Ambient temperature at the origin [environment]";
  parameter Real samplePeriod(unit="s") = 0.01 "[datasheet]";
  parameter Real startTime(unit="s") = 0 "[configuration]";
  parameter Integer latencySamples(min=0) = 0 "[datasheet]";
  parameter Integer seed = 1 "[configuration]";
  parameter Integer streamId = 321 "[configuration]";
  parameter Real pressureBias(unit="Pa") = 0 "Zero offset [datasheet]";
  parameter Real pressureDriftStd(unit="Pa") = 0 "[datasheet]";
  parameter Real pressureDriftTau(unit="s") = 300 "[assumed]";
  parameter Real pressureNoiseStd(unit="Pa") = 0 "[datasheet]";
  parameter Real pressureQuantum(unit="Pa") = 0 "[datasheet]";
  parameter Real fullScale(unit="Pa") = 1e30 "Clipped to +-fullScale [datasheet]";
  parameter Real temperatureBias(unit="K") = 0 "[datasheet]";
  parameter Real temperatureNoiseStd(unit="K") = 0 "[datasheet]";
  parameter Real temperatureQuantum(unit="K") = 0 "[datasheet]";
  input Real wind_w[3](each unit="m/s") = zeros(3) "Air velocity over ground, NED";
  output Real differentialPressure(unit="Pa");
  output Real temperature(unit="K");
  output Boolean valid;
  output Real validity = if valid then 1 else 0 "1 if valid, else 0";
protected
  Airspeed ideal(position_b=position_b,R_bs=R_bs,pressure0=pressure0,temperature0=temperature0,
    wind_w=wind_w);
  Sampler sampler(n=2,samplePeriod=samplePeriod,startTime=startTime,latencySamples=latencySamples,
    seed=seed,streamId=streamId,bias={pressureBias,temperatureBias},
    driftStd={pressureDriftStd,0},driftTau={pressureDriftTau,300},
    noiseStd={pressureNoiseStd,temperatureNoiseStd},quantum={pressureQuantum,temperatureQuantum},
    lower={-fullScale,-1e30},upper={fullScale,1e30});
equation
  ideal.truth=truth;
  sampler.u={ideal.differentialPressure,ideal.temperature};
  differentialPressure=sampler.y[1];
  temperature=sampler.y[2];
  valid=sampler.valid;
end SampledAirspeed;
