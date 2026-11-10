within ModelicaCPSLibrary.Systems.Peripherals;
model SampledOpticalFlow "F1: F0 per frame: flow noise and resolution, maximum flow rate, gyro noise, latency; invalid frames out of the range band"
  extends PartialPeripheral;
  parameter Real minRange(unit="m") = 0.08;
  parameter Real maxRange(unit="m") = 30;
  parameter Real groundDown(unit="m") = 0;
  parameter Real samplePeriod(unit="s") = 0.01 "Frame period of the reports";
  parameter Real startTime(unit="s") = 0;
  parameter Integer latencySamples(min=0) = 0;
  parameter Integer seed = 1;
  parameter Integer streamId = 331;
  parameter Real flowNoiseStd(unit="rad/s") = 0;
  parameter Real flowQuantum(unit="rad/s") = 0 "One pixel of displacement per frame";
  parameter Real maxFlowRate(unit="rad/s") = 1e30 "Clipped to +-maxFlowRate";
  parameter Real flowScale = 0 "Scale-factor error";
  parameter Real gyroNoiseStd(unit="rad/s") = 0;
  parameter Real gyroBias(unit="rad/s") = 0;
  output Real flowRate[2](each unit="rad/s");
  output Real bodyRate[2](each unit="rad/s");
  output Boolean valid;
  output Real validity = if valid then 1 else 0 "1 if valid, else 0";
protected
  OpticalFlow ideal(position_b=position_b,R_bs=R_bs,minRange=minRange,maxRange=maxRange,
    groundDown=groundDown);
  Sampler sampler(available=ideal.valid,n=4,samplePeriod=samplePeriod,startTime=startTime,latencySamples=latencySamples,
    seed=seed,streamId=streamId,scale={flowScale,flowScale,0,0},bias={0,0,gyroBias,gyroBias},
    noiseStd={flowNoiseStd,flowNoiseStd,gyroNoiseStd,gyroNoiseStd},
    quantum={flowQuantum,flowQuantum,0,0},lower={-maxFlowRate,-maxFlowRate,-1e30,-1e30},
    upper={maxFlowRate,maxFlowRate,1e30,1e30});
equation
  ideal.truth=truth;
  sampler.u=cat(1,ideal.flowRate,ideal.bodyRate);
  flowRate=sampler.y[1:2];
  bodyRate=sampler.y[3:4];
  valid=sampler.valid;
end SampledOpticalFlow;
