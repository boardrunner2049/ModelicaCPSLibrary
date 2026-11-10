within ModelicaCPSLibrary.Systems.Peripherals;
model SampledExternalAhrs "E1: E0 at the output rate with the unit's errors: roll/pitch and heading noise and Gauss-Markov drift, gyro and accelerometer noise and bias drift, position and velocity noise and drift, latency"
  extends PartialPeripheral;
  parameter Real gravity(unit="m/s2") = 9.80665 "[physics]";
  parameter Real samplePeriod(unit="s") = 0.01 "[datasheet]";
  parameter Real startTime(unit="s") = 0 "[configuration]";
  parameter Integer latencySamples(min=0) = 0 "[datasheet]";
  parameter Integer seed = 1 "[configuration]";
  parameter Integer streamId = 371 "[configuration]";
  parameter Real attitudeNoiseStd(unit="rad") = 0 "Roll and pitch [datasheet]";
  parameter Real attitudeDriftStd(unit="rad") = 0 "Roll and pitch [datasheet]";
  parameter Real headingNoiseStd(unit="rad") = 0 "[datasheet]";
  parameter Real headingDriftStd(unit="rad") = 0 "[datasheet]";
  parameter Real attitudeDriftTau(unit="s") = 60 "[assumed]";
  parameter Real gyroNoiseStd(unit="rad/s") = 0 "Per sample [datasheet]";
  parameter Real gyroBiasStd(unit="rad/s") = 0 "[datasheet]";
  parameter Real accelNoiseStd(unit="m/s2") = 0 "Per sample [datasheet]";
  parameter Real accelBiasStd(unit="m/s2") = 0 "[datasheet]";
  parameter Real biasTau(unit="s") = 300 "[assumed]";
  parameter Real positionNoiseStd(unit="m") = 0 "Horizontal [datasheet]";
  parameter Real verticalNoiseStd(unit="m") = 0 "[datasheet]";
  parameter Real positionDriftStd(unit="m") = 0 "Horizontal [datasheet]";
  parameter Real verticalDriftStd(unit="m") = 0 "[datasheet]";
  parameter Real positionDriftTau(unit="s") = 60 "[assumed]";
  parameter Real velocityNoiseStd(unit="m/s") = 0 "[datasheet]";
  output Real attitude[3](each unit="rad");
  output Real quaternion[4];
  output Real rate[3](each unit="rad/s");
  output Real accel[3](each unit="m/s2");
  output Real position[3](each unit="m");
  output Real velocity[3](each unit="m/s");
  output Boolean valid;
  output Real validity = if valid then 1 else 0 "1 if valid, else 0";
protected
  ExternalAhrs ideal(position_b=position_b,R_bs=R_bs,gravity=gravity);
  Sampler sampler(n=15,samplePeriod=samplePeriod,startTime=startTime,latencySamples=latencySamples,
    seed=seed,streamId=streamId,
    noiseStd=cat(1,{attitudeNoiseStd,attitudeNoiseStd,headingNoiseStd},fill(gyroNoiseStd,3),
      fill(accelNoiseStd,3),{positionNoiseStd,positionNoiseStd,verticalNoiseStd},fill(velocityNoiseStd,3)),
    driftStd=cat(1,{attitudeDriftStd,attitudeDriftStd,headingDriftStd},fill(gyroBiasStd,3),
      fill(accelBiasStd,3),{positionDriftStd,positionDriftStd,verticalDriftStd},zeros(3)),
    driftTau=cat(1,fill(attitudeDriftTau,3),fill(biasTau,6),fill(positionDriftTau,3),fill(300,3)));
equation
  ideal.truth = truth;
  sampler.u = cat(1, ideal.attitude, ideal.rate, ideal.accel, ideal.position, ideal.velocity);
  attitude = sampler.y[1:3];
  quaternion = eulerQuaternion(sampler.y[1:3]);
  rate = sampler.y[4:6];
  accel = sampler.y[7:9];
  position = sampler.y[10:12];
  velocity = sampler.y[13:15];
  valid = sampler.valid;
end SampledExternalAhrs;
