within ModelicaCPSLibrary.Systems.Sensing.Range;
model SampledSensor "L1: sampled biased/noisy/quantized slant range and atomic status delivery"
  extends PartialSensor;
  parameter Real samplePeriod(unit="s")=0.02;
  parameter Real startTime(unit="s")=0;
  parameter Integer latencySamples(min=0)=0;
  parameter Integer seed=1;
  parameter Integer streamId=301;
  parameter Real bias(unit="m")=0;
  parameter Real noiseStd(unit="m")=0 "Per-sample standard deviation at zero range";
  parameter Real noiseSlope=0 "Additional standard deviation per metre";
  parameter Real quantum(unit="m")=0 "Zero disables rounding; no saturation";
  parameter Real referenceTemperature(unit="K")=293.15;
  parameter Real temperatureCoefficient(unit="m/K")=0;
  input Real sensorTemperature(unit="K")=referenceTemperature;
  input Boolean returnAvailable=true;
  input Boolean deviceError=false;
  input Real signalQuality=1 "Scenario quality [0,1], independent of injected bias";
  output Real sampleTime(unit="s");
  output Real deliveryTime(unit="s");
  output Integer sequence;
  output Real quality "Zero for invalid packets";
protected
  Sensor ideal(minRange=minRange,maxRange=maxRange,parallelTolerance=parallelTolerance,
    direction_w=direction_w,planeNormal_w=planeNormal_w,planeOffset=planeOffset);
  SampledDelivery delivery(n=2,samplePeriod=samplePeriod,startTime=startTime,latencySamples=latencySamples);
  discrete Integer rngState[Modelica.Math.Random.Generators.Xorshift128plus.nState](
    start=Modelica.Math.Random.Generators.Xorshift128plus.initialState(streamId,seed),each fixed=true);
  discrete Real normal[1](each start=0,each fixed=true);
  Real raw;
  Real rounded;
  Integer acquisitionStatus;
initial equation
  assert(noiseStd>=0 and noiseSlope>=0 and quantum>=0,"Invalid range error parameters");
equation
  ideal.position_w=position_w;
  assert(signalQuality>=0 and signalQuality<=1,"Range signal quality must be in [0,1]");
  when sample(startTime,samplePeriod) then
    (normal,rngState)=normalDraw(1,pre(rngState));
  end when;
  raw=ideal.distance+bias+temperatureCoefficient*(sensorTemperature-referenceTemperature)
    +(noiseStd+noiseSlope*ideal.distance)*normal[1];
  rounded=noEvent(if quantum>0 then quantum*floor(raw/max(quantum,Modelica.Constants.small)+0.5) else raw);
  acquisitionStatus=if deviceError then DeviceError else if not returnAvailable then NoReturn
    else if not ideal.valid then ideal.status else if rounded<minRange then TooClose
    else if rounded>maxRange then TooFar else Valid;
  delivery.u={if acquisitionStatus==Valid then rounded else 0,if acquisitionStatus==Valid then signalQuality else 0};
  delivery.acquisitionStatus=acquisitionStatus;
  delivery.acquisitionValid=acquisitionStatus==Valid;
  distance=delivery.y[1];
  quality=delivery.y[2];
  status=delivery.status;
  valid=delivery.valid;
  sampleTime=delivery.sampleTime;
  deliveryTime=delivery.deliveryTime;
  sequence=delivery.sequence;
end SampledSensor;
