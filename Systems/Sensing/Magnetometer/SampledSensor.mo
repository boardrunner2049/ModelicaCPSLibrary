within ModelicaCPSLibrary.Systems.Sensing.Magnetometer;
model SampledSensor "M1: sampled field with soft-iron distortion, hard-iron offset, white noise, Gauss-Markov bias drift, quantization, clipping and delivery latency"
  extends PartialSensor;
  parameter Real samplePeriod(unit="s") = 0.01 "Acquisition period (100 Hz)";
  parameter Real startTime(unit="s") = 0 "First acquisition, at or after simulation start";
  parameter Integer latencySamples(min=0) = 0 "Delivery delay in sample periods";
  parameter Integer seed = 1;
  parameter Integer streamId = 401;
  parameter Real softIron[3,3] = identity(3) "Measured = softIron*field + bias (sensor frame)";
  parameter Real noiseStd[3](each unit="T") = zeros(3) "White noise per sample";
  parameter Real biasStd[3](each unit="T") = zeros(3) "Stationary standard deviation of the bias drift";
  parameter Real biasTau[3](each unit="s") = fill(600,3) "Bias drift correlation times";
  parameter Real quantum[3](each unit="T") = zeros(3) "Resolution; 0 for none";
  parameter Real fullScale[3](each unit="T") = fill(1e-3,3) "Clipping range (+-)";
  output Real sampleTime(unit="s") "Acquisition epoch of the delivered sample; -1 before the first";
  output Integer sequence "Delivered samples";
  output Boolean valid "A sample has been delivered";
  output Real biasState[3](each unit="T") "Diagnostic truth of the bias drift";
protected
  discrete Integer rngState[Modelica.Math.Random.Generators.Xorshift128plus.nState](
    start=Modelica.Math.Random.Generators.Xorshift128plus.initialState(streamId,seed),each fixed=true);
  discrete Real normal[6](each start=0,each fixed=true);
  discrete Real drift[3](each start=0,each fixed=true);
  discrete Boolean initialized(start=false,fixed=true);
  discrete Real acquired[3](each start=0,each fixed=true);
  final parameter Real phi[3] = exp(-samplePeriod ./ biasTau);
  SampledDelivery delivery(n=3,samplePeriod=samplePeriod,startTime=startTime,latencySamples=latencySamples);
initial equation
  assert(min(biasTau)>0 and min(noiseStd)>=0 and min(biasStd)>=0 and min(quantum)>=0 and min(fullScale)>0,
    "Invalid magnetometer error parameters");
equation
  when sample(startTime,samplePeriod) then
    (normal,rngState)=normalDraw(6,pre(rngState));
    for j in 1:3 loop
      drift[j]=if not pre(initialized) then biasStd[j]*normal[j]
        else phi[j]*pre(drift[j])+biasStd[j]*sqrt(1-phi[j]^2)*normal[j];
    end for;
    initialized=true;
    acquired={IMU.quantize((softIron*field_s)[j]+bias[j]+drift[j]+noiseStd[j]*normal[3+j],
      fullScale[j],quantum[j]) for j in 1:3};
  end when;
  biasState=drift;
  delivery.u=acquired;
  delivery.acquisitionStatus=1;
  delivery.acquisitionValid=true;
  magneticField=delivery.y;
  sampleTime=delivery.sampleTime;
  sequence=delivery.sequence;
  valid=delivery.valid;
end SampledSensor;
