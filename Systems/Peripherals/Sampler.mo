within ModelicaCPSLibrary.Systems.Peripherals;
model Sampler "Sampled measurement chain per channel: scale error, bias, Gauss-Markov drift, white noise (fixed and proportional), quantization, clipping and delivery latency"
  parameter Integer n(min=1) = 1 "[configuration]";
  parameter Real samplePeriod(unit="s") = 0.02 "Acquisition period [datasheet]";
  parameter Real startTime(unit="s") = 0 "First acquisition, at or after simulation start [configuration]";
  parameter Integer latencySamples(min=0) = 0 "Delivery delay in sample periods [datasheet]";
  parameter Integer seed = 1 "[configuration]";
  parameter Integer streamId = 401 "[configuration]";
  parameter Real scale[n] = zeros(n) "Scale-factor error: (1 + scale) u [datasheet]";
  parameter Real bias[n] = zeros(n) "Fixed offset [datasheet]";
  parameter Real driftStd[n] = zeros(n) "Stationary standard deviation of the Gauss-Markov drift [datasheet]";
  parameter Real driftTau[n](each unit="s") = fill(300, n) "Drift correlation time [datasheet]";
  parameter Real noiseStd[n] = zeros(n) "White noise per sample [datasheet]";
  parameter Real noiseProportional[n] = zeros(n) "Added noise standard deviation per unit of |u| [datasheet]";
  parameter Real quantum[n] = zeros(n) "Resolution; 0 for none [datasheet]";
  parameter Real lower[n] = fill(-1e30, n) "Clipping, after quantization [datasheet]";
  parameter Real upper[n] = fill(1e30, n) "[datasheet]";
  input Real u[n] "Ideal measurand";
  input Boolean available = true "The ideal measurand is valid (else the sample is delivered invalid)";
  output Real y[n] "Delivered sample, held";
  output Boolean valid "The delivered sample is valid";
  output Integer sequence "Delivered samples";
  output Real sampleTime(unit="s") "Acquisition epoch of the delivered sample; -1 before the first";
protected
  discrete Integer rngState[Modelica.Math.Random.Generators.Xorshift128plus.nState](
    start=Modelica.Math.Random.Generators.Xorshift128plus.initialState(streamId,seed),each fixed=true);
  discrete Real normal[2*n](each start=0,each fixed=true);
  discrete Real drift[n](each start=0,each fixed=true);
  discrete Boolean initialized(start=false,fixed=true);
  Real acquired[n];
  Sensing.SampledDelivery delivery(n=n,samplePeriod=samplePeriod,startTime=startTime,
    latencySamples=latencySamples);
initial equation
  assert(min(driftTau)>0 and min(driftStd)>=0 and min(noiseStd)>=0 and min(noiseProportional)>=0
    and min(quantum)>=0,"Invalid sampler error parameters");
equation
  when sample(startTime,samplePeriod) then
    (normal,rngState)=Sensing.normalDraw(2*n,pre(rngState));
    for i in 1:n loop
      drift[i]=if not pre(initialized) then driftStd[i]*normal[i]
        else exp(-samplePeriod/driftTau[i])*pre(drift[i])
          +driftStd[i]*sqrt(1-exp(-2*samplePeriod/driftTau[i]))*normal[i];
    end for;
    initialized=true;
  end when;
  for i in 1:n loop
    acquired[i]=min(upper[i],max(lower[i],Sensing.roundTo((1+scale[i])*u[i]+bias[i]+drift[i]
      +(noiseStd[i]+noiseProportional[i]*abs(u[i]))*normal[n+i],quantum[i])));
  end for;
  delivery.u=acquired;
  delivery.acquisitionStatus=if available then 1 else 2;
  delivery.acquisitionValid=available;
  y=delivery.y;
  valid=delivery.valid;
  sequence=delivery.sequence;
  sampleTime=delivery.sampleTime;
end Sampler;
