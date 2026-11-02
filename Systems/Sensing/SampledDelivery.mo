within ModelicaCPSLibrary.Systems.Sensing;
model SampledDelivery "Atomic value/status acquisition and integer-period delivery queue"
  parameter Integer n(min=1)=1;
  parameter Real samplePeriod(unit="s")=0.1;
  parameter Real startTime(unit="s")=0 "First acquisition, at or after simulation start";
  parameter Integer latencySamples(min=0)=0;
  input Real u[n];
  input Integer acquisitionStatus "Family-specific status, including invalid samples";
  input Boolean acquisitionValid;
  output Real y[n](each start=0,each fixed=true);
  output Integer status(start=0,fixed=true) "Zero before first delivery";
  output Boolean valid(start=false,fixed=true);
  output Real sampleTime(unit="s",start=-1,fixed=true);
  output Real deliveryTime(unit="s",start=-1,fixed=true);
  output Integer sequence(start=0,fixed=true) "Counts delivered packets, including invalid ones";
protected
  discrete Integer acquired(start=0,fixed=true);
  discrete Real history[latencySamples+1,n](each start=0,each fixed=true);
  discrete Integer statusHistory[latencySamples+1](each start=0,each fixed=true);
  discrete Boolean validHistory[latencySamples+1](each start=false,each fixed=true);
initial equation
  assert(samplePeriod>0 and latencySamples>=0 and startTime>=time,"Invalid acquisition/delivery clock");
equation
  when sample(startTime,samplePeriod) then
    acquired=pre(acquired)+1;
    history[1,:]=u;
    statusHistory[1]=acquisitionStatus;
    validHistory[1]=acquisitionValid;
    for j in 2:latencySamples+1 loop
      history[j,:]=pre(history[j-1,:]);
      statusHistory[j]=pre(statusHistory[j-1]);
      validHistory[j]=pre(validHistory[j-1]);
    end for;
    y=if acquired>latencySamples then history[latencySamples+1,:] else pre(y);
    status=if acquired>latencySamples then statusHistory[latencySamples+1] else pre(status);
    valid=if acquired>latencySamples then validHistory[latencySamples+1] else false;
    sequence=if acquired>latencySamples then acquired-latencySamples else 0;
    sampleTime=if acquired>latencySamples then startTime+(sequence-1)*samplePeriod else -1;
    deliveryTime=if acquired>latencySamples then time else -1;
  end when;
end SampledDelivery;
