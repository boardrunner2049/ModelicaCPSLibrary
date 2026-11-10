within ModelicaCPSLibrary.Systems.Sensing.Barometer;
model SampledSensor "B1: sampled pressure and temperature with white noise, Gauss-Markov pressure drift, quantization and delivery latency"
  extends PartialSensor;
  parameter Real samplePeriod(unit="s") = 0.02 "Acquisition period (50 Hz); the part's measurement times bound it [configuration]";
  parameter Real startTime(unit="s") = 0 "First acquisition, at or after simulation start [configuration]";
  parameter Integer latencySamples(min=0) = 0 "Delivery delay in sample periods; structural (sizes the queue), fixed when the model is compiled [assumed]";
  parameter Integer seed = 1 "Noise stream seed [configuration]";
  parameter Integer streamId = 301 "Noise stream id [configuration]";
  parameter Real pressureNoiseStd(unit="Pa") = 0 "White noise per sample [datasheet]";
  parameter Real pressureDriftStd(unit="Pa") = 0 "Stationary standard deviation of the drift; no datasheet states a drift process [assumed]";
  parameter Real pressureDriftTau(unit="s") = 300 "Drift correlation time [assumed]";
  parameter Real pressureQuantum(unit="Pa") = 0 "Pressure resolution; 0 for none [datasheet]";
  parameter Real temperatureNoiseStd(unit="K") = 0 "White temperature noise per sample [assumed]";
  parameter Real temperatureQuantum(unit="K") = 0 "Temperature resolution; 0 for none [datasheet]";
  output Real pressureDrift(unit="Pa") "Diagnostic truth of the drift";
protected
  // Seeded at the first acquisition rather than by a start value, so that seed
  // and streamId stay settable in an exported FMU: Rumoca evaluates a start
  // value at compile time. The draws are the same as a start-value seeding.
  discrete Integer rngState[Modelica.Math.Random.Generators.Xorshift128plus.nState](each start=0,each fixed=true);
  discrete Real normal[3](each start=0,each fixed=true);
  discrete Real drift(start=0,fixed=true);
  discrete Boolean initialized(start=false,fixed=true);
  final parameter Real phi = exp(-samplePeriod/pressureDriftTau);
  discrete Real acquired[4](each start=0,each fixed=true);
  SampledDelivery delivery(n=4,samplePeriod=samplePeriod,startTime=startTime,latencySamples=latencySamples);
initial equation
  assert(pressureDriftTau>0 and pressureNoiseStd>=0 and pressureDriftStd>=0 and temperatureNoiseStd>=0
    and pressureQuantum>=0 and temperatureQuantum>=0,"Invalid barometer error parameters");
equation
  when sample(startTime,samplePeriod) then
    (normal,rngState)=seededNormalDraw(3,pre(rngState),pre(initialized),streamId,seed);
    drift=if not pre(initialized) then pressureDriftStd*normal[1]
      else phi*pre(drift)+pressureDriftStd*sqrt(1-phi^2)*normal[1];
    initialized=true;
    acquired[1]=roundTo(ambientPressure+pressureBias+drift+pressureNoiseStd*normal[2],pressureQuantum);
    acquired[2]=roundTo(ambientTemperature+temperatureBias+temperatureNoiseStd*normal[3],temperatureQuantum);
    // The pressure error as a height error, through the local density p/(R T).
    acquired[3]=altitude_w+altitudeBias-(acquired[1]-ambientPressure)
      *287.05287*ambientTemperature/(ambientPressure*Modelica.Constants.g_n);
    acquired[4]=climbRate_w;
  end when;
  pressureDrift=drift;
  delivery.u=acquired;
  delivery.acquisitionStatus=1;
  delivery.acquisitionValid=true;
  pressure=delivery.y[1];
  temperature=delivery.y[2];
  altitude=delivery.y[3];
  climbRate=delivery.y[4];
  sampleTime=delivery.sampleTime;
  deliveryTime=delivery.deliveryTime;
  sequence=delivery.sequence;
  valid=delivery.valid;
end SampledSensor;
