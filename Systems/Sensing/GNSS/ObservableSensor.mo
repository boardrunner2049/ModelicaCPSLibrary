within fire_modelica_models.Systems.Sensing.GNSS;
model ObservableSensor "G2: satellite observables, visibility, clock and weighted PVT solution"
  extends PartialSampledSensor;
  parameter Integer nSat(min=1)=4;
  parameter Reference reference;
  parameter Real initialPosition[3](each unit="m")=zeros(3) "NED solver guess, independent of truth";
  parameter Real elevationMask(unit="rad")=0.0872664625997165 "Five degrees";
  parameter Integer seed=1;
  parameter Integer streamId=202;
  parameter Real rangeNoiseStd[nSat](each unit="m")=zeros(nSat);
  parameter Real rateNoiseStd[nSat](each unit="m/s")=zeros(nSat);
  parameter Real rangeSigma[nSat](each unit="m")=fill(3,nSat) "Receiver assumed standard deviations; strictly positive";
  parameter Real rateSigma[nSat](each unit="m/s")=fill(0.1,nSat);
  parameter Integer maxIterations=12;
  parameter Real convergenceTolerance(unit="m")=1e-4;
  parameter Real rankTolerance=1e-10;
  parameter Real carrierFrequency(unit="Hz")=1575.42e6;
  input Real satellitePosition_e[nSat,3](each unit="m") "ECEF satellite state at reception epoch";
  input Real satelliteVelocity_e[nSat,3](each unit="m/s");
  input Real satelliteClockBias[nSat](each unit="s")=zeros(nSat);
  input Real satelliteClockDrift[nSat]=zeros(nSat);
  input Boolean satelliteAvailable[nSat]=fill(true,nSat) "Health, data age and external obstruction mask";
  input Real receiverClockBias(unit="s")=0 "Clock truth, separate from packet latency";
  input Real receiverClockDrift=0;
  input Real ionosphereDelay[nSat](each unit="m")=zeros(nSat);
  input Real troposphereDelay[nSat](each unit="m")=zeros(nSat);
  input Real multipath[nSat](each unit="m")=zeros(nSat);
  input Real rangeError[nSat](each unit="m")=zeros(nSat) "Additional code-observable disturbance";
  input Real rateError[nSat](each unit="m/s")=zeros(nSat) "Range-rate disturbance, including time-varying delay derivatives";
  input Real rangeCorrection[nSat](each unit="m")=zeros(nSat) "Receiver modeled path delay, not automatic truth";
  input Real rateCorrection[nSat](each unit="m/s")=zeros(nSat);
  output Real pseudorange[nSat](each unit="m",each start=0,each fixed=true) "Acquisition diagnostics, not delayed";
  output Real rangeRate[nSat](each unit="m/s",each start=0,each fixed=true);
  output Real doppler[nSat](each unit="Hz") "Negative range rate / carrier wavelength";
  output Boolean visible[nSat](each start=false,each fixed=true);
  output Real elevation[nSat](each unit="rad");
  output Integer satellitesUsed "Count at delivered epoch, even for failed geometry";
  output Real clockBias(unit="s") "Delivered estimate";
  output Real clockDrift;
  output Real pdop "Delivered; -1 for invalid solution (zero before first packet)";
protected
  constant Real c=299792458;
  parameter Real origin[3](each fixed=false);
  parameter Real R_ew[3,3](each fixed=false);
  Real receiver_e[3];
  Real receiverVelocity_e[3];
  Real geometricRange[nSat];
  Real gradient[nSat,3];
  Real satelliteRate[nSat];
  Real los[nSat,3];
  Real up_e[3];
  discrete Real normal[2*nSat](each start=0,each fixed=true);
  discrete Integer rngState[Modelica.Math.Random.Generators.Xorshift128plus.nState](
    start=Modelica.Math.Random.Generators.Xorshift128plus.initialState(streamId,seed),each fixed=true);
  discrete Real solution[6](each start=0,each fixed=true);
  discrete Real solutionCovariance[6,6](each start=0,each fixed=true);
  discrete Real solutionClock(start=0,fixed=true);
  discrete Real solutionDrift(start=0,fixed=true);
  discrete Real solutionDop(start=-1,fixed=true);
  discrete Boolean solutionValid(start=false,fixed=true);
  SampledDelivery qualityDelivery(n=4,samplePeriod=samplePeriod,startTime=startTime,latencySamples=latencySamples);
initial equation
  (origin,R_ew)=referenceFrame(reference);
  assert(nSat>=1 and min(rangeSigma)>0 and min(rateSigma)>0,"Invalid GNSS weights");
  assert(min(rangeNoiseStd)>=0 and min(rateNoiseStd)>=0,"Invalid GNSS noise");
  assert(maxIterations>=1 and convergenceTolerance>0 and rankTolerance>0 and rankTolerance<1,"Invalid navigation solver settings");
  assert(carrierFrequency>0 and abs(elevationMask)<=Modelica.Constants.pi/2,"Invalid GNSS mask or carrier frequency");
  assert(max(abs(positionBias))+max(abs(velocityBias))<=0,"G2 errors belong in observables, not a second PVT error layer");
equation
  receiver_e=origin+R_ew*position_w;
  receiverVelocity_e=R_ew*velocity_w;
  // Ellipsoid normal at the current antenna; local NED basis remains fixed at reference.
  up_e={receiver_e[1],receiver_e[2],receiver_e[3]/(1-6.6943799901413165e-3)} /
    max(1,sqrt(receiver_e[1]^2+receiver_e[2]^2+(receiver_e[3]/(1-6.6943799901413165e-3))^2));
  (geometricRange,gradient,satelliteRate,los)=constellationGeometry(receiver_e,satellitePosition_e,satelliteVelocity_e);
  for i in 1:nSat loop
    elevation[i]=asin(min(1,max(-1,los[i,:]*up_e)));
    doppler[i]=-rangeRate[i]*carrierFrequency/c;
  end for;
  acquiredPvt=solution;
  acquiredCovariance=solutionCovariance;
  acquiredValid=solutionValid;
  qualityDelivery.u={solutionClock,solutionDrift,solutionDop,sum({if visible[i] then 1 else 0 for i in 1:nSat})};
  qualityDelivery.acquisitionValid=solutionValid;
  qualityDelivery.acquisitionStatus=if solutionValid then 3 else 0;
  clockBias=qualityDelivery.y[1];
  clockDrift=qualityDelivery.y[2];
  pdop=qualityDelivery.y[3];
  satellitesUsed=integer(qualityDelivery.y[4]);
algorithm
  when sample(startTime,samplePeriod) then
    (normal,rngState):=normalDraw(2*nSat,pre(rngState));
    for i in 1:nSat loop
      visible[i]:=satelliteAvailable[i] and geometricRange[i]>1 and elevation[i]>=elevationMask;
      pseudorange[i]:=(1+satelliteClockDrift[i])*geometricRange[i]+c*(receiverClockBias-satelliteClockBias[i])
        +ionosphereDelay[i]+troposphereDelay[i]+multipath[i]+rangeError[i]+rangeNoiseStd[i]*normal[i];
      rangeRate[i]:=(1+satelliteClockDrift[i])*(satelliteRate[i]+gradient[i,:]*receiverVelocity_e)
        +c*(receiverClockDrift-satelliteClockDrift[i])+rateError[i]+rateNoiseStd[i]*normal[nSat+i];
    end for;
    (solution,solutionCovariance,solutionClock,solutionDrift,solutionDop,solutionValid):=
      solvePvt(satellitePosition_e,satelliteVelocity_e,satelliteClockBias,satelliteClockDrift,
        pseudorange-rangeCorrection,rangeRate-rateCorrection,visible,rangeSigma,rateSigma,
        origin+R_ew*initialPosition,origin,R_ew,maxIterations,convergenceTolerance,rankTolerance);
  end when;
end ObservableSensor;
