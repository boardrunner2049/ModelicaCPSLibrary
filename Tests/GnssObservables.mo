within ModelicaCPSLibrary.Tests;
model GnssObservables "G2 exact PVT, clock errors, corrections, delayed visibility loss and rank rejection"
  parameter Real sat0[6,3]=[24000000,0,0;18000000,16000000,0;18000000,-16000000,0;
    18000000,0,16000000;18000000,0,-16000000;21000000,12000000,12000000];
  parameter Real satVelocity[6,3]=[0,2100,1300;-1200,1300,2400;1100,1500,-1300;
    1500,-2400,1200;-1800,2000,1400;1000,-1700,1900];
  Real sat[6,3];
  Real p[3];
  Real v[3];
  Systems.Sensing.GNSS.ObservableSensor nominal(nSat=6,samplePeriod=0.1,latencySamples=1,
    receiverClockBias=1e-4+2e-9*time,receiverClockDrift=2e-9,
    satelliteClockBias={1e-5+i*1e-6+i*1e-10*time for i in 1:6},
    satelliteClockDrift={i*1e-10 for i in 1:6},
    satelliteAvailable={not(time>=0.25 and time<0.45) or i<=3 for i in 1:6});
  Systems.Sensing.GNSS.ObservableSensor common(nSat=6,samplePeriod=0.1,rangeError=fill(300,6),rateError=fill(5,6));
  Systems.Sensing.GNSS.ObservableSensor corrected(nSat=6,samplePeriod=0.1,
    ionosphereDelay=fill(2,6),troposphereDelay=fill(3,6),multipath=fill(1,6),rangeCorrection=fill(6,6));
  Systems.Sensing.GNSS.ObservableSensor rankDeficient(nSat=4,samplePeriod=0.1);
  Systems.Sensing.GNSS.ObservableSensor oneError(nSat=6,samplePeriod=0.1,rangeError={30,0,0,0,0,0});
equation
  p={120,-75,-50}+{3,-2,0.5}*time;
  v={3,-2,0.5};
  for i in 1:6 loop
    sat[i,:]=sat0[i,:]+satVelocity[i,:]*time;
  end for;
  nominal.position_w=p;
  nominal.velocity_w=v;
  nominal.satellitePosition_e=sat;
  nominal.satelliteVelocity_e=satVelocity;
  common.position_w=p;
  common.velocity_w=v;
  common.satellitePosition_e=sat;
  common.satelliteVelocity_e=satVelocity;
  corrected.position_w=p;
  corrected.velocity_w=v;
  corrected.satellitePosition_e=sat;
  corrected.satelliteVelocity_e=satVelocity;
  oneError.position_w=p;
  oneError.velocity_w=v;
  oneError.satellitePosition_e=sat;
  oneError.satelliteVelocity_e=satVelocity;
  rankDeficient.position_w=p;
  rankDeficient.velocity_w=v;
  for i in 1:4 loop
    rankDeficient.satellitePosition_e[i,:]=sat[1,:];
    rankDeficient.satelliteVelocity_e[i,:]=satVelocity[1,:];
  end for;
  when sample(0.05,0.1) then
    assert(common.valid and corrected.valid,"Full-rank geometry must yield fix");
    assert(max(abs(common.position-({120,-75,-50}+v*common.sampleTime)))<1e-4,"Common range error should be absorbed by clock");
    assert(max(abs(common.velocity-v))<1e-6 and abs(common.clockBias-300/299792458)<1e-12 and abs(common.clockDrift-5/299792458)<1e-14,"Range/rate clock estimation incorrect");
    assert(max(abs(corrected.position-({120,-75,-50}+v*corrected.sampleTime)))<1e-4,"Path correction sign incorrect");
    assert(not rankDeficient.valid and rankDeficient.fixType==0 and rankDeficient.satellitesUsed==4,"Satellite count alone must not establish a fix");
    assert(oneError.valid and max(abs(oneError.position-common.position))>1,"Single-channel range errors must affect PVT");
    assert(max(abs(oneError.covariance-common.covariance))<1e-4*max(abs(common.covariance)),"Injected error must not directly expose truth through covariance");
    assert(common.pdop>0 and min({common.covariance[i,i] for i in 1:6})>0,"Reported uncertainty missing");
  end when;
  when time>=0.15 then
    assert(nominal.valid and nominal.sequence==1 and abs(nominal.sampleTime)<1e-12,"G2 first delivery wrong");
    assert(max(abs(nominal.position-{120,-75,-50}))<1e-4 and max(abs(nominal.velocity-v))<1e-6,"G2 no-error solution wrong");
    assert(abs(nominal.clockBias-1e-4)<1e-12 and abs(nominal.clockDrift-2e-9)<1e-14,"G2 satellite/receiver clock corrections wrong");
  end when;
  when time>=0.45 then
    assert(not nominal.valid and nominal.satellitesUsed==3 and nominal.sequence==4,"Visibility/fix loss must be delivered together");
  end when;
  when time>=0.65 then
    assert(nominal.valid and nominal.satellitesUsed==6 and nominal.sequence==6,"G2 reacquisition failed");
  end when;
  annotation(experiment(StopTime=0.7,Interval=0.002,Tolerance=1e-9));
end GnssObservables;
