within fire_modelica_models.Examples;
model GnssRange "G0/G1/G2 and L0/L1 with an illustrative broadcast constellation and moving antenna"
  parameter Integer nSat=24;
  Systems.Sensing.GNSS.BroadcastConstellation constellation(nSat=nSat,
    ephemeris(each eccentricity=0.01,
      omega0={2*Modelica.Constants.pi*floor((i-1)/4)/6 for i in 1:nSat},
      meanAnomaly={2*Modelica.Constants.pi*mod(i-1,4)/4+0.2*floor((i-1)/4) for i in 1:nSat}));
  Systems.Sensing.GNSS.Sensor g0;
  Systems.Sensing.GNSS.StatisticalSensor g1(samplePeriod=0.1,latencySamples=1,seed=2718,
    noiseFactor=diagonal({0.5,0.5,1,0.05,0.05,0.1}));
  Systems.Sensing.GNSS.ObservableSensor g2(nSat=nSat,samplePeriod=0.1,latencySamples=1,
    satelliteClockBias=constellation.clockBias,satelliteClockDrift=constellation.clockDrift,
    satelliteAvailable=constellation.available,receiverClockBias=1e-4);
  Systems.Sensing.Range.Sensor l0(direction_w={sin(0.2),0,cos(0.2)});
  Systems.Sensing.Range.SampledSensor l1(samplePeriod=0.02,latencySamples=1,
    direction_w={sin(0.2),0,cos(0.2)},noiseStd=0.02,quantum=0.01,seed=2718);
  output Real idealPosition[3];
  output Real statisticalPosition[3];
  output Real position[3];
  output Real velocity[3];
  output Real sampleTime;
  output Real deliveryTime;
  output Integer sequence;
  output Boolean valid;
  output Real distance;
  output Real rangeSampleTime;
  output Integer rangeSequence;
  output Boolean rangeValid;
protected
  Real antenna[3];
equation
  antenna={2*time,time,-20};
  g0.position_w=antenna;
  g0.velocity_w={2,1,0};
  g1.position_w=antenna;
  g1.velocity_w={2,1,0};
  g2.position_w=antenna;
  g2.velocity_w={2,1,0};
  g2.satellitePosition_e=constellation.position_e;
  g2.satelliteVelocity_e=constellation.velocity_e;
  l0.position_w=antenna;
  l1.position_w=antenna;
  idealPosition=g0.position;
  statisticalPosition=g1.position;
  position=g2.position;
  velocity=g2.velocity;
  sampleTime=g2.sampleTime;
  deliveryTime=g2.deliveryTime;
  sequence=g2.sequence;
  valid=g2.valid;
  distance=l1.distance;
  rangeSampleTime=l1.sampleTime;
  rangeSequence=l1.sequence;
  rangeValid=l1.valid;
  annotation(experiment(StopTime=1,Interval=0.002,Tolerance=1e-9));
end GnssRange;
