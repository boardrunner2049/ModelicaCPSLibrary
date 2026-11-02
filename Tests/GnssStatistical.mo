within fire_modelica_models.Tests;
model GnssStatistical "G0/G1 antenna kinematics, epochs, delayed fix loss and reproducible correlated errors"
  Systems.Sensing.GNSS.Sensor ideal(positionBias={1,2,3},velocityBias={0.1,0.2,0.3});
  Systems.Sensing.GNSS.StatisticalSensor delayed(samplePeriod=0.1,latencySamples=2,
    positionBias={1,2,3},velocityBias={0.1,0.2,0.3},fixAvailable=not (time>=0.15 and time<0.35));
  Systems.Sensing.GNSS.StatisticalSensor repeatA(samplePeriod=0.002,seed=2718,streamId=10,
    noiseFactor=[1,0,0,0,0,0;0.5,1,0,0,0,0;0,0,1,0,0,0;0.2,0,0,0.1,0,0;0,0,0,0,0.1,0;0,0,0,0,0,0.1],
    biasFactor=0.1*identity(6),biasTau=fill(0.02,6));
  Systems.Sensing.GNSS.StatisticalSensor repeatB(samplePeriod=0.002,seed=2718,streamId=10,
    noiseFactor=repeatA.noiseFactor,biasFactor=repeatA.biasFactor,biasTau=repeatA.biasTau);
  Systems.Sensing.GNSS.StatisticalSensor independent(samplePeriod=0.002,seed=2718,streamId=11,
    noiseFactor=repeatA.noiseFactor);
  Systems.Sensing.SensorSuite suite(redeclare model GnssModel=Systems.Sensing.GNSS.StatisticalSensor,
    rGnss_b={1,0,0},gnss(samplePeriod=0.1));
equation
  ideal.position_w={time,2*time,3*time};
  ideal.velocity_w={1,2,3};
  delayed.position_w=ideal.position_w;
  delayed.velocity_w=ideal.velocity_w;
  repeatA.position_w=zeros(3);
  repeatA.velocity_w=zeros(3);
  repeatB.position_w=zeros(3);
  repeatB.velocity_w=zeros(3);
  independent.position_w=zeros(3);
  independent.velocity_w=zeros(3);
  suite.p_w={0,0,-10};
  suite.v_w={1,2,3};
  suite.R_wb=[0,-1,0;1,0,0;0,0,1];
  suite.omega_b={0,0,2};
  suite.a_w=zeros(3);
  suite.alpha_b=zeros(3);
  suite.gravity_w={0,0,9.81};
  suite.magneticField_w=zeros(3);
  suite.pressure=101325;
  suite.temperature=293.15;
  when time>=0.15 then
    assert(not delayed.valid and delayed.sequence==0,"G1 queue must start empty");
    assert(max(abs(ideal.position-{1.15,2.3,3.45}))<1e-8,"G0 must remain continuous");
  end when;
  when time>=0.25 then
    assert(delayed.valid and delayed.sequence==1 and abs(delayed.sampleTime)<1e-12,"First G1 epoch incorrect");
    assert(max(abs(delayed.position-{1,2,3}))<1e-12 and max(abs(delayed.velocity-{1.1,2.2,3.3}))<1e-12,"PVT must share acquisition time");
    assert(abs(delayed.deliveryTime-0.2)<1e-12,"Latency must not change acquisition time");
    assert(max(abs(suite.gnss.position-{0,1,-10}))<1e-12 and max(abs(suite.gnss.velocity-{-1,2,3}))<1e-12,"Antenna lever arm applied incorrectly");
  end when;
  when time>=0.45 then
    assert(not delayed.valid and delayed.fixType==0 and delayed.sequence==3,"Fix loss must be delayed with PVT");
    assert(max(abs(delayed.position))<1e-12,"Invalid PVT payload is zero, not a valid stale fix");
  end when;
  when time>=0.65 then
    assert(delayed.valid and delayed.sequence==5 and abs(delayed.sampleTime-0.4)<1e-12,"Fix reacquisition incorrect");
  end when;
  when sample(0.001,0.002) then
    assert(max(abs(repeatA.position-repeatB.position))<1e-14 and max(abs(repeatA.velocity-repeatB.velocity))<1e-14,"Seed replay must be exact");
    assert(max(abs(repeatA.position-independent.position))>1e-8,"Independent RNG streams collapsed");
    assert(abs(repeatA.covariance[1,2]-0.5)<1e-12 and abs(repeatA.covariance[1,4]-0.2)<1e-12,"Full PVT covariance lost correlation");
  end when;
  annotation(experiment(StopTime=4,Interval=0.002,Tolerance=1e-9));
end GnssStatistical;
