within fire_modelica_models.Tests;
model ImuStatistical "Sample epochs, queue validity, calibrated axes and deterministic independent streams"
  Systems.Sensing.IMU.StatisticalSensor delayed(p(samplePeriod=0.01,latencySamples=2),accelBias={0.1,0,0});
  Systems.Sensing.IMU.StatisticalSensor repeatA(p(samplePeriod=0.001,seed=2718,streamId=1,noiseStd=fill(0.2,6),biasStd=fill(0.1,6),biasTau=fill(0.02,6)));
  Systems.Sensing.IMU.StatisticalSensor repeatB(p(samplePeriod=0.001,seed=2718,streamId=1,noiseStd=fill(0.2,6),biasStd=fill(0.1,6),biasTau=fill(0.02,6)));
  Systems.Sensing.IMU.StatisticalSensor independent(p(samplePeriod=0.001,seed=2718,streamId=2,noiseStd=fill(0.2,6),biasStd=fill(0.1,6),biasTau=fill(0.02,6)));
  Systems.Sensing.IMU.StatisticalSensor calibrated(p(samplePeriod=0.01,scale={0.1,0,0,0,0,0},
    accelMisalignment=[0,0.2,0;0,0,0;0,0,0],gyroGSensitivity=identity(3)),accelBias={0.3,0,0});
equation
  delayed.R_wb=identity(3);
  delayed.a_w={time,0,0};
  delayed.gravity_w=zeros(3);
  delayed.omega_b=zeros(3);
  delayed.alpha_b=zeros(3);
  repeatA.R_wb=identity(3);
  repeatA.a_w=zeros(3);
  repeatA.gravity_w=zeros(3);
  repeatA.omega_b=zeros(3);
  repeatA.alpha_b=zeros(3);
  repeatB.R_wb=identity(3);
  repeatB.a_w=zeros(3);
  repeatB.gravity_w=zeros(3);
  repeatB.omega_b=zeros(3);
  repeatB.alpha_b=zeros(3);
  independent.R_wb=identity(3);
  independent.a_w=zeros(3);
  independent.gravity_w=zeros(3);
  independent.omega_b=zeros(3);
  independent.alpha_b=zeros(3);
  calibrated.R_wb=identity(3);
  calibrated.a_w={1,2,3};
  calibrated.gravity_w=zeros(3);
  calibrated.omega_b={4,5,6};
  calibrated.alpha_b=zeros(3);
  when time>=0.015 then
    assert(not delayed.valid and delayed.sequence==0 and abs(delayed.sampleTime+1)<1e-12,"Delay must not report unacquired samples");
  end when;
  when time>=0.025 then
    assert(delayed.valid and delayed.sequence==1 and abs(delayed.sampleTime)<1e-12,"First delayed epoch is incorrect");
    assert(abs(delayed.acceleration[1]-0.1)<1e-10,"Delivery queue must retain the acquisition value");
  end when;
  when time>=0.036 then
    assert(abs(delayed.sampleTime-0.01)<1e-12 and abs(delayed.deliveryTime-0.03)<1e-12,"Acquisition and delivery epochs were conflated");
    assert(abs(delayed.acceleration[1]-0.11)<1e-10,"Output must hold between deliveries");
    assert(abs(calibrated.acceleration[1]-1.8)<1e-12 and max(abs(calibrated.gyro-{5,7,9}))<1e-12,"Scale/cross-axis/g sensitivity mismatch");
  end when;
  when sample(0.0005,0.001) then
    assert(max(abs(repeatA.acceleration-repeatB.acceleration))<1e-14,"Identical seeds must replay exactly");
    assert(max(abs(repeatA.gyro-independent.gyro))>1e-10,"Independent stream IDs must not share a sequence");
  end when;
  annotation(experiment(StopTime=1,Interval=0.001,Tolerance=1e-9));
end ImuStatistical;
