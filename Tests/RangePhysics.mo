within ModelicaCPSLibrary.Tests;
model RangePhysics "Ray/plane slant distance, arbitrary normal scale and distinct invalid statuses"
  Systems.Sensing.Range.Sensor tilted(direction_w={0.6,0,0.8},planeNormal_w={-0.2,0,1});
  Systems.Sensing.Range.Sensor scaled(direction_w={0.6,0,0.8},planeNormal_w={-0.4,0,2});
  Systems.Sensing.Range.Sensor parallelRay(direction_w={1,0,0});
  Systems.Sensing.Range.Sensor behind(direction_w={0,0,-1});
  Systems.Sensing.Range.Sensor close(minRange=0.2);
  Systems.Sensing.Range.Sensor far(maxRange=5);
  Systems.Sensing.Range.SampledSensor delayed(samplePeriod=0.1,latencySamples=2,bias=0.12,quantum=0.1,
    temperatureCoefficient=0.01,sensorTemperature=303.15,signalQuality=0.7,
    returnAvailable=not(time>=0.15 and time<0.35),deviceError=time>=0.45 and time<0.55);
  Systems.Sensing.Range.SampledSensor biasedTooClose(minRange=0.2,bias=-2);
  Systems.Sensing.Range.SampledSensor biasedTooFar(maxRange=2,bias=2);
  Systems.Sensing.Range.SampledSensor repeatA(noiseStd=0.2,seed=42);
  Systems.Sensing.Range.SampledSensor repeatB(noiseStd=0.2,seed=42);
equation
  tilted.position_w={0,0,-10};
  scaled.position_w=tilted.position_w;
  parallelRay.position_w={0,0,-10};
  behind.position_w={0,0,-10};
  close.position_w={0,0,-0.1};
  far.position_w={0,0,-10};
  delayed.position_w={0,0,-10-time};
  biasedTooClose.position_w={0,0,-1};
  biasedTooFar.position_w={0,0,-1};
  repeatA.position_w={0,0,-10};
  repeatB.position_w={0,0,-10};
  when time>=0.15 then
    assert(abs(tilted.distance-10/0.68)<1e-10 and abs(scaled.distance-tilted.distance)<1e-10,"Tilted slant range or normal scaling incorrect");
    assert(not parallelRay.valid and parallelRay.status==2 and behind.status==2,"Parallel/behind rays must be no-return");
    assert(close.status==3 and far.status==4,"Too-close and too-far must remain distinct");
    assert(not delayed.valid and delayed.status==0,"No range packet before delivery");
    assert(biasedTooClose.status==3 and biasedTooFar.status==4,"Invalid corrupted ranges must not be clamped valid");
  end when;
  when time>=0.25 then
    assert(delayed.valid and delayed.sequence==1 and abs(delayed.distance-10.2)<1e-10,"Range calibration/rounding/acquisition incorrect");
    assert(abs(delayed.sampleTime)<1e-12 and abs(delayed.deliveryTime-0.2)<1e-12 and abs(delayed.quality-0.7)<1e-12,"Range metadata not co-timed");
  end when;
  when time>=0.45 then
    assert(not delayed.valid and delayed.status==2 and delayed.sequence==3 and abs(delayed.distance)<1e-12,"Delayed no-return incorrect");
  end when;
  when time>=0.75 then
    assert(not delayed.valid and delayed.status==5 and delayed.sequence==6,"Device error must have its own delayed status");
  end when;
  when time>=0.85 then
    assert(delayed.valid and delayed.sequence==7,"Range recovery incorrect");
  end when;
  when sample(0.01,0.02) then
    assert(abs(repeatA.distance-repeatB.distance)<1e-14,"Range seed replay differs");
  end when;
  annotation(experiment(StopTime=1,Interval=0.002,Tolerance=1e-9));
end RangePhysics;
