within fire_modelica_models.Tests;
model ImuDigitalErrors "I2 zero-error reduction to I1, including ADC, decimation and queue"
  parameter Systems.Sensing.IMU.Parameters p(samplePeriod=0.004,adcDecimation=4,latencySamples=2,
    analogTau=fill(0.002,6),digitalTau=fill(0.003,6),quantum=fill(0.01,6));
  Systems.Sensing.IMU.DigitalSensor digital(p=p);
  Systems.Sensing.IMU.ErrorSensor zeroErrors(p=p);
  Systems.Sensing.IMU.ErrorSensor calibrated(p(samplePeriod=0.004,adcDecimation=4,
    scale={0.1,0,0,0,0,0},accelMisalignment=[0,0.2,0;0,0,0;0,0,0],gyroGSensitivity=identity(3),
    temperatureCoefficient={0.1,0,0,0,0,0}),accelBias={0.3,0,0},sensorTemperature=298.15);
equation
  digital.R_wb=identity(3);
  digital.a_w={sin(20*time),2*cos(10*time),1};
  digital.gravity_w={0,0,9.80665};
  digital.omega_b={time,-time,0.2};
  digital.alpha_b={1,-1,0};
  zeroErrors.R_wb=digital.R_wb;
  zeroErrors.a_w=digital.a_w;
  zeroErrors.gravity_w=digital.gravity_w;
  zeroErrors.omega_b=digital.omega_b;
  zeroErrors.alpha_b=digital.alpha_b;
  calibrated.R_wb=identity(3);
  calibrated.a_w={1,2,3};
  calibrated.gravity_w=zeros(3);
  calibrated.omega_b={4,5,6};
  calibrated.alpha_b=zeros(3);
  when sample(0.0005,0.001) then
    assert(max(abs(digital.acceleration-zeroErrors.acceleration))+max(abs(digital.gyro-zeroErrors.gyro))<1e-12,
      "I2 with zero errors must exactly reduce to I1");
    assert(digital.valid==zeroErrors.valid and digital.sequence==zeroErrors.sequence and
      abs(digital.sampleTime-zeroErrors.sampleTime)<1e-15 and abs(digital.deliveryTime-zeroErrors.deliveryTime)<1e-15,
      "I1/I2 sample contracts differ");
  end when;
  when time>=0.02 then
    assert(abs(calibrated.acceleration[1]-2.3)<1e-9 and max(abs(calibrated.gyro-{5,7,9}))<1e-9,
      "I2 calibration/temperature/g sensitivity failed");
  end when;
  annotation(experiment(StopTime=0.2,Interval=0.001,Tolerance=1e-9));
end ImuDigitalErrors;
