within fire_modelica_models.Tests;
model ImuAcoustic "Illustrative pressure resonance through anti-alias response and ADC; not hardware calibration"
  parameter Real toneFrequency=110;
  parameter Real modeFrequency=100;
  parameter Real modeDamping=0.1;
  parameter Real tau=0.0001;
  final parameter Real w0=2*Modelica.Constants.pi*modeFrequency;
  Systems.Sensing.IMU.AcousticSensor acoustic(acousticPressure=sin(2*Modelica.Constants.pi*toneFrequency*time),
    p(samplePeriod=0.01,analogTau=fill(tau,6)),
    resonanceFrequency=fill(modeFrequency,6),dampingRatio=fill(modeDamping,6),
    pressureGain={w0^2,0,0,0,0,0},displacementGain=ones(6));
  Systems.Sensing.IMU.AcousticSensor quiet(
    p(samplePeriod=0.01,analogTau=fill(tau,6)),
    resonanceFrequency=fill(modeFrequency,6),dampingRatio=fill(modeDamping,6),
    pressureGain={w0^2,0,0,0,0,0},displacementGain=ones(6));
  Systems.Sensing.IMU.AcousticSensor rectified(acousticPressure=acoustic.acousticPressure,
    p(samplePeriod=0.01,analogTau=fill(tau,6)),
    resonanceFrequency=fill(modeFrequency,6),dampingRatio=fill(modeDamping,6),
    pressureGain={w0^2,0,0,0,0,0},displacementGain=zeros(6),quadraticGain=ones(6));
  Systems.Sensing.IMU.DeviceSensor device(p(samplePeriod=0.01,analogTau=fill(tau,6)));
equation
  acoustic.R_wb=identity(3);
  acoustic.a_w=zeros(3);
  acoustic.gravity_w=zeros(3);
  acoustic.omega_b=zeros(3);
  acoustic.alpha_b=zeros(3);
  quiet.R_wb=identity(3);
  quiet.a_w=zeros(3);
  quiet.gravity_w=zeros(3);
  quiet.omega_b=zeros(3);
  quiet.alpha_b=zeros(3);
  rectified.R_wb=identity(3);
  rectified.a_w=zeros(3);
  rectified.gravity_w=zeros(3);
  rectified.omega_b=zeros(3);
  rectified.alpha_b=zeros(3);
  device.R_wb=identity(3);
  device.a_w=zeros(3);
  device.gravity_w=zeros(3);
  device.omega_b=zeros(3);
  device.alpha_b=zeros(3);
  when sample(0.005,0.01) then
    assert(max(abs(quiet.acceleration-device.acceleration))<1e-12,"Zero pressure must recover I2");
    assert(abs(acoustic.acceleration[2])+abs(acoustic.acceleration[3])+sum(abs(acoustic.gyro))<1e-12,"Acoustic channels coupled without coefficients");
  end when;
  annotation(experiment(StopTime=1,Interval=0.001,Tolerance=1e-9));
end ImuAcoustic;
