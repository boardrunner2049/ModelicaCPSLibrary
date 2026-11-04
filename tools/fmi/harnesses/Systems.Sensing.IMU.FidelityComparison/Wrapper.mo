// Harness wrapper: the IMU fidelity ladder of Systems.Sensing.IMU side by side.
// One rigid-body motion drives the ideal sensor and every sampled fidelity:
//   stat     StatisticalSensor        I1 statistical (errors added to the calibrated truth)
//   digital  DigitalSensor            I1 deterministic device readout (no errors)
//   device   DeviceSensor             legacy I2: errors through the device readout
//   errors   ErrorSensor              I2: as device, plus the equivalentError input (default 0)
//   em       ElectromechanicalSensor  I3: physical MEMS axes, errors at the ADC
//   acoustic AcousticSensor           legacy I3: device + acoustic modes (pressure 0)
// All share the output clock (2 ms, latency 1 sample, set here because both are structural)
// and, except digital, the same stochastic error set with the same seed and stream, so they
// draw the same normals: differences between them are the deterministic readout/physics
// differences, which is what their error budgets bound. The device family uses a 0.5 ms
// analog bandwidth and ADC LSB 0.01 m/s2 / 0.001 rad/s, no IIR.
model FireHarness_ImuFidelities
  import ModelicaCPSLibrary.Systems.Sensing.IMU;
  input Real R_wb[3,3](start=identity(3));
  input Real a_w[3](each unit="m/s2");
  input Real gravity_w[3](each unit="m/s2");
  input Real omega_b[3](each unit="rad/s");
  input Real alpha_b[3](each unit="rad/s2");
  parameter Real noiseStd[6] = {0.02, 0.02, 0.02, 0.002, 0.002, 0.002};
  parameter Real biasStd[6] = {0.01, 0.01, 0.01, 0.0005, 0.0005, 0.0005};
  IMU.Sensor ideal;
  IMU.StatisticalSensor stat(p(samplePeriod=0.002, latencySamples=1, seed=11, streamId=5,
    noiseStd=noiseStd, biasStd=biasStd, biasTau=fill(1000, 6)));
  IMU.DigitalSensor digital(p(samplePeriod=0.002, latencySamples=1, analogTau=fill(0.0005, 6),
    quantum={0.01, 0.01, 0.01, 0.001, 0.001, 0.001}));
  IMU.DeviceSensor device(p(samplePeriod=0.002, latencySamples=1, seed=11, streamId=5,
    noiseStd=noiseStd, biasStd=biasStd, biasTau=fill(1000, 6), analogTau=fill(0.0005, 6),
    quantum={0.01, 0.01, 0.01, 0.001, 0.001, 0.001}));
  IMU.ErrorSensor errors(p(samplePeriod=0.002, latencySamples=1, seed=11, streamId=5,
    noiseStd=noiseStd, biasStd=biasStd, biasTau=fill(1000, 6), analogTau=fill(0.0005, 6),
    quantum={0.01, 0.01, 0.01, 0.001, 0.001, 0.001}));
  IMU.ElectromechanicalSensor em(p(samplePeriod=0.002, latencySamples=1, seed=11, streamId=5,
    noiseStd=noiseStd, biasStd=biasStd, biasTau=fill(1000, 6)));
  IMU.AcousticSensor acoustic(p(samplePeriod=0.002, latencySamples=1, seed=11, streamId=5,
    noiseStd=noiseStd, biasStd=biasStd, biasTau=fill(1000, 6), analogTau=fill(0.0005, 6),
    quantum={0.01, 0.01, 0.01, 0.001, 0.001, 0.001}),
    resonanceFrequency=fill(100, 6), dampingRatio=fill(0.1, 6), pressureGain=fill((2*Modelica.Constants.pi*100)^2, 6),
    displacementGain=ones(6), acousticPressure=0);
equation
  ideal.R_wb = R_wb; ideal.a_w = a_w; ideal.gravity_w = gravity_w; ideal.omega_b = omega_b; ideal.alpha_b = alpha_b;
  stat.R_wb = R_wb; stat.a_w = a_w; stat.gravity_w = gravity_w; stat.omega_b = omega_b; stat.alpha_b = alpha_b;
  digital.R_wb = R_wb; digital.a_w = a_w; digital.gravity_w = gravity_w; digital.omega_b = omega_b; digital.alpha_b = alpha_b;
  device.R_wb = R_wb; device.a_w = a_w; device.gravity_w = gravity_w; device.omega_b = omega_b; device.alpha_b = alpha_b;
  errors.R_wb = R_wb; errors.a_w = a_w; errors.gravity_w = gravity_w; errors.omega_b = omega_b; errors.alpha_b = alpha_b;
  em.R_wb = R_wb; em.a_w = a_w; em.gravity_w = gravity_w; em.omega_b = omega_b; em.alpha_b = alpha_b;
  acoustic.R_wb = R_wb; acoustic.a_w = a_w; acoustic.gravity_w = gravity_w; acoustic.omega_b = omega_b; acoustic.alpha_b = alpha_b;
end FireHarness_ImuFidelities;
