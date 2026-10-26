within fire_modelica_models.Tests;
model ImuElectromechanical "Three physical axes; rate is recovered from Coriolis displacement"
  Systems.Sensing.IMU.ElectromechanicalSensor imu(p(samplePeriod=0.002));
  Systems.Sensing.IMU.VibratoryGyro startup(p(startupFraction=0.1,driveFrequency=1003,referenceFrequency=1000));
equation
  imu.R_wb=identity(3);
  imu.a_w={1,-2,0};
  imu.gravity_w={0,0,9.80665};
  imu.omega_b={0.2,-0.3,0.1};
  imu.alpha_b=zeros(3);
  startup.angularRate=0;
  when time>=0.3 then
    assert(max(abs(imu.gyro-{0.2,-0.3,0.1}))<0.003,"Physical gyro scale/sign or demodulation failed");
    assert(max(abs(imu.acceleration-{1,-2,-9.80665}))<1e-7,"Physical proof-mass readout failed");
  end when;
  when time>=0.9 then
    assert(startup.locked and abs(startup.driveControl.amplitude/startup.p.driveAmplitude-1)<0.02,
      "AGC/PLL startup did not converge");
    assert(abs(startup.driveControl.frequency/(2*Modelica.Constants.pi)-1003)<0.5,"PLL did not track mechanical detuning");
  end when;
  annotation(experiment(StopTime=1,Interval=0.001,Tolerance=1e-9));
end ImuElectromechanical;
