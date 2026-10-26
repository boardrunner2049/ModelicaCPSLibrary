within fire_modelica_models.Tests;
model ImuAcousticCoupling "Independent static coupling; compare physical and steady-state surrogate demodulation"
  parameter Physical.Sensors.MEMS.GyroParameters gp;
  parameter Real w0=2*Modelica.Constants.pi*gp.referenceFrequency;
  parameter Real nominalForce=gp.mass*2*Modelica.Constants.pi*gp.driveFrequency/gp.driveQ*w0*gp.driveAmplitude;
  Vulnerabilities.Acoustic.SoundPowerToPressure power(acousticPower=4*Modelica.Constants.pi/(1.21*343),distance=1);
  Vulnerabilities.Acoustic.SoundPowerToPressure off;
  Vulnerabilities.Acoustic.TonePressure tone(frequency=1001,phase=0.3,rmsPressure=0.01);
  Vulnerabilities.Acoustic.PressureToForce coupling(pressure=tone.pressure);
  Vulnerabilities.Acoustic.PressureToForce zero(pressure=0);
  Vulnerabilities.Acoustic.EquivalentGyroResponse equivalent(gyroPhysics=gp,frequency=1001,phase=0.3,rmsPressure=0.01);
  Vulnerabilities.Acoustic.EquivalentGyroResponse equivalentOff(gyroPhysics=gp,frequency=1001,rmsPressure=0);
  Systems.Sensing.IMU.VibratoryGyro forced(p=gp,angularRate=0,
    externalDriveForce=coupling.gyroDriveForce[3],externalSenseForce=coupling.gyroSenseForce[3]);
  Systems.Sensing.IMU.VibratoryGyro quiet(p=gp,angularRate=0);
  Systems.Sensing.IMU.VibratoryGyro driveForced(p=gp,angularRate=0,
    externalDriveForce=0.5*nominalForce*cos(w0*time));
equation
  assert(abs(power.rmsPressure-1)<1e-12 and abs(off.rmsPressure)<1e-30,"Power-to-pressure units or zero-power handling failed");
  assert(sum(abs(zero.gyroDriveForce))+sum(abs(zero.gyroSenseForce))+sum(abs(zero.accelForce))<1e-30,
    "Zero pressure must produce zero force");
  assert(sum(abs(equivalentOff.equivalentError))<1e-30,"Equivalent vulnerability must not inject nominal bias");
  assert(abs(quiet.measuredRate)<1e-12,"Quiet zero-rate gyro must have zero output");
  when time>=0.4 then
    assert(abs(driveForced.driveControl.amplitude/gp.driveAmplitude-1)<0.02,"AGC failed to reject drive disturbance");
    assert(driveForced.driveControl.driveVoltage<0.65*quiet.driveControl.driveVoltage,"AGC did not reduce electrostatic drive");
  end when;
  annotation(experiment(StopTime=0.5,Interval=0.0005,Tolerance=1e-9));
end ImuAcousticCoupling;
