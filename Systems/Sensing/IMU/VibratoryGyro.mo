within ModelicaCPSLibrary.Systems.Sensing.IMU;
model VibratoryGyro "One physical rate axis: mechanics, electrodes, C/V, AGC, PLL and sense demodulation"
  parameter ModelicaCPSLibrary.Physical.Sensors.MEMS.GyroParameters p;
  input Real angularRate(unit="rad/s");
  input Real externalDriveForce(unit="N")=0;
  input Real externalSenseForce(unit="N")=0;
  output Real measuredRate(unit="rad/s");
  output Boolean locked;
  ModelicaCPSLibrary.Physical.Sensors.MEMS.GyroMechanics mechanics(p=p);
  ModelicaCPSLibrary.Physical.Sensors.Electronics.CombDrive actuator(C0=p.driveC0,length=p.driveLength,bias=p.biasVoltage);
  ModelicaCPSLibrary.Physical.Sensors.Electronics.CapacitiveReadout driveReadout(
    C0=p.readC0,length=p.readLength,carrierVoltage=p.carrierVoltage,Cf=p.feedbackCapacitance,tau=p.afeTau,rail=p.rail);
  ModelicaCPSLibrary.Physical.Sensors.Electronics.CapacitiveReadout senseReadout(
    C0=p.readC0,length=p.readLength,carrierVoltage=p.carrierVoltage,Cf=p.feedbackCapacitance,tau=p.afeTau,rail=p.rail);
  ModelicaCPSLibrary.Physical.Sensors.Electronics.DriveControl driveControl(p=p);
  ModelicaCPSLibrary.Physical.Sensors.Electronics.SenseDemodulator demodulator(p=p);
equation
  mechanics.angularRate=angularRate;
  mechanics.driveForce=actuator.force+externalDriveForce;
  mechanics.senseForce=externalSenseForce;
  actuator.displacement=mechanics.x;
  actuator.command=driveControl.voltage;
  driveReadout.displacement=mechanics.x;
  senseReadout.displacement=mechanics.y;
  driveControl.measuredDisplacement=driveReadout.measuredDisplacement;
  demodulator.measuredDisplacement=senseReadout.measuredDisplacement;
  demodulator.phase=driveControl.phase;
  measuredRate=demodulator.rate;
  locked=driveControl.locked and not driveReadout.saturated and not senseReadout.saturated;
end VibratoryGyro;
