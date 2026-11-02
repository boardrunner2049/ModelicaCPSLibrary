within ModelicaCPSLibrary.Physical.Sensors.Electronics;
model DriveControl "Measured I/Q drives bounded PI AGC and PLL, with anti-windup"
  parameter ModelicaCPSLibrary.Physical.Sensors.MEMS.GyroParameters p;
  input Real measuredDisplacement(unit="m");
  output Real phase(unit="rad", start=0, fixed=true);
  output Real frequency(unit="rad/s");
  output Real voltage(unit="V");
  output Real amplitude(unit="m");
  output Real amplitudeError;
  output Real phaseError(unit="rad");
  output Boolean locked;
  final parameter Real w0=2*Modelica.Constants.pi*p.referenceFrequency;
  final parameter Real phaseSet=-atan(w0*p.afeTau);
  final parameter Real gainCorrection=sqrt(1+(w0*p.afeTau)^2);
  final parameter Real forceGain=2*p.driveC0*p.biasVoltage/p.driveLength;
  final parameter Real u0=(p.mass*2*Modelica.Constants.pi*p.driveFrequency/p.driveQ)*w0*p.driveAmplitude/forceGain;
  Real i1(nominal=p.driveAmplitude,start=-p.startupFraction*p.driveAmplitude*w0*p.afeTau/(1+(w0*p.afeTau)^2), fixed=true);
  Real i2(nominal=p.driveAmplitude,start=-p.startupFraction*p.driveAmplitude*w0*p.afeTau/(1+(w0*p.afeTau)^2), fixed=true);
  Real q1(nominal=p.driveAmplitude,start=p.startupFraction*p.driveAmplitude/(1+(w0*p.afeTau)^2), fixed=true);
  Real q2(nominal=p.driveAmplitude,start=p.startupFraction*p.driveAmplitude/(1+(w0*p.afeTau)^2), fixed=true);
  Real agcIntegral(start=0,fixed=true);
  Real pllIntegral(start=0,fixed=true);
  Real rawVoltage;
  Real driveVoltage;
  Real rawFrequency;
initial equation
  assert(p.referenceFrequency>0 and p.driveAmplitude>0 and p.detectorTau>0 and p.afeTau>0,
    "Invalid drive reference/detector parameters");
  assert(p.agcKp>=0 and p.agcKi>=0 and p.pllKp>=0 and p.pllKi>=0 and p.pllRange>0 and p.pllRange<1,
    "Invalid AGC/PLL gains or range");
  assert(p.startupFraction>=0 and p.startupFraction<=1 and p.maxDriveVoltage>0 and u0<p.maxDriveVoltage,
    "Invalid startup fraction or insufficient drive voltage headroom");
equation
  p.detectorTau*der(i1)+i1=2*measuredDisplacement*cos(phase);
  p.detectorTau*der(i2)+i2=i1;
  p.detectorTau*der(q1)+q1=2*measuredDisplacement*sin(phase);
  p.detectorTau*der(q2)+q2=q1;
  amplitude=sqrt(i2^2+q2^2)*gainCorrection;
  amplitudeError=1-amplitude/p.driveAmplitude;
  rawVoltage=u0*(1+p.agcKp*amplitudeError+agcIntegral);
  driveVoltage=min(p.maxDriveVoltage,max(0,rawVoltage));
  der(agcIntegral)=if noEvent((rawVoltage>=p.maxDriveVoltage and amplitudeError>0) or
    (rawVoltage<=0 and amplitudeError<0)) then 0 else p.agcKi*amplitudeError;
  phaseError=if noEvent(amplitude>1e-4*p.driveAmplitude) then
    Modelica.Math.atan2(sin(Modelica.Math.atan2(i2,q2)-phaseSet),cos(Modelica.Math.atan2(i2,q2)-phaseSet)) else 0;
  rawFrequency=w0+p.pllKp*phaseError+pllIntegral;
  frequency=min(w0*(1+p.pllRange),max(w0*(1-p.pllRange),rawFrequency));
  der(pllIntegral)=if noEvent((rawFrequency>=w0*(1+p.pllRange) and phaseError>0) or
    (rawFrequency<=w0*(1-p.pllRange) and phaseError<0)) then 0 else p.pllKi*phaseError;
  der(phase)=frequency;
  voltage=driveVoltage*cos(phase);
  locked=noEvent(abs(amplitudeError)<0.05 and abs(phaseError)<0.05);
end DriveControl;
