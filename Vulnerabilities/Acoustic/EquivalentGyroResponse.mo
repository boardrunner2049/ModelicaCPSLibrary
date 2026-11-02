within ModelicaCPSLibrary.Vulnerabilities.Acoustic;
model EquivalentGyroResponse "MFQuadrotor-inspired steady carrier response and baseband demodulation; no AGC/PLL transients"
  parameter ModelicaCPSLibrary.Physical.Sensors.MEMS.GyroParameters gyroPhysics;
  parameter Real frequency(unit="Hz")=1000;
  parameter Real phase(unit="rad")=0;
  parameter Real effectiveArea(unit="m2")=1e-10;
  parameter Real incidenceAngle(unit="rad")=0;
  parameter Real driveCoupling[3]=zeros(3);
  parameter Real senseCoupling[3]={0,0,1};
  input Real rmsPressure(unit="Pa")=0 "Slowly varying RMS envelope, NOT a sampled carrier";
  output Real equivalentError[6];
  final parameter Real w=2*Modelica.Constants.pi*frequency;
  final parameter Real wr=2*Modelica.Constants.pi*gyroPhysics.referenceFrequency;
  final parameter Real wd=2*Modelica.Constants.pi*gyroPhysics.driveFrequency;
  final parameter Real ws=2*Modelica.Constants.pi*gyroPhysics.senseFrequency;
  final parameter Real m=gyroPhysics.mass;
  final parameter Real cd=m*wd/gyroPhysics.driveQ;
  final parameter Real cs=m*ws/gyroPhysics.senseQ;
  final parameter Real ad=m*(wd^2-w^2);
  final parameter Real as_=m*(ws^2-w^2);
  final parameter Real bd=cd*w;
  final parameter Real bs=cs*w;
  final parameter Real denRe=ad*as_-bd*bs-gyroPhysics.crossStiffness^2;
  final parameter Real denIm=ad*bs+bd*as_;
  final parameter Real den=denRe^2+denIm^2;
  final parameter Real refPhase=Modelica.Math.atan2(-cs*wr,m*(ws^2-wr^2))-atan(wr*gyroPhysics.afeTau)+gyroPhysics.demodPhaseError;
  final parameter Real scale=2*m*gyroPhysics.driveAmplitude*wr/
    sqrt((m*(ws^2-wr^2))^2+(cs*wr)^2)/sqrt(1+(wr*gyroPhysics.afeTau)^2);
  PressureToForce coupling(pressure=sqrt(2)*rmsPressure,effectiveArea=effectiveArea,incidenceAngle=incidenceAngle,
    driveCoupling=driveCoupling,senseCoupling=senseCoupling);
  Real responseRe[3];
  Real responseIm[3];
  Real numRe[3];
  Real numIm[3];
  Real filtered1[3](each start=0,each fixed=true);
  Real filtered2[3](each start=0,each fixed=true);
  Real rawError[3];
  Real delta;
initial equation
  assert(frequency>0 and gyroPhysics.mass>0 and gyroPhysics.driveFrequency>0 and gyroPhysics.senseFrequency>0 and
    gyroPhysics.referenceFrequency>0 and gyroPhysics.driveQ>0 and gyroPhysics.senseQ>0 and
    gyroPhysics.driveAmplitude>0 and gyroPhysics.afeTau>0 and gyroPhysics.senseTau>0,
    "Invalid equivalent gyro response parameters");
  assert(gyroPhysics.crossStiffness^2<m^2*wd^2*ws^2 and den>0,"Invalid coupled modal response");
equation
  assert(rmsPressure>=0,"Equivalent response expects nonnegative RMS pressure");
  delta=(w-wr)*time+phase-refPhase;
  numRe=ad*coupling.gyroSenseForce-gyroPhysics.crossStiffness*coupling.gyroDriveForce;
  numIm=bd*coupling.gyroSenseForce;
  responseRe=(numRe*denRe+numIm*denIm)/den;
  responseIm=(numIm*denRe-numRe*denIm)/den;
  rawError=-((responseRe+w*gyroPhysics.afeTau*responseIm)*cos(delta)-
    (responseIm-w*gyroPhysics.afeTau*responseRe)*sin(delta))/((1+(w*gyroPhysics.afeTau)^2)*scale);
  gyroPhysics.senseTau*der(filtered1)+filtered1=rawError;
  gyroPhysics.senseTau*der(filtered2)+filtered2=filtered1;
  equivalentError[1:3]=zeros(3);
  equivalentError[4:6]=filtered2;
end EquivalentGyroResponse;
