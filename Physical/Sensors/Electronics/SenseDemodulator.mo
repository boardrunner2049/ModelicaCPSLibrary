within fire_modelica_models.Physical.Sensors.Electronics;
model SenseDemodulator "Open-loop I/Q detection with fixed nominal scale and phase calibration"
  parameter fire_modelica_models.Physical.Sensors.MEMS.GyroParameters p;
  input Real measuredDisplacement(unit="m");
  input Real phase(unit="rad");
  output Real rate(unit="rad/s");
  output Real quadrature(unit="m");
  final parameter Real w=2*Modelica.Constants.pi*p.referenceFrequency;
  final parameter Real ws=2*Modelica.Constants.pi*p.senseFrequency;
  final parameter Real stiffness=p.mass*ws^2;
  final parameter Real damping=p.mass*ws/p.senseQ;
  final parameter Real phaseShift=Modelica.Math.atan2(-damping*w,stiffness-p.mass*w^2)-atan(w*p.afeTau)+p.demodPhaseError;
  final parameter Real scale=2*p.mass*p.driveAmplitude*w/
    sqrt((stiffness-p.mass*w^2)^2+(damping*w)^2)/sqrt(1+(w*p.afeTau)^2);
  Real i1(nominal=scale,start=0,fixed=true);
  Real i2(nominal=scale,start=0,fixed=true);
  Real q1(nominal=scale,start=0,fixed=true);
  Real q2(nominal=scale,start=0,fixed=true);
initial equation
  assert(p.senseTau>0,"Sense demodulator time constant must be positive");
equation
  p.senseTau*der(i1)+i1=2*measuredDisplacement*cos(phase+phaseShift);
  p.senseTau*der(i2)+i2=i1;
  p.senseTau*der(q1)+q1=2*measuredDisplacement*sin(phase+phaseShift);
  p.senseTau*der(q2)+q2=q1;
  rate=-i2/scale;
  quadrature=q2;
end SenseDemodulator;
