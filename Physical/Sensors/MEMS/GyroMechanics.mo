within fire_modelica_models.Physical.Sensors.MEMS;
model GyroMechanics "Reciprocal Coriolis coupling, drive/sense proof-mass dynamics"
  parameter GyroParameters p;
  input Real angularRate(unit="rad/s");
  input Real driveForce(unit="N");
  input Real senseForce(unit="N");
  output Real x(unit="m", nominal=p.driveAmplitude, start=0, fixed=true);
  output Real vx(unit="m/s", nominal=2*Modelica.Constants.pi*p.driveFrequency*p.driveAmplitude, start=p.startupFraction*p.driveAmplitude*2*Modelica.Constants.pi*p.referenceFrequency, fixed=true);
  output Real y(unit="m", nominal=0.001*p.driveAmplitude, start=0, fixed=true);
  output Real vy(unit="m/s", nominal=0.001*2*Modelica.Constants.pi*p.senseFrequency*p.driveAmplitude, start=0, fixed=true);
  final parameter Real wd=2*Modelica.Constants.pi*p.driveFrequency;
  final parameter Real ws=2*Modelica.Constants.pi*p.senseFrequency;
  final parameter Real kd=p.mass*wd^2;
  final parameter Real ks=p.mass*ws^2;
  final parameter Real cd=p.mass*wd/p.driveQ;
  final parameter Real cs=p.mass*ws/p.senseQ;
initial equation
  assert(p.mass>0 and p.driveFrequency>0 and p.senseFrequency>0 and p.driveQ>0 and p.senseQ>0,
    "Invalid gyro mass/frequency/Q");
  assert(p.crossStiffness^2<kd*ks,"Gyro stiffness matrix must be positive definite");
equation
  der(x)=vx;
  der(y)=vy;
  p.mass*der(vx)+cd*vx+kd*x+p.crossStiffness*y=driveForce+2*p.mass*angularRate*vy;
  p.mass*der(vy)+cs*vy+ks*y+p.crossStiffness*x=senseForce-2*p.mass*angularRate*vx;
end GyroMechanics;
