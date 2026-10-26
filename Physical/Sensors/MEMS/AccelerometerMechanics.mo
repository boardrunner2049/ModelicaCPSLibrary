within fire_modelica_models.Physical.Sensors.MEMS;
model AccelerometerMechanics "Specific force excites a damped proof mass"
  parameter AccelerometerParameters p;
  input Real specificForce(unit="m/s2");
  input Real externalForce(unit="N");
  output Real displacement(unit="m", nominal=10/(2*Modelica.Constants.pi*p.frequency)^2);
  output Real velocity(unit="m/s", nominal=10/(2*Modelica.Constants.pi*p.frequency), start=0, fixed=true);
  final parameter Real w=2*Modelica.Constants.pi*p.frequency;
initial equation
  assert(p.mass>0 and p.frequency>0 and p.quality>0,"Invalid accelerometer mechanics");
  displacement=(-specificForce+externalForce/p.mass)/w^2;
equation
  der(displacement)=velocity;
  p.mass*der(velocity)+p.mass*w/p.quality*velocity+p.mass*w^2*displacement=
    -p.mass*specificForce+externalForce;
end AccelerometerMechanics;
