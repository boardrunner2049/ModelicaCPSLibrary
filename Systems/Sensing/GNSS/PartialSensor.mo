within fire_modelica_models.Systems.Sensing.GNSS;
partial model PartialSensor "Antenna-point local NED PVT contract; mounting belongs to caller"
  parameter Real positionBias[3](each unit="m")=zeros(3);
  parameter Real velocityBias[3](each unit="m/s")=zeros(3);
  input Real position_w[3](each unit="m");
  input Real velocity_w[3](each unit="m/s");
  output Real position[3](each unit="m");
  output Real velocity[3](each unit="m/s");
end PartialSensor;
