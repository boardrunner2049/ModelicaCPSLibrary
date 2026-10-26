within fire_modelica_models.Systems.Sensing.IMU;
model StatisticalSensor "Legacy I1: calibrated rigid motion, sampled statistical errors and delivery"
  extends PartialSampledSensor(final deviceReadout=false);
equation
  frontEnd=calibrated;
end StatisticalSensor;
