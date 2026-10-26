within fire_modelica_models.Systems.Sensing.IMU;
model AcousticSensor "Legacy I3: pressure-driven identified MEMS modes through the I2 readout"
  extends PartialDeviceSensor;
  parameter Real resonanceFrequency[6](each unit="Hz");
  parameter Real dampingRatio[6];
  parameter Real pressureGain[6];
  parameter Real displacementGain[6];
  parameter Real velocityGain[6] = zeros(6);
  parameter Real quadraticGain[6] = zeros(6);
  input Real acousticPressure(unit="Pa") = 0 "Pressure at this sensor; source/propagation belongs to scenario";
  fire_modelica_models.Physical.Sensors.MEMS.ResonantCoupling coupling(
    frequency=resonanceFrequency,damping=dampingRatio,pressureGain=pressureGain,
    displacementGain=displacementGain,velocityGain=velocityGain,quadraticGain=quadraticGain);
equation
  coupling.pressure=acousticPressure;
  acousticError=coupling.error;
end AcousticSensor;
