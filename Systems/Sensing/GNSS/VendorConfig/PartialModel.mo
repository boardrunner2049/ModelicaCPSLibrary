within ModelicaCPSLibrary.Systems.Sensing.GNSS.VendorConfig;
partial model PartialModel "Measured-value boundary; no truth, mount or physical biases"
  input Real measuredPosition[3](each unit="m") "Measured antenna position, local NED";
  input Real measuredVelocity[3](each unit="m/s") "Measured antenna velocity, local NED";
  output Real position[3](each unit="m");
  output Real velocity[3](each unit="m/s");
end PartialModel;
