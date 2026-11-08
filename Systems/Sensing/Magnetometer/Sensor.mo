within ModelicaCPSLibrary.Systems.Sensing.Magnetometer;

model Sensor "M0: continuous mounted magnetic field plus hard-iron offset"
  extends PartialSensor;
equation
  magneticField = field_s + bias;
end Sensor;
