within ModelicaCPSLibrary.Systems.Sensing.Magnetometer;
partial model PartialSensor "Mounted magnetometer value contract; magnetic inputs, bias and output in Tesla, sensor frame"
  parameter Real R_bs[3,3] = identity(3) "Body-to-sensor relative attitude; v_b = R_bs*v_s";
  parameter Real bias[3](each unit="T") = zeros(3) "Hard-iron offset";
  input Real R_wb[3,3];
  input Real magneticField_w[3](each unit="T");
  output Real magneticField[3](each unit="T");
protected
  Real field_s[3](each unit="T") = transpose(R_bs)*transpose(R_wb)*magneticField_w "True field, sensor frame";
initial equation
  assert(max(abs(transpose(R_bs)*R_bs-identity(3)))<1e-9,"Sensor mount must be a proper rotation");
end PartialSensor;
