within ModelicaCPSLibrary.Systems.Sensing.Magnetometer;

model Sensor "Continuous mounted magnetic field; magnetic inputs and bias are in Tesla"
  parameter Real R_bs[3,3] = identity(3) "Body-to-sensor relative attitude; v_b = R_bs*v_s";
  parameter Real bias[3](each unit="T") = zeros(3);
  input Real R_wb[3,3];
  input Real magneticField_w[3](each unit="T");
  output Real magneticField[3](each unit="T");

initial equation
  assert(max(abs(transpose(R_bs)*R_bs-identity(3)))<1e-9,"Sensor mount must be a proper rotation");
    // and abs(Modelica.Math.Matrices.det(R_bs)-1)<1e-9
equation
  magneticField = transpose(R_bs)*transpose(R_wb)*magneticField_w + bias;
end Sensor;
