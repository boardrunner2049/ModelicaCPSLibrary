within ModelicaCPSLibrary.Systems.Peripherals;
partial model PartialPeripheral "A device rigidly mounted on the vehicle and fed its truth"
  parameter Real position_b[3](each unit="m") = zeros(3) "Mount position from the combined CG, body FRD";
  parameter Real R_bs[3,3] = identity(3) "Device axes to body FRD (v_b = R_bs v_s)";
  input Interfaces.VehicleState truth "Vehicle truth at the combined CG";
protected
  Real mountPosition_w[3](each unit="m") "Mount position, NED";
  Real mountVelocity_b[3](each unit="m/s") "Mount velocity over ground, body FRD";
equation
  mountPosition_w = truth.p_w + truth.R_wb*position_b;
  mountVelocity_b = truth.v_b + cross(truth.omega_b, position_b);
end PartialPeripheral;
