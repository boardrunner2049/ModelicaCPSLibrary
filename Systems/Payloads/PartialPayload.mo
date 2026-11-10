within ModelicaCPSLibrary.Systems.Payloads;
partial model PartialPayload "A payload attached to the vehicle: the force and moment it puts on the vehicle"
  parameter Real position_b[3](each unit="m") = zeros(3) "Attachment, body FRD from the combined CG [configuration]";
  parameter Real gravity(unit="m/s2") = 9.80665 "[physics]";
  input Interfaces.VehicleState truth "Vehicle truth at the combined CG";
  output Real force_b[3](each unit="N") "On the vehicle at the attachment, body FRD";
  output Real moment_b[3](each unit="N.m") "On the vehicle about the combined CG, body FRD";
protected
  Real attach_w[3](each unit="m") "Attachment, NED";
  Real attachVelocity_w[3](each unit="m/s");
equation
  attach_w = truth.p_w + truth.R_wb*position_b;
  attachVelocity_w = truth.v_w + truth.R_wb*cross(truth.omega_b, position_b);
end PartialPayload;
