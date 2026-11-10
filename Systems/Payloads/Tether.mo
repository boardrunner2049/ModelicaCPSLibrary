within ModelicaCPSLibrary.Systems.Payloads;
model Tether "A power tether to a ground station: its winch keeps a constant tension toward the anchor, the vehicle carries half the hanging tether's weight, and it supplies power up to its limit"
  extends PartialPayload;
  parameter Real anchor_w[3](each unit="m") = zeros(3) "Ground station, NED [configuration]";
  parameter Real tension0(unit="N") = 5 "Winch tension [assumed]";
  parameter Real massPerLength(unit="kg/m") = 0.02 "Tether [datasheet]";
  parameter Real maxLength(unit="m") = 100 "[datasheet]";
  parameter Real maxPower(unit="W") = 2000 "Power through the tether [datasheet]";
  input Real utilization = 0 "Fraction of its capacity the vehicle draws (the power module's)";
  output Real length(unit="m");
  output Real capacity(unit="W") "Power it can deliver";
  output Real supply(unit="W") "Power delivered";
  output Real overLength "1 beyond maxLength";
protected
  Real d[3](each unit="m");
  Real f_w[3](each unit="N");
equation
  d = anchor_w - attach_w;
  length = sqrt(d*d + 1e-9);
  overLength = if length > maxLength then 1 else 0;
  // beyond its length the tether stops the vehicle like a stiff line
  f_w = (tension0 + (if length > maxLength then 2000*(length - maxLength) else 0))*d/length
    + {0, 0, massPerLength*min(length, maxLength)*gravity/2};
  force_b = transpose(truth.R_wb)*f_w;
  moment_b = cross(position_b, force_b);
  capacity = maxPower;
  supply = capacity*utilization;
end Tether;
