within ModelicaCPSLibrary.Systems.Payloads;
partial model PartialSlungLoad "A point-mass load on a line from the attachment: tension only (spring-damper stretch), gravity, ground contact and friction for the load"
  extends PartialPayload;
  parameter Real mass(unit="kg") = 1 "Load";
  parameter Real stiffness(unit="N/m") = 1000 "Line";
  parameter Real damping(unit="N.s/m") = 30 "Line";
  parameter Real groundDown(unit="m") = 0 "Ground plane, NED down";
  parameter Real groundStiffness(unit="N/m") = 3000;
  parameter Real groundDamping(unit="N.s/m") = 150;
  parameter Real groundFriction(unit="N.s/m") = 50 "Horizontal damping of the load on the ground";
  output Real load_w[3](each unit="m") "Load position, NED";
  output Real tension(unit="N");
  output Real lineLength(unit="m") "Distance from the attachment to the load";
protected
  Real p[3](each unit="m", each fixed=false);
  Real v[3](each unit="m/s", each start=0, each fixed=true);
  Real rest(unit="m") "Unstretched line length";
  Boolean attached "The line carries the load";
  Real d[3](each unit="m") "Load to attachment";
  Real stretchRate(unit="m/s");
  Real ground(unit="N") "Ground reaction on the load, up";
  Real lineForce_w[3](each unit="N") "On the load";
initial equation
  // hanging at rest under the attachment, or on the ground below it
  p = {attach_w[1], attach_w[2], min(attach_w[3] + rest + mass*gravity/stiffness, groundDown)};
equation
  der(p) = v;
  d = attach_w - p;
  lineLength = sqrt(d*d + 1e-9);
  stretchRate = d*(attachVelocity_w - v)/lineLength;
  tension = if attached and lineLength > rest then max(0, stiffness*(lineLength - rest) + damping*stretchRate) else 0;
  lineForce_w = tension*d/lineLength;
  ground = if p[3] > groundDown then max(0, groundStiffness*(p[3] - groundDown) + groundDamping*v[3]) else 0;
  mass*der(v) = lineForce_w + {0, 0, mass*gravity} + {-groundFriction*v[1]*(if ground > 0 then 1 else 0),
    -groundFriction*v[2]*(if ground > 0 then 1 else 0), -ground};
  load_w = p;
  force_b = -transpose(truth.R_wb)*lineForce_w;
  moment_b = cross(position_b, force_b);
end PartialSlungLoad;
