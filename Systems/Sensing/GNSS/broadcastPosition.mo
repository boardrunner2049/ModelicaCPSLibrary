within fire_modelica_models.Systems.Sensing.GNSS;
function broadcastPosition "GPS broadcast ECEF orbit and L1 code clock, including relativity"
  input BroadcastEphemeris eph;
  input Real epoch(unit="s") "GNSS seconds of week, never UTC or wall clock";
  output Real position[3];
  output Real clockBias(unit="s");
  output Boolean valid;
protected
  Real tk;
  Real tc;
  Real a;
  Real M;
  Real E;
  Real phi;
  Real u;
  Real r;
  Real inc;
  Real node;
  Real x;
  Real y;
algorithm
  assert(eph.sqrtA>0 and eph.eccentricity>=0 and eph.eccentricity<1 and eph.fitInterval>0,
    "Invalid broadcast ephemeris");
  tk:=mod(epoch-eph.toe+302400,604800)-302400;
  tc:=mod(epoch-eph.toc+302400,604800)-302400;
  a:=eph.sqrtA^2;
  M:=mod(eph.meanAnomaly+(sqrt(3.986005e14/a^3)+eph.deltaN)*tk+Modelica.Constants.pi,
    2*Modelica.Constants.pi)-Modelica.Constants.pi;
  E:=if eph.eccentricity<0.8 then M else if M>=0 then Modelica.Constants.pi else -Modelica.Constants.pi;
  for k in 1:20 loop
    E:=E-(E-eph.eccentricity*sin(E)-M)/(1-eph.eccentricity*cos(E));
  end for;
  phi:=atan2(sqrt(1-eph.eccentricity^2)*sin(E),cos(E)-eph.eccentricity)+eph.argumentOfPerigee;
  u:=phi+eph.cuc*cos(2*phi)+eph.cus*sin(2*phi);
  r:=a*(1-eph.eccentricity*cos(E))+eph.crc*cos(2*phi)+eph.crs*sin(2*phi);
  inc:=eph.inclination+eph.inclinationDot*tk+eph.cic*cos(2*phi)+eph.cis*sin(2*phi);
  node:=eph.omega0+(eph.omegaDot-7.2921151467e-5)*tk-7.2921151467e-5*eph.toe;
  x:=r*cos(u);
  y:=r*sin(u);
  position:={x*cos(node)-y*cos(inc)*sin(node),x*sin(node)+y*cos(inc)*cos(node),y*sin(inc)};
  clockBias:=eph.af0+eph.af1*tc+eph.af2*tc^2-4.442807633e-10*eph.eccentricity*eph.sqrtA*sin(E)-eph.tgd;
  valid:=eph.healthy and abs(tk)<=eph.fitInterval/2 and abs(tc)<=eph.fitInterval/2
    and abs(E-eph.eccentricity*sin(E)-M)<1e-10;
end broadcastPosition;
