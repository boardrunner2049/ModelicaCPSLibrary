within ModelicaCPSLibrary.Tests;
model GnssEphemeris "Analytic circular orbit, week wrap, expiry, WGS84 and signal derivative checks"
  parameter Systems.Sensing.GNSS.BroadcastEphemeris circular(eccentricity=0,inclination=0,omegaDot=0,
    af0=2e-5,af1=3e-10,tgd=1e-8);
  parameter Systems.Sensing.GNSS.BroadcastEphemeris stale(eccentricity=0,fitInterval=2);
  parameter Systems.Sensing.GNSS.BroadcastEphemeris unhealthy(healthy=false);
  Real position[3];
  Real velocity[3];
  Real clockBias;
  Real clockDrift;
  Boolean valid;
protected
  parameter Real origin[3](each fixed=false);
  parameter Real rotation[3,3](each fixed=false);
  parameter Real wrapped[3](each fixed=false);
  parameter Real ignoredClock(fixed=false);
  parameter Boolean ignoredValid(fixed=false);
  parameter Real range(fixed=false);
  parameter Real gradient[3](each fixed=false);
  parameter Real rate(fixed=false);
  parameter Real los[3](each fixed=false);
  parameter Real rangePlus(fixed=false);
  parameter Real rangeMinus(fixed=false);
  parameter Real unusedGradient[3](each fixed=false);
  parameter Real unusedRate(fixed=false);
  parameter Real unusedLos[3](each fixed=false);
  parameter Real receiver[3](each fixed=false);
  parameter Real satellite[3](each fixed=false);
  parameter Real satVelocity[3](each fixed=false);
initial algorithm
  (origin,rotation):=Systems.Sensing.GNSS.referenceFrame(Systems.Sensing.GNSS.Reference());
  assert(max(abs(origin-{6378137,0,0}))<1e-8 and max(abs(rotation-[0,0,-1;0,1,0;1,0,0]))<1e-12,"WGS84 equatorial NED frame incorrect");
  (origin,rotation):=Systems.Sensing.GNSS.referenceFrame(Systems.Sensing.GNSS.Reference(latitude=0.7,longitude=-1.2,altitude=300));
  assert(max(abs(transpose(rotation)*rotation-identity(3)))<1e-12 and abs(Modelica.Math.Matrices.det(rotation)-1)<1e-12,"NED-to-ECEF must be proper rotation");
  (wrapped,ignoredClock,ignoredValid):=Systems.Sensing.GNSS.broadcastPosition(circular,604800);
  assert(ignoredValid and max(abs(wrapped-{circular.sqrtA^2,0,0}))<1e-6,"GNSS week wrap failed");
  (wrapped,ignoredClock,ignoredValid):=Systems.Sensing.GNSS.broadcastPosition(stale,2);
  assert(not ignoredValid,"Expired ephemeris must not be available");
  (wrapped,ignoredClock,ignoredValid):=Systems.Sensing.GNSS.broadcastPosition(unhealthy,0);
  assert(not ignoredValid,"Unhealthy ephemeris must not be available");
  receiver:={6378137,100,200};
  satellite:={21000000,13000000,15000000};
  satVelocity:={-1200,2300,1500};
  (range,gradient,rate,los):=Systems.Sensing.GNSS.signalGeometry(receiver,satellite,satVelocity);
  for j in 1:3 loop
    (rangePlus,unusedGradient,unusedRate,unusedLos):=Systems.Sensing.GNSS.signalGeometry(
      receiver+{if i==j then 0.5 else 0 for i in 1:3},satellite,satVelocity);
    (rangeMinus,unusedGradient,unusedRate,unusedLos):=Systems.Sensing.GNSS.signalGeometry(
      receiver-{if i==j then 0.5 else 0 for i in 1:3},satellite,satVelocity);
    assert(abs(rangePlus-rangeMinus-gradient[j])<1e-7,"Flight-time Jacobian inconsistent with range");
  end for;
  (rangePlus,unusedGradient,unusedRate,unusedLos):=Systems.Sensing.GNSS.signalGeometry(receiver,satellite+0.01*satVelocity,satVelocity);
  (rangeMinus,unusedGradient,unusedRate,unusedLos):=Systems.Sensing.GNSS.signalGeometry(receiver,satellite-0.01*satVelocity,satVelocity);
  assert(abs((rangePlus-rangeMinus)/0.02-rate)<1e-5,"Range-rate inconsistent with time derivative");
  (range,gradient,rate,los):=Systems.Sensing.GNSS.signalGeometry(receiver,satellite,zeros(3));
  assert(abs(range-sqrt((satellite-receiver)*(satellite-receiver))-
    7.2921151467e-5/299792458*(satellite[1]*receiver[2]-satellite[2]*receiver[1]))<0.001,"Earth rotation sign wrong");
equation
  (position,velocity,clockBias,clockDrift,valid)=Systems.Sensing.GNSS.broadcastState(circular,time);
  when time>=0.01 then
    assert(valid and abs(sqrt(position*position)-circular.sqrtA^2)<1e-6,"Circular ephemeris radius changed");
    assert(abs(sqrt(velocity*velocity)-circular.sqrtA^2*abs(sqrt(3.986005e14/circular.sqrtA^6)-7.2921151467e-5))<1e-3,"Ephemeris velocity wrong");
    assert(abs(clockBias-(2e-5+3e-10*time-1e-8))<1e-14 and abs(clockDrift-3e-10)<1e-14,"Ephemeris clock wrong");
  end when;
  annotation(experiment(StopTime=0.02,Interval=0.001,Tolerance=1e-9));
end GnssEphemeris;
