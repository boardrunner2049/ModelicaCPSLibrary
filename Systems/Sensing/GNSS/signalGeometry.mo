within ModelicaCPSLibrary.Systems.Sensing.GNSS;
function signalGeometry "Linear satellite flight-time propagation and Earth rotation, with consistent derivatives"
  input Real receiver[3] "ECEF at reception";
  input Real satellite[3] "ECEF at reception";
  input Real satelliteVelocity[3] "ECEF derivative at reception";
  output Real range;
  output Real gradient[3] "d(range)/d(receiver), including flight-time dependence";
  output Real satelliteRate "Range rate for a stationary ECEF receiver";
  output Real lineOfSight[3];
protected
  constant Real c=299792458;
  constant Real omega=7.2921151467e-5;
  Real tau;
  Real rotation[3,3];
  Real transmitted[3];
  Real rotatedVelocity[3];
  Real k[3];
  Real divisor;
algorithm
  tau:=sqrt((satellite-receiver)*(satellite-receiver))/c;
  for j in 1:6 loop
    rotation:=[cos(omega*tau),sin(omega*tau),0;-sin(omega*tau),cos(omega*tau),0;0,0,1];
    transmitted:=rotation*(satellite-satelliteVelocity*tau);
    range:=sqrt((transmitted-receiver)*(transmitted-receiver));
    tau:=range/c;
  end for;
  lineOfSight:=(transmitted-receiver)/max(range,1e-6);
  rotatedVelocity:=rotation*satelliteVelocity;
  k:=-cross({0,0,omega},transmitted)-rotatedVelocity;
  divisor:=1-lineOfSight*k/c;
  gradient:=-lineOfSight/divisor;
  satelliteRate:=lineOfSight*rotatedVelocity/divisor;
end signalGeometry;
