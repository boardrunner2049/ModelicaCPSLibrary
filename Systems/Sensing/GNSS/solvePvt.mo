within ModelicaCPSLibrary.Systems.Sensing.GNSS;
function solvePvt "Weighted range/range-rate solution; no receiver truth used as an initial guess"
  input Real satellite[:,3];
  input Real satelliteVelocity[size(satellite,1),3];
  input Real satelliteClock[size(satellite,1)];
  input Real satelliteClockDrift[size(satellite,1)];
  input Real pseudorange[size(satellite,1)];
  input Real rangeRate[size(satellite,1)];
  input Boolean used[size(satellite,1)];
  input Real rangeSigma[size(satellite,1)];
  input Real rateSigma[size(satellite,1)];
  input Real initialPosition[3];
  input Real origin[3];
  input Real R_ew[3,3];
  input Integer maxIterations=12;
  input Real convergenceTolerance=1e-4;
  input Real rankTolerance=1e-10;
  output Real pvt[6];
  output Real covariance[6,6];
  output Real clockBias;
  output Real clockDrift;
  output Real pdop;
  output Boolean valid;
protected
  constant Real c=299792458;
  Integer count;
  Real position[3];
  Real clockDistance;
  Real range;
  Real gradient[3];
  Real satelliteRate;
  Real los[3];
  Real H[size(satellite,1),4];
  Real residual[size(satellite,1)];
  Real normal[4,4];
  Real velocityNormal[4,4];
  Real geometryNormal[4,4];
  Real inverse[4,4];
  Real velocityInverse[4,4];
  Real geometryInverse[4,4];
  Real rhs[4];
  Real velocityRhs[4];
  Real update[4];
  Real solution[4];
  Boolean invertible;
  Boolean converged;
algorithm
  pvt:=zeros(6);
  covariance:=zeros(6,6);
  clockBias:=0;
  clockDrift:=0;
  pdop:=-1;
  valid:=false;
  count:=sum({if used[i] then 1 else 0 for i in 1:size(satellite,1)});
  if count<4 then
    return;
  end if;
  position:=initialPosition;
  clockDistance:=0;
  converged:=false;
  for iteration in 1:maxIterations loop
    normal:=zeros(4,4);
    rhs:=zeros(4);
    for i in 1:size(satellite,1) loop
      H[i,:]:=zeros(4);
      residual[i]:=0;
      if used[i] then
        (range,gradient,satelliteRate,los):=signalGeometry(position,satellite[i,:],satelliteVelocity[i,:]);
        H[i,:]:=cat(1,(1+satelliteClockDrift[i])*gradient,{1});
        residual[i]:=pseudorange[i]+c*satelliteClock[i]-(1+satelliteClockDrift[i])*range-clockDistance;
        normal:=normal+outerProduct(H[i,:],H[i,:])/rangeSigma[i]^2;
        rhs:=rhs+H[i,:]*residual[i]/rangeSigma[i]^2;
      end if;
    end for;
    (inverse,invertible):=inverseNormal(normal,rankTolerance);
    if not invertible then
      return;
    end if;
    update:=inverse*rhs;
    position:=position+update[1:3];
    clockDistance:=clockDistance+update[4];
    if max(abs(update))<convergenceTolerance then
      converged:=true;
      break;
    end if;
  end for;
  if not converged then
    return;
  end if;
  normal:=zeros(4,4);
  velocityNormal:=zeros(4,4);
  geometryNormal:=zeros(4,4);
  velocityRhs:=zeros(4);
  for i in 1:size(satellite,1) loop
    if used[i] then
      (range,gradient,satelliteRate,los):=signalGeometry(position,satellite[i,:],satelliteVelocity[i,:]);
      H[i,:]:=cat(1,(1+satelliteClockDrift[i])*gradient,{1});
      normal:=normal+outerProduct(H[i,:],H[i,:])/rangeSigma[i]^2;
      velocityNormal:=velocityNormal+outerProduct(H[i,:],H[i,:])/rateSigma[i]^2;
      geometryNormal:=geometryNormal+outerProduct(H[i,:],H[i,:]);
      velocityRhs:=velocityRhs+H[i,:]*(rangeRate[i]+c*satelliteClockDrift[i]
        -(1+satelliteClockDrift[i])*satelliteRate)/rateSigma[i]^2;
    end if;
  end for;
  (inverse,invertible):=inverseNormal(normal,rankTolerance);
  if not invertible then
    return;
  end if;
  (velocityInverse,invertible):=inverseNormal(velocityNormal,rankTolerance);
  if not invertible then
    return;
  end if;
  (geometryInverse,invertible):=inverseNormal(geometryNormal,rankTolerance);
  if not invertible then
    return;
  end if;
  solution:=velocityInverse*velocityRhs;
  pvt[1:3]:=transpose(R_ew)*(position-origin);
  pvt[4:6]:=transpose(R_ew)*solution[1:3];
  covariance[1:3,1:3]:=transpose(R_ew)*inverse[1:3,1:3]*R_ew;
  covariance[4:6,4:6]:=transpose(R_ew)*velocityInverse[1:3,1:3]*R_ew;
  clockBias:=clockDistance/c;
  clockDrift:=solution[4]/c;
  pdop:=sqrt(max(0,sum({geometryInverse[j,j] for j in 1:3})));
  valid:=true;
end solvePvt;
