within ModelicaCPSLibrary.Systems.Sensing.GNSS;
function broadcastStates "Evaluate an array of broadcast records at one reception epoch"
  input BroadcastEphemeris ephemeris[:];
  input Real epoch;
  output Real position[size(ephemeris,1),3];
  output Real velocity[size(ephemeris,1),3];
  output Real clockBias[size(ephemeris,1)];
  output Real clockDrift[size(ephemeris,1)];
  output Boolean available[size(ephemeris,1)];
protected
  Real p[3];
  Real v[3];
  Real b;
  Real d;
  Boolean valid;
algorithm
  for i in 1:size(ephemeris,1) loop
    (p,v,b,d,valid):=broadcastState(ephemeris[i],epoch);
    position[i,:]:=p;
    velocity[i,:]:=v;
    clockBias[i]:=b;
    clockDrift[i]:=d;
    available[i]:=valid;
  end for;
end broadcastStates;
