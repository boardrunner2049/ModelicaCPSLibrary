within ModelicaCPSLibrary.Systems.Sensing.GNSS;
function broadcastState "Broadcast position and centered numerical ECEF velocity/clock drift"
  input BroadcastEphemeris eph;
  input Real epoch(unit="s");
  output Real position[3];
  output Real velocity[3];
  output Real clockBias;
  output Real clockDrift;
  output Boolean valid;
protected
  Real before[3];
  Real after[3];
  Real clockBefore;
  Real clockAfter;
  Boolean unusedBefore;
  Boolean unusedAfter;
algorithm
  (position,clockBias,valid):=broadcastPosition(eph,epoch);
  (before,clockBefore,unusedBefore):=broadcastPosition(eph,epoch-0.05);
  (after,clockAfter,unusedAfter):=broadcastPosition(eph,epoch+0.05);
  velocity:=(after-before)/0.1;
  clockDrift:=(clockAfter-clockBefore)/0.1;
end broadcastState;
