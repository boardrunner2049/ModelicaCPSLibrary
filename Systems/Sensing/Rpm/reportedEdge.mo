within ModelicaCPSLibrary.Systems.Sensing.Rpm;
function reportedEdge "Reported time of a rising edge: the true time plus the fixed delay and one seeded jitter draw"
  input Real trueTime(unit="s");
  input Real delay(unit="s");
  input Real jitterStd(unit="s");
  input Integer stateIn[4] "Xorshift128plus state; ignored until seeded";
  input Boolean seeded "false at the first edge";
  input Integer streamId;
  input Integer seed;
  output Real reported(unit="s");
  output Integer stateOut[4];
protected
  Real z[1];
algorithm
  (z, stateOut) := ModelicaCPSLibrary.Systems.Sensing.seededNormalDraw(1, stateIn, seeded, streamId, seed);
  reported := trueTime + delay + jitterStd*z[1];
  annotation(Documentation(info="<html><p>The draw and its use are one equation. With the draw as
its own equation in the state-event clause, Rumoca 0.10.1 used the previous edge's draw (and 0 at
the first edge) while OpenModelica used the current one; the cross-check
(<code>tools/calibration</code>, <code>crosscheck</code>) found it.</p></html>"));
end reportedEdge;
