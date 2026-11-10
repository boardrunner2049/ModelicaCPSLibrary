within ModelicaCPSLibrary.Systems.Sensing.Rpm;
model PulseSensor "E1: pulses from a phase-continuous shaft: exact edge count, logic level, and the latest edge times buffered for consumers that step coarser than the pulses"
  parameter Real pulsesPerRevolution(min=1) = 1 "Magnets, slots or marks per revolution [datasheet]";
  parameter Real dutyCycle(min=0, max=1) = 0.5 "Fraction of each pulse period the output is high [datasheet]";
  parameter Real phaseOffset = 0 "Initial pulse phase, in pulse periods, in [0, 1) [configuration]";
  parameter Integer bufferSize(min=2) = 32 "Edge times kept; structural, fixed when compiled [configuration]";
  parameter Real edgeDelay(unit="s") = 0 "Fixed delay of the reported edge time after the true edge [datasheet]";
  parameter Real edgeJitterStd(unit="s") = 0 "White timing jitter of each reported edge [datasheet]";
  parameter Real timeout(unit="s") = 1 "No edge for this long: rpm is reported 0 [configuration]";
  parameter Integer seed = 1 "[configuration]";
  parameter Integer streamId = 501 "[configuration]";
  input Real speed(unit="rad/s") "Shaft speed; the pulses count its magnitude (a Hall pickup has no direction)";
  discrete output Integer count(start=0, fixed=true) "Rising edges since the start, exact";
  output Boolean level "Logic output: high for dutyCycle of each pulse period, rising at each edge";
  discrete output Real lastEdgeTime(unit="s", start=-1, fixed=true) "Reported time of the latest rising edge; -1 before the first";
  discrete output Real edgeTimes[bufferSize](each unit="s", each start=-1, each fixed=true)
    "Reported times of the latest rising edges, newest first; -1 where none yet";
  output Real rpm(unit="1/min") "Speed from the latest two edges (as an autopilot measures it); 0 before two or after timeout";
  Real cycles(start=phaseOffset, fixed=true) "Pulse phase: rising edge k at cycles = k";
protected
  discrete Integer rngState[Modelica.Math.Random.Generators.Xorshift128plus.nState](each start=0, each fixed=true);
  discrete Integer rngSeed[Modelica.Math.Random.Generators.Xorshift128plus.nState](each start=0, each fixed=true);
  discrete Real normal[1](each start=0, each fixed=true);
  discrete Real previousEdgeTime(start=-1, fixed=true);
initial equation
  assert(phaseOffset >= 0 and phaseOffset < 1 and edgeJitterStd >= 0 and timeout > 0, "Invalid pulse sensor parameters");
equation
  der(cycles) = abs(speed)*pulsesPerRevolution/(2*Modelica.Constants.pi);
  when cycles >= pre(count) + 1 then
    count = pre(count) + 1;
    rngSeed = ModelicaCPSLibrary.Systems.Sensing.seededState(pre(rngState), pre(count) > 0, streamId, seed);
    (normal, rngState) = ModelicaCPSLibrary.Systems.Sensing.normalDraw(1, rngSeed);
    previousEdgeTime = pre(lastEdgeTime);
    lastEdgeTime = time + edgeDelay + edgeJitterStd*normal[1];
    // A shift by concatenation: Rumoca rejects the equivalent for-loop in a
    // state-event clause.
    edgeTimes = cat(1, {lastEdgeTime}, pre(edgeTimes[1:bufferSize - 1]));
  end when;
  level = cycles - count < dutyCycle;
  rpm = if count >= 2 and time - lastEdgeTime < timeout and lastEdgeTime > previousEdgeTime
    then 60/(pulsesPerRevolution*(lastEdgeTime - previousEdgeTime)) else 0;
  annotation(Documentation(info="<html><p>Edges are state events located in time. A consumer that
reads every <code>h</code> seconds recovers every edge since its last read: the new edges are
<code>count - previousCount</code>, their times the first that many of <code>edgeTimes</code>. If more
than <code>bufferSize</code> edges fall in one step the count stays exact but the older times are lost;
the consumer detects it from the count. Coincident edges cannot occur (one per pulse period).
Reported times include <code>edgeDelay</code> and jitter; the count and the level use the true
edges. Simulation start (or an FMU reset) restarts the phase at <code>phaseOffset</code>.</p></html>"));
end PulseSensor;
