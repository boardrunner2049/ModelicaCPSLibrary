within ModelicaCPSLibrary.Systems.Peripherals;
model PulseRpm "R2: per shaft, phase-continuous pulses (Sensing.Rpm.PulseSensor): exact edge counts and buffered edge times, rpm from the latest two edges"
  parameter Integer n(min=1) = 1 "[configuration]";
  parameter Real pulsesPerRevolution = 1 "[datasheet]";
  parameter Real dutyCycle = 0.5 "[datasheet]";
  parameter Integer bufferSize(min=2) = 32 "Edge times kept per shaft [configuration]";
  parameter Real edgeDelay(unit="s") = 0 "[datasheet]";
  parameter Real edgeJitterStd(unit="s") = 0 "[datasheet]";
  parameter Real timeout(unit="s") = 1 "[configuration]";
  parameter Integer seed = 1 "[configuration]";
  parameter Integer streamId = 342 "[configuration]";
  input Real speed[n](each unit="rad/s");
  output Real rpm[n];
  output Real pulseCount[n] "Rising edges since the start (exact integers)";
  output Real lastEdgeTime[n](each unit="s");
  output Real edgeTimes[n*bufferSize](each unit="s") "Shaft i's latest edges at (i-1)*bufferSize+1.., newest first; -1 where none";
  output Boolean valid = true;
  output Real validity = 1 "1 if valid, else 0";
protected
  Sensing.Rpm.PulseSensor sensors[n](each pulsesPerRevolution=pulsesPerRevolution, each dutyCycle=dutyCycle,
    each bufferSize=bufferSize, each edgeDelay=edgeDelay, each edgeJitterStd=edgeJitterStd, each timeout=timeout,
    each seed=seed, streamId={streamId + i for i in 1:n});
equation
  for i in 1:n loop
    sensors[i].speed = speed[i];
    rpm[i] = sensors[i].rpm;
    pulseCount[i] = sensors[i].count;
    lastEdgeTime[i] = sensors[i].lastEdgeTime;
    for j in 1:bufferSize loop
      edgeTimes[(i - 1)*bufferSize + j] = sensors[i].edgeTimes[j];
    end for;
  end for;
end PulseRpm;
