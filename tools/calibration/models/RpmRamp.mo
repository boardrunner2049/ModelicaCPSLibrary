model RpmRamp "PulseSensor on a shaft accelerating at a constant rate, speed = omega0 + alpha*time, inside the model so the edges do not depend on the consumer's step"
  parameter Real omega0(unit="rad/s") = 314.159;
  parameter Real alpha(unit="rad/s2") = 0;
  parameter Real pulsesPerRevolution = 12;
  parameter Real phaseOffset = 0;
  parameter Real edgeDelay(unit="s") = 0;
  parameter Real edgeJitterStd(unit="s") = 0;
  parameter Integer seed = 1;
  output Integer count;
  output Real lastEdgeTime(unit="s");
  output Real edgeTimes[32](each unit="s");
  output Real rpm;
  output Boolean level;
  ModelicaCPSLibrary.Systems.Sensing.Rpm.PulseSensor sensor(bufferSize=32, pulsesPerRevolution=pulsesPerRevolution,
    phaseOffset=phaseOffset, edgeDelay=edgeDelay, edgeJitterStd=edgeJitterStd, seed=seed);
equation
  sensor.speed = omega0 + alpha*time;
  count = sensor.count;
  lastEdgeTime = sensor.lastEdgeTime;
  edgeTimes = sensor.edgeTimes;
  rpm = sensor.rpm;
  level = sensor.level;
end RpmRamp;
