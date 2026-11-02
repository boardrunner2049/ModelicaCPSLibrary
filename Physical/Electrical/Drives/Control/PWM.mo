within ModelicaCPSLibrary.Physical.Electrical.Drives.Control;
block PWM "Leading-edge high-side PWM, turn-on delay and immediate turn-off"
  parameter Modelica.Units.SI.Frequency frequency;
  parameter Modelica.Units.SI.Time deadTime = 0;
  input Real modulation;
  input Boolean highSelected[3];
  input Boolean lowSelected[3](each start=false,each fixed=true);
  output Boolean highGate[3];
  output Boolean lowGate[3];
protected
  discrete Modelica.Units.SI.Time periodStart(start=0,fixed=true);
  discrete Modelica.Units.SI.Time highReady[3](each start=0,each fixed=true);
  discrete Modelica.Units.SI.Time lowReady[3](each start=0,each fixed=true);
  Boolean highRequest[3](each start=false,each fixed=true);
initial equation
  assert(frequency > 0 and deadTime >= 0 and deadTime*frequency < 0.5,
    "PWM requires frequency > 0 and 0 <= deadTime < half a period");
equation
  when sample(0,1/frequency) then
    periodStart = time;
  end when;
  for j in 1:3 loop
    highRequest[j] = highSelected[j] and time-periodStart < modulation/frequency;
    when {initial(),change(highRequest[j])} then
      highReady[j] = time+deadTime;
    end when;
    when {initial(),change(lowSelected[j])} then
      lowReady[j] = time+deadTime;
    end when;
    highGate[j] = highRequest[j] and time >= highReady[j];
    lowGate[j] = lowSelected[j] and time >= lowReady[j];
  end for;
end PWM;
