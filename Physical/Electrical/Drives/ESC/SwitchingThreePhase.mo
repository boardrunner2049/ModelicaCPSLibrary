within ModelicaCPSLibrary.Physical.Electrical.Drives.ESC;
model SwitchingThreePhase "P4 six-switch inverter with high-side PWM and diode freewheel"
  extends PartialThreePhase;
  parameter Modelica.Units.SI.Frequency pwmFrequency;
  parameter Modelica.Units.SI.Time deadTime = 0;
  Physical.Electrical.Drives.Control.PWM pwm(frequency=pwmFrequency,deadTime=deadTime);
  Drives.Bridges.SwitchingHalfBridge legs[3](each Ron=Ron,each Rdiode=Rdiode,
    each Vdiode=Vdiode,each Goff=Goff);
equation
  pwm.modulation = control.modulation;
  pwm.highSelected = control.highSelected;
  pwm.lowSelected = control.lowSelected;
  for j in 1:3 loop
    connect(dc_p,legs[j].dc_p);
    connect(dc_n,legs[j].dc_n);
    connect(phase[j],legs[j].phase);
    legs[j].highGate = pwm.highGate[j];
    legs[j].lowGate = pwm.lowGate[j];
  end for;
end SwitchingThreePhase;
