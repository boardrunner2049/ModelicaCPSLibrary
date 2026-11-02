within ModelicaCPSLibrary.Physical.Electrical.Drives.ESC;
model AverageThreePhase "P3 six-step asynchronous PWM mean model"
  extends PartialThreePhase;
  Drives.Bridges.AverageHalfBridge legs[3](each Ron=Ron,each Rdiode=Rdiode,
    each Vdiode=Vdiode,each Goff=Goff);
equation
  for j in 1:3 loop
    connect(dc_p,legs[j].dc_p);
    connect(dc_n,legs[j].dc_n);
    connect(phase[j],legs[j].phase);
    legs[j].modulation = control.modulation;
    legs[j].highSelected = control.highSelected[j];
    legs[j].lowGate = control.lowSelected[j];
  end for;
end AverageThreePhase;
