within ModelicaCPSLibrary.Physical.Electrical.Drives.Control;
block SixStep "Ideal-position forward six-step commutation with latched recirculation then Hi-Z"
  parameter Modelica.Units.SI.Current currentThreshold = 1e-3 "Switch release threshold, not a state reset";
  input Real command;
  input Modelica.Units.SI.Angle electricalAngle;
  input Modelica.Units.SI.Current phaseCurrent[3];
  output Real modulation;
  output Boolean highSelected[3];
  output Boolean lowSelected[3];
  output Boolean coast;
  output Boolean recirculating(start=false,fixed=true);
  output Integer sector;
protected
  constant Integer highTable[6] = {3,1,1,2,2,3};
  constant Integer lowTable[6] = {2,2,3,3,1,1};
  Integer lowPhase;
  discrete Integer stopLow(start=2,fixed=true);
  Boolean enabled(start=false,fixed=true);
  Boolean drained;
initial equation
  assert(currentThreshold > 0,"SixStep requires positive currentThreshold");
equation
  modulation = noEvent(min(1,max(0,command)));
  // Evaluate exact zero at command events; do not retain an on-state through relation hysteresis.
  enabled = noEvent(command > 0);
  sector = 1+integer(floor(mod(electricalAngle+Modelica.Constants.pi/6,
    2*Modelica.Constants.pi)/(Modelica.Constants.pi/3)));
  lowPhase = lowTable[sector];
  // Scalar ABC relations also avoid compiler ambiguity for array reductions in event roots.
  drained = max(abs(phaseCurrent[1]),max(abs(phaseCurrent[2]),abs(phaseCurrent[3]))) <= currentThreshold;
  when {initial(),change(enabled),drained} then
    if initial() or enabled then
      recirculating = false;
      stopLow = lowPhase;
    elseif not enabled and pre(enabled) then
      recirculating = not drained;
      stopLow = pre(lowPhase);
    else
      recirculating = false;
      stopLow = pre(stopLow);
    end if;
  end when;
  for j in 1:3 loop
    highSelected[j] = enabled and j == highTable[sector];
    lowSelected[j] = if enabled then j == lowPhase else recirculating and j == stopLow;
  end for;
  coast = not enabled and not recirculating;
  annotation(Documentation(info="<html><p>Rotor position is known exactly. No FOC, speed/current loop or sensorless startup. During PWM freewheel the selected low-side remains on. At command cutoff that sink leg is held until all phase currents fall below currentThreshold, then all gates are off. Diodes remain connected, including possible regeneration. Initial zero-command operation begins in Hi-Z even with nonzero initial winding current.</p></html>"));
end SixStep;
