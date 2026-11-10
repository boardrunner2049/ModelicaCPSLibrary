within ModelicaCPSLibrary.Systems.Power;
model Battery "Lithium pack: open-circuit voltage of the state of charge (cell curve), series resistance, one RC polarization branch, coulomb counting"
  parameter Integer cells(min=1) = 4 "Cells in series";
  parameter Real capacity(unit="A.h") = 5.2 "Rated capacity";
  parameter Real resistance(unit="Ohm") = 0.02 "Pack series (DC) resistance";
  parameter Real polarizationResistance(unit="Ohm") = 0 "RC branch resistance; 0 for none";
  parameter Real polarizationTau(unit="s") = 30 "RC branch time constant";
  parameter Real soc_start = 1 "State of charge at the start, 0..1";
  parameter Real socPoints[:] = {0, 0.05, 0.1, 0.2, 0.5, 0.8, 0.95, 1}
    "State-of-charge breakpoints of the cell curve";
  parameter Real cellVoltagePoints[size(socPoints, 1)](each unit="V") =
    {3.27, 3.61, 3.69, 3.73, 3.82, 3.99, 4.12, 4.2}
    "Open-circuit cell voltage at the breakpoints (generic LiPo curve)";
  parameter Real currentTau(unit="s") = 0.01
    "Lag of the current the resistances see; separates the pack from loads that depend on its voltage";
  input Real current(unit="A") "Discharge current";
  output Real voltage(unit="V") "Terminal voltage";
  output Real openCircuitVoltage(unit="V");
  output Real soc "State of charge, 0..1";
  output Real consumed(unit="A.h") "Charge drawn since the start";
protected
  Real charge(unit="A.h", start=soc_start*capacity, fixed=true);
  Real lagged(unit="A", start=0, fixed=true);
  Real polarization(unit="V", start=0, fixed=true);
initial equation
  assert(capacity > 0 and resistance >= 0 and polarizationResistance >= 0 and polarizationTau > 0
    and currentTau > 0 and soc_start >= 0 and soc_start <= 1, "Invalid battery parameters");
equation
  der(charge) = -current/3600;
  der(lagged) = (current - lagged)/currentTau;
  der(polarization) = (polarizationResistance*lagged - polarization)/polarizationTau;
  soc = charge/capacity;
  consumed = soc_start*capacity - charge;
  openCircuitVoltage = cells*interpolate(min(1, max(0, soc)), socPoints, cellVoltagePoints);
  voltage = openCircuitVoltage - resistance*lagged - polarization;
end Battery;
