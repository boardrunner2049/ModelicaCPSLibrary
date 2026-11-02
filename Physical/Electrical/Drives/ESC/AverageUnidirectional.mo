within ModelicaCPSLibrary.Physical.Electrical.Drives.ESC;
model AverageUnidirectional "Ideal nonregenerative averaged drive with current decay and coast"
  input Real escCommand(unit="1");
  input Modelica.Units.SI.Voltage dcVoltage;
  input Modelica.Units.SI.Voltage openCircuitVoltage "Motor back-EMF used during blocked conduction";
  Modelica.Electrical.Analog.Interfaces.PositivePin p "Motor positive terminal";
  Modelica.Electrical.Analog.Interfaces.NegativePin n "Motor return; grounded here";
  output Real modulation(unit="1");
  output Modelica.Units.SI.Voltage terminalVoltage;
  output Modelica.Units.SI.Current motorCurrent;
  output Modelica.Units.SI.Current dcCurrent;
  output Modelica.Units.SI.Power electricalPower;
  output Boolean coast "Zero current and insufficient forward voltage";
  output Boolean freewheel "Zero commanded voltage with positive circulating current";
protected
  Modelica.Units.SI.Voltage requestedVoltage;
  Modelica.Electrical.Analog.Sources.SignalVoltage source;
  Modelica.Electrical.Analog.Basic.Ground ground;
equation
  assert(dcVoltage >= 0, "AverageUnidirectional requires nonnegative dcVoltage");
  modulation = noEvent(min(1,max(0,escCommand)));
  requestedVoltage = modulation*dcVoltage;
  motorCurrent = -p.i;
  connect(source.p,p);
  connect(source.n,n);
  connect(n,ground.p);
  coast = motorCurrent <= 0 and requestedVoltage <= openCircuitVoltage;
  terminalVoltage = if coast then openCircuitVoltage else requestedVoltage;
  source.v = terminalVoltage;
  dcCurrent = if coast or dcVoltage <= 0 then 0 else modulation*motorCurrent;
  electricalPower = dcVoltage*dcCurrent;
  freewheel = not coast and requestedVoltage <= 0;
  annotation(Documentation(info="<html><p>Ideal averaged one-quadrant DC-equivalent converter. During positive-current conduction, terminal voltage is modulation*dcVoltage. Zero command retains a zero-voltage circulating path until current reaches zero. At that zero-crossing only, current is projected to zero and terminal voltage follows the supplied back-EMF. This fixed-state realization of ideal blocking avoids changing the DAE index when opening an inductor. There is no current reset on command cutoff. Connect coast to the motor currentZero input so that the motor projects its actual winding-current state at the zero-crossing. This model requires an inductive load and its open-circuit voltage. No PWM ripple, phase commutation, switching losses, current limit or battery model. PWM-cycle discontinuous conduction is not resolved.</p></html>"));
end AverageUnidirectional;
