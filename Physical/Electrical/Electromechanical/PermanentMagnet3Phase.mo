within ModelicaCPSLibrary.Physical.Electrical.Electromechanical;
model PermanentMagnet3Phase "Lossless non-salient PM conversion, with explicit ABC terminals"
  parameter Integer polePairs(min=1);
  parameter Modelica.Units.SI.ElectricalTorqueConstant ke "Phase peak back-EMF per mechanical rad/s";
  parameter Modelica.Units.SI.Angle theta0 = 0 "Electrical offset at zero mechanical angle";
  parameter Boolean trapezoidal = true "False selects sinusoidal back-EMF";
  Modelica.Electrical.Analog.Interfaces.PositivePin p[3];
  Modelica.Electrical.Analog.Interfaces.NegativePin neutral;
  Modelica.Mechanics.Rotational.Interfaces.Flange_b flange;
  Modelica.Mechanics.Rotational.Interfaces.Support support;
  output Modelica.Units.SI.Angle electricalAngle;
  output Modelica.Units.SI.AngularVelocity omega;
  output Modelica.Units.SI.Voltage backEMF[3];
  output Modelica.Units.SI.Torque torque;
  Real shape[3];
initial equation
  assert(polePairs >= 1 and ke > 0,"PermanentMagnet3Phase requires positive polePairs and ke");
equation
  electricalAngle = polePairs*(flange.phi-support.phi)+theta0;
  omega = der(flange.phi-support.phi);
  for j in 1:3 loop
    shape[j] = if trapezoidal then trapezoid(electricalAngle-(j-1)*2*Modelica.Constants.pi/3)
      else sin(electricalAngle-(j-1)*2*Modelica.Constants.pi/3);
    backEMF[j] = ke*omega*shape[j];
    p[j].v-neutral.v = backEMF[j];
  end for;
  sum(p.i)+neutral.i = 0;
  torque = ke*sum(shape .* p.i);
  flange.tau = -torque;
  support.tau = torque;
  annotation(Documentation(info="<html><p>e_j=ke*omega*f_j(theta_e); tau=ke*sum(f_j*i_j). Thus sum(e_j*i_j)=tau*omega, including at zero speed without dividing by omega. No winding resistance, inductance or inertia is included here. ke is per-phase peak per mechanical speed, not a line-line RMS or DC-equivalent constant. No saliency, saturation or reluctance torque.</p></html>"));
end PermanentMagnet3Phase;
