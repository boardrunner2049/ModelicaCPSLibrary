within ModelicaCPSLibrary.Physical.Mechanical.Aerodynamics.Airframe;

record Coefficients "Reference geometry and nondimensional stability and control derivatives"
  // Defaults: the Skywalker (2013) airframe of the last_letter simulator
  // (G. Zogopoulos-Papaliakos, skywalker_2013/aerodynamics.yaml), also the
  // ArduPilot SITL plane. Control derivatives are per radian of deflection.
  parameter Real S(unit = "m2") = 0.45 "Wing reference area";
  parameter Real b(unit = "m") = 1.88 "Span";
  parameter Real c(unit = "m") = 0.24 "Mean aerodynamic chord";
  parameter Real CL0 = 0.56;
  parameter Real CLalpha(unit = "1/rad") = 6.9;
  parameter Real CLq = 0;
  parameter Real CLelevator(unit = "1/rad") = 0;
  parameter Real stallBlend = 50 "Sigmoid sharpness of the stall transition (M)";
  parameter Real alphaStall(unit = "rad") = 0.4712;
  parameter Real oswald = 0.9 "Oswald efficiency of the induced drag";
  parameter Real CDp = 0.1 "Parasitic drag";
  parameter Real CDq = 0;
  parameter Real CDelevator(unit = "1/rad") = 0 "Drag per radian of |elevator|";
  parameter Real CY0 = 0;
  parameter Real CYbeta(unit = "1/rad") = -0.98;
  parameter Real CYp = 0;
  parameter Real CYr = 0;
  parameter Real CYaileron(unit = "1/rad") = 0;
  parameter Real CYrudder(unit = "1/rad") = -0.2;
  parameter Real Cl0 = 0;
  parameter Real Clbeta(unit = "1/rad") = -0.12;
  parameter Real Clp = -1.0;
  parameter Real Clr = 0.14;
  parameter Real Claileron(unit = "1/rad") = 0.25;
  parameter Real Clrudder(unit = "1/rad") = -0.037;
  parameter Real Cm0 = 0.045;
  parameter Real Cmalpha(unit = "1/rad") = -0.7;
  parameter Real Cmq = -20;
  parameter Real Cmelevator(unit = "1/rad") = 1.0;
  parameter Real Cn0 = 0;
  parameter Real Cnbeta(unit = "1/rad") = 0.25;
  parameter Real Cnp = 0.022;
  parameter Real Cnr = -1;
  parameter Real Cnaileron(unit = "1/rad") = 0;
  parameter Real Cnrudder(unit = "1/rad") = 0.1;
  annotation(Documentation(info = "<html><p>Sign convention of the control deflections: a positive
aileron rolls right, a positive elevator pitches nose up, a positive rudder yaws nose right (each
through its moment derivative, which is then positive). Rate derivatives are per unit of the
normalised rates p b/2V, q c/2V, r b/2V.</p></html>"));
end Coefficients;
