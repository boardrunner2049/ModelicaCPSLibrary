within ModelicaCPSLibrary.Physical.Mechanical.Aerodynamics.Airframe;

model StabilityDerivatives "Quasi-steady airframe force and moment from stability derivatives, with post-stall flat-plate lift"
  parameter Coefficients coefficients;
  parameter Real airDensity(unit = "kg/m3") = 1.225;
  parameter Real referenceSpeed(min = 1e-6, unit = "m/s") = 0.5
    "Regularises alpha, beta and the rate normalisation near zero airspeed";
  input Real vAir_b[3](each unit = "m/s") "Air-relative velocity of the reference point, body FRD";
  input Real omega_b[3](each unit = "rad/s");
  input Real aileron(unit = "rad");
  input Real elevator(unit = "rad");
  input Real rudder(unit = "rad");
  output Real force_b[3](each unit = "N");
  output Real moment_b[3](each unit = "N.m") "About the aerodynamic reference point";
  output Real alpha(unit = "rad") "Angle of attack";
  output Real beta(unit = "rad") "Sideslip angle";
  output Real dynamicPressure(unit = "Pa");
protected
  final parameter Real aspectRatio = coefficients.b ^ 2 / coefficients.S;
  Real speedScale(unit = "m/s");
  Real sigma "0 attached flow .. 1 flat plate";
  Real CLattached;
  Real CL;
  Real CD;
  Real CX;
  Real CY;
  Real CZ;
  Real qS(unit = "N");
equation
  dynamicPressure = 0.5 * airDensity * (vAir_b * vAir_b);
  qS = dynamicPressure * coefficients.S;
  speedScale = sqrt(vAir_b * vAir_b + referenceSpeed ^ 2);
  // Second arguments are strictly positive, so both angles and their
  // derivatives exist at rest (the vehicle starts on the ground).
  alpha = atan2(vAir_b[3], sqrt(vAir_b[1] ^ 2 + referenceSpeed ^ 2));
  beta = atan2(vAir_b[2], sqrt(vAir_b[1] ^ 2 + vAir_b[3] ^ 2 + referenceSpeed ^ 2));
  // Beard and McLain (2012), Small Unmanned Aircraft, eq. 4.9-4.10.
  sigma = (1 + exp(-coefficients.stallBlend * (alpha - coefficients.alphaStall))
      + exp(coefficients.stallBlend * (alpha + coefficients.alphaStall)))
    / ((1 + exp(-coefficients.stallBlend * (alpha - coefficients.alphaStall)))
      * (1 + exp(coefficients.stallBlend * (alpha + coefficients.alphaStall))));
  CLattached = coefficients.CL0 + coefficients.CLalpha * alpha;
  CL = (1 - sigma) * CLattached + sigma * 2 * sin(alpha) * noEvent(abs(sin(alpha))) * cos(alpha)
    + coefficients.CLq * coefficients.c * omega_b[2] / (2 * speedScale)
    + coefficients.CLelevator * elevator;
  CD = coefficients.CDp + CLattached ^ 2 / (Modelica.Constants.pi * coefficients.oswald * aspectRatio)
    + coefficients.CDq * coefficients.c * omega_b[2] / (2 * speedScale)
    + coefficients.CDelevator * noEvent(abs(elevator));
  CX = -CD * cos(alpha) + CL * sin(alpha);
  CZ = -CD * sin(alpha) - CL * cos(alpha);
  CY = coefficients.CY0 + coefficients.CYbeta * beta
    + coefficients.b / (2 * speedScale) * (coefficients.CYp * omega_b[1] + coefficients.CYr * omega_b[3])
    + coefficients.CYaileron * aileron + coefficients.CYrudder * rudder;
  force_b = qS * {CX, CY, CZ};
  moment_b = qS * {
    coefficients.b * (coefficients.Cl0 + coefficients.Clbeta * beta
      + coefficients.b / (2 * speedScale) * (coefficients.Clp * omega_b[1] + coefficients.Clr * omega_b[3])
      + coefficients.Claileron * aileron + coefficients.Clrudder * rudder),
    coefficients.c * (coefficients.Cm0 + coefficients.Cmalpha * alpha
      + coefficients.c / (2 * speedScale) * coefficients.Cmq * omega_b[2]
      + coefficients.Cmelevator * elevator),
    coefficients.b * (coefficients.Cn0 + coefficients.Cnbeta * beta
      + coefficients.b / (2 * speedScale) * (coefficients.Cnp * omega_b[1] + coefficients.Cnr * omega_b[3])
      + coefficients.Cnaileron * aileron + coefficients.Cnrudder * rudder)};
  annotation(Documentation(info = "<html><p>Lift blends from the attached-flow line to flat-plate lift
through a sigmoid at the stall angle; drag is parasitic plus induced drag of the attached-flow lift.
Body-axis coefficients follow from lift and drag in the stability frame. Coefficients are evaluated at
the true dynamic pressure; only the angle and rate normalisations use the regularised speed, so all
forces vanish smoothly at rest.</p></html>"));
end StabilityDerivatives;
