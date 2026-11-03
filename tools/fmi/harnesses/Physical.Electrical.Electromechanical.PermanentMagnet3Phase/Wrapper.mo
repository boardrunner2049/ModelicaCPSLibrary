// Harness wrapper for ModelicaCPSLibrary.Physical.Electrical.Electromechanical.PermanentMagnet3Phase.
// The ideal PM converter has open acausal ports (p[3], neutral, flange,
// support) and parameters without values, so it cannot be exported alone.
// This wrapper runs two converters as generators on one shaft:
//   - trapMachine: trapezoidal back-EMF (the library default),
//   - sinMachine:  sinusoidal back-EMF (trapezoidal = false),
// both with polePairs = 7, ke = 0.01 V/(rad/s); the trapezoidal one with an
// electrical offset theta0 = 0.4 rad, the sinusoidal one with theta0 = 0
// (so the offset is exercised and the two angle bookkeepings differ). The shaft angle
// is imposed by an exact MSL speed source driven by the `speed` input, the
// supports are fixed, and each converter feeds a star of three 0.1 Ohm
// resistors whose star point is the converter's neutral.
// Reference terms are made observable: the trapezoidal converter's neutral is
// lifted off ground by a 3 V / 50 Hz sine source (so p[j].v - neutral.v is
// distinguishable from p[j].v, and the source current must vanish if the
// converter's KCL is right); the sinusoidal converter's support sits on its
// own housing turned by phi0 = 0.1 rad (so flange.phi - support.phi is
// distinguishable from flange.phi). Each housing reports its reaction torque.
// The resistors are an explicit OnePort (v = R*i) rather than MSL
// Basic.Resistor, whose heat-port assertion is refused by the Rumoca FMI C
// profile (see the harness report).
// Observers: *PowerResidual = mechanical power into the converter through its
// flange minus the power dissipated in its load resistors (zero for a
// lossless converter).
model FireHarness_PermanentMagnet3Phase
  model Load "Linear resistor, v = R*i"
    extends Modelica.Electrical.Analog.Interfaces.OnePort;
    parameter Modelica.Units.SI.Resistance R;
  equation
    v = R*i;
  end Load;

  input Real speed(unit = "rad/s") "Imposed mechanical shaft speed";
  ModelicaCPSLibrary.Physical.Electrical.Electromechanical.PermanentMagnet3Phase trapMachine(
    polePairs = 7, ke = 0.01, theta0 = 0.4, trapezoidal = true);
  ModelicaCPSLibrary.Physical.Electrical.Electromechanical.PermanentMagnet3Phase sinMachine(
    polePairs = 7, ke = 0.01, theta0 = 0, trapezoidal = false);
  Modelica.Mechanics.Rotational.Sources.Speed drive(exact = true);
  Modelica.Mechanics.Rotational.Components.Fixed trapHousing;
  Modelica.Mechanics.Rotational.Components.Fixed sinHousing(phi0 = 0.1);
  Modelica.Electrical.Analog.Sources.SignalVoltage neutralLift "3 V, 50 Hz sine";
  Load trapLoad[3](each R = 0.1);
  Load sinLoad[3](each R = 0.1);
  Modelica.Electrical.Analog.Basic.Ground trapGround;
  Modelica.Electrical.Analog.Basic.Ground sinGround;
  output Modelica.Units.SI.Torque trapTorque = trapMachine.torque;
  output Modelica.Units.SI.Torque sinTorque = sinMachine.torque;
  output Modelica.Units.SI.Angle electricalAngle = sinMachine.electricalAngle;
  output Modelica.Units.SI.Power trapPowerResidual = trapMachine.flange.tau*drive.w - sum(trapLoad.v .* trapLoad.i);
  output Modelica.Units.SI.Current neutralLiftCurrent = neutralLift.i;
  output Modelica.Units.SI.Voltage trapNeutralVoltage = trapMachine.neutral.v;
  output Modelica.Units.SI.Torque trapHousingTorque = trapHousing.flange.tau;
  output Modelica.Units.SI.Torque sinHousingTorque = sinHousing.flange.tau;
  output Modelica.Units.SI.Power sinPowerResidual = sinMachine.flange.tau*drive.w - sum(sinLoad.v .* sinLoad.i);
equation
  drive.w_ref = speed;
  neutralLift.v = 3*sin(2*Modelica.Constants.pi*50*time);
  connect(drive.flange, trapMachine.flange);
  connect(drive.flange, sinMachine.flange);
  connect(trapMachine.support, trapHousing.flange);
  connect(sinMachine.support, sinHousing.flange);
  for j in 1:3 loop
    connect(trapMachine.p[j], trapLoad[j].p);
    connect(trapLoad[j].n, trapMachine.neutral);
    connect(sinMachine.p[j], sinLoad[j].p);
    connect(sinLoad[j].n, sinMachine.neutral);
  end for;
  connect(trapMachine.neutral, neutralLift.p);
  connect(neutralLift.n, trapGround.p);
  connect(sinMachine.neutral, sinGround.p);
end FireHarness_PermanentMagnet3Phase;
