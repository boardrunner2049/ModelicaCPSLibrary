/* FMI harness: Physical.Electrical.Electromechanical.PermanentMagnet3Phase.
 *
 * Wrapper.mo spins two lossless PM converters (trapezoidal and sinusoidal
 * back-EMF, 7 pole pairs, ke = 0.01 V s/rad, trapezoidal theta0 = 0.4 rad)
 * on one shaft whose speed is
 * imposed exactly, each generating into a star of 0.1 Ohm resistors.
 * The trapezoidal star point is lifted by a 3 V / 50 Hz neutral source and
 * the sinusoidal machine's housing is fixed at phi0 = 0.1 rad, so the
 * converter's neutral and support references are both exercised.
 *
 * Speed profile: at rest for 10 ms, ramp to +500 rad/s by 60 ms, hold to
 * 100 ms, ramp through zero to -300 rad/s by 150 ms, hold to 200 ms.
 * checks.toml establishes: the generator braking torque in closed form
 * (sinusoidal: -1.5*ke^2*omega/R, ripple-free; trapezoidal: between
 * -2 and -3 ke^2*omega/R as one phase is always in transition), its sign
 * reversal with the speed, the back-EMF amplitude ke*omega, the balanced
 * sinusoidal set (zero neutral current), each phase's back-EMF waveform
 * ke*omega*f(theta_e - (j-1)*2*pi/3) at every step (a swapped or mis-shifted
 * phase, or a dropped theta0, fails), the electrical angle as polePairs
 * times the shaft angle relative to the support (sinusoidal housing at
 * 0.1 rad), converter KCL (zero current through the neutral source although
 * the trapezoidal star current swings by +-50 A), the lifted neutral leaving
 * the phase-to-neutral back-EMF intact, the support reaction (+torque on each
 * housing), and power conservation (mechanical power
 * in = resistor losses) for both converters at every step. */
#include "fire_harness.h"

static double speed_profile(double t) {
  if (t < 0.010)
    return 0.0;
  if (t < 0.060)
    return 500.0 * (t - 0.010) / 0.050;
  if (t < 0.100)
    return 500.0;
  if (t < 0.150)
    return 500.0 - 800.0 * (t - 0.100) / 0.050;
  return -300.0;
}

static const fh_input_t inputs[] = {
    FH_INPUT(speed, FH_FUNCTION(speed_profile)),
};

static const fh_output_t outputs[] = {
    FH_OUTPUT(traptorque),        FH_OUTPUT(sintorque),        FH_OUTPUT(electricalangle),
    FH_OUTPUT(trappowerresidual), FH_OUTPUT(sinpowerresidual), FH_OUTPUT(neutralliftcurrent),
    FH_OUTPUT(trapneutralvoltage), FH_OUTPUT(traphousingtorque), FH_OUTPUT(sinhousingtorque),
};

int main(void) {
  const fh_scenario_t scenario = {
      .title = "Physical.Electrical.Electromechanical.PermanentMagnet3Phase",
      .stop_s = 0.2,
      .step_s = 0.00001,
      .print_s = 0.01,
      .inputs = inputs,
      .input_count = FH_COUNT(inputs),
      .outputs = outputs,
      .output_count = FH_COUNT(outputs),
  };
  return fh_run(&scenario);
}
