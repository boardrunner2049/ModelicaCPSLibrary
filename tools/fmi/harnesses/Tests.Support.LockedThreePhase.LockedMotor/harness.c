/* FMI harness: Tests.Support.LockedThreePhase.LockedMotor.
 *
 * Wrapper.mo drives the locked-rotor ThreePhasePM (0.2 Ohm, 1 mH per phase,
 * tau = L/R = 5 ms, ke = 0.01, 7 pole pairs, trapezoidal back-EMF,
 * theta0 = 0.3 rad) from three independent phase voltage sources to ground;
 * the star point floats. With the shaft bolted down the electrical angle is
 * frozen at theta0 and the back-EMF is zero, so each phase is an R-L driven
 * by v_j - mean(v), and the torque ke*sum(f_j*i_j) uses the frozen shape
 * f = {0.5730, -1, +1}: every phase carries a different torque weight.
 *
 * Scenario (unbalanced, phase-distinct voltages):
 *   [0, 10) ms     va = vb = vc = 0          at rest, no current
 *   [10, 50) ms    (2, -1, 1) V              currents rise to (20, -25, 5)/3 A
 *   [50, 90) ms    (7, 4, 6) V               +5 V common-mode step: currents
 *                                            must continue undisturbed
 *   from 70 ms     loadTorque = 0.05 N m     only the lock reaction changes
 *   [90, 150) ms   (-2, 1, -1) V             reversal through zero (93.47 ms)
 * checks.toml establishes, from the closed-form first-order response: the
 * per-phase currents (catches swapped phases), the torque from the frozen
 * trapezoid shape, the lock reaction torque = Te - loadTorque, no motion
 * and no back-EMF, the electrical angle frozen at theta0, common-mode
 * rejection and the star point at the mean phase voltage, floating-star KCL,
 * copper loss and magnetic energy, and the integrated energy balance. */
#include "fire_harness.h"

static double segment(double t, double v1, double v2, double v3) {
  if (t < 0.010)
    return 0.0;
  if (t < 0.050)
    return v1;
  if (t < 0.090)
    return v2;
  return v3;
}

static double va_profile(double t) { return segment(t, 2.0, 7.0, -2.0); }
static double vb_profile(double t) { return segment(t, -1.0, 4.0, 1.0); }
static double vc_profile(double t) { return segment(t, 1.0, 6.0, -1.0); }

static const fh_input_t inputs[] = {
    FH_INPUT(va, FH_FUNCTION(va_profile)),
    FH_INPUT(vb, FH_FUNCTION(vb_profile)),
    FH_INPUT(vc, FH_FUNCTION(vc_profile)),
    FH_INPUT(loadtorque, FH_STEP(0.070, 0.0, 0.05)),
};

static const fh_output_t outputs[] = {
    FH_OUTPUT(ia),         FH_OUTPUT(ib),          FH_OUTPUT(ic),          FH_OUTPUT(torque),
    FH_OUTPUT(locktorque), FH_OUTPUT(starvoltage), FH_OUTPUT(energyerror),
};

int main(void) {
  const fh_scenario_t scenario = {
      .title = "Tests.Support.LockedThreePhase.LockedMotor",
      .stop_s = 0.15,
      .step_s = 0.0001,
      .print_s = 0.005,
      .inputs = inputs,
      .input_count = FH_COUNT(inputs),
      .outputs = outputs,
      .output_count = FH_COUNT(outputs),
  };
  return fh_run(&scenario);
}
