/* FMI harness: Tests.ElectromechanicalLoad.
 *
 * Closed bench (no top-level inputs): four AverageUnidirectional ESCs at a
 * fixed 50 % command on a 16 V bus each drive a DynamicDCEquivalent motor
 * (R = 0.2 ohm, k = 0.02 N.m/A) against a quadratic load kQ*omega^2. The
 * motors differ in one parameter each relative to motor 1:
 *   motor 2: kQ doubled (3e-7), motor 3: J x4 (4e-4), motor 4: L x2 (2 mH).
 * Each starts at rest, accelerates through a current peak of ~30 A and
 * settles at the loaded equilibrium (R kQ/k) w^2 + k w - 8 V = 0.
 *
 * checks.toml compares against an independent scipy Radau integration of
 *   L di/dt = 8 - R i - k w,  J dw/dt = k i - kQ w^2
 * (rtol 1e-11): the inductance-limited initial current slope (8 V / L), the
 * inertia-limited acceleration, the equilibrium speed/current of each
 * motor, torque balance k*i = kQ*w^2 and power balance 8 V * i = R i^2 +
 * kQ w^3 at steady state, and that the one-quadrant drive never coasts or
 * conducts negative current while continuously commanded.
 *
 * Not covered: the bench holds the command at 0.5 for the whole run, so
 * the ESC coast/freewheel path and the motor's currentZero reinit are never
 * exercised (the closed test has no inputs or parameter that could remove
 * the command mid-run); those belong to the ESC/motor component harnesses. */
#include "fire_harness.h"

int main(void) {
  const fh_scenario_t scenario = {
      .title = "Tests.ElectromechanicalLoad",
      .stop_s = 3.1,
      .step_s = 0.001,
      .print_s = 0.25,
      .inputs = NULL,
      .input_count = 0,
      .outputs = NULL,
      .output_count = 0,
  };
  return fh_run(&scenario);
}
