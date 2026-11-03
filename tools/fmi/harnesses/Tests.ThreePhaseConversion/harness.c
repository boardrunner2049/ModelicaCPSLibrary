/* FMI harness: Tests.ThreePhaseConversion.
 *
 * Closed test bench, no top-level inputs: a PermanentMagnet3Phase converter
 * (polePairs = 3, ke = 0.02 V/(rad/s) phase peak, sinusoidal, theta0 = 0.2)
 * is spun at a prescribed 10 rad/s against a fixed housing while three
 * signal current sources inject i_j = 2 sin(3 theta + 0.2 - (j-1) 2pi/3)
 * into a grounded star. The run covers one full electrical half-turn plus
 * (electrical angle 0.2 -> 3.2 rad), so every phase EMF passes a peak.
 *
 * checks.toml establishes, against closed form:
 *   - electrical angle = 3*10*t + 0.2 and omega = 10 rad/s;
 *   - phase EMF e_j = ke*omega*sin(theta_e - (j-1)2pi/3) at two instants and
 *     peak amplitude ke*omega = 0.2 V;
 *   - torque = 3/2 * ke * Ipeak = 0.06 N.m for all time (balanced currents);
 *   - lossless conversion: sum e_j i_j = tau*omega = 0.6 W for all time;
 *   - star-point KCL: neutral current is zero.
 *
 * Not covered: despite the test's "signed-current" title, the bench only
 * injects currents in phase with the EMF, so torque and power are always
 * positive (motoring); the generating sign is never exercised. It cannot be
 * reached by a parameter override either: the bench's own asserts require
 * torque = +0.06 and fix the current phase to 3*theta + 0.2. The flange and
 * support torques are not checked separately because the component defines
 * them as -torque and +torque. */
#include "fire_harness.h"

int main(void) {
  const fh_scenario_t scenario = {
      .title = "Tests.ThreePhaseConversion",
      .stop_s = 0.1,
      .step_s = 0.0001,
      .print_s = 0.01,
      .inputs = NULL,
      .input_count = 0,
      .outputs = NULL,
      .output_count = 0,
  };
  return fh_run(&scenario);
}
