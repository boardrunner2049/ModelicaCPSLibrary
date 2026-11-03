/* FMI harness: Tests.PropulsionStep (closed test model, no inputs).
 *
 * Scenario: an ESC PassThrough with command 0.5 drives a FirstOrderSpeed motor
 * (tau 50 ms, omegaMax 100 rad/s) whose shaft spins a CCW (spinSign +1) and a
 * CW (spinSign -1) QuadraticBlade; two more motors get demands 2 and -1.
 * 0.5 s (10 tau) at a 1 ms step. world.toml gives the CW blade a 1.7x thrust
 * coefficient (cw.kT = 1.7e-3; the model asserts only the CCW thrust).
 *
 * checks.toml establishes: the closed-form step 50(1 - e^-t/tau) at tau,
 * 3 tau, 10 tau with no overshoot; thrust kT w^2 follows the actual (lagging)
 * speed for each blade with its own kT (thrust ratio 1.7 at every instant);
 * yaw torques oppose spin, have opposite signs, cancel exactly and never flip;
 * load torque is nonnegative; demand 2 clamps at omegaMax, demand -1 at 0.
 *
 * No outputs exist, so the console prints only time; values are in the trace. */
#include "fire_harness.h"

int main(void) {
  const fh_scenario_t scenario = {
      .title = "Tests.PropulsionStep",
      .stop_s = 0.5,
      .step_s = 0.001,
      .print_s = 0.05,
      .inputs = NULL,
      .input_count = 0,
      .outputs = NULL,
      .output_count = 0,
  };
  return fh_run(&scenario);
}
