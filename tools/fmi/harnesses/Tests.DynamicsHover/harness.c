/* FMI harness: Tests.DynamicsHover (closed test model, no inputs).
 *
 * Scenario: a 2 kg RigidBody6DOF at NED z = -1 m with body thrust -19.62 N
 * (= m g) and no moment; 5 s (five times the experiment StopTime) at a 1 ms
 * step so any drift would accumulate. world.toml starts the body yawed by
 * 0.7 rad (q_start override): body z stays vertical, so it must still hover,
 * now with a non-identity rotation matrix.
 *
 * checks.toml establishes: position, velocity and inertial acceleration stay
 * at the equilibrium within 1e-9 for 5 s; the accelerometer reads -g on body z
 * (FRD up is -z) and nothing laterally; the yawed start attitude is held
 * (q, euler yaw 0.7, zero roll/pitch, unit norm, R_wb[3,3] = 1); no rotation.
 *
 * No outputs exist, so the console prints only time; values are in the trace. */
#include "fire_harness.h"

int main(void) {
  const fh_scenario_t scenario = {
      .title = "Tests.DynamicsHover",
      .stop_s = 5.0,
      .step_s = 0.001,
      .print_s = 0.5,
      .inputs = NULL,
      .input_count = 0,
      .outputs = NULL,
      .output_count = 0,
  };
  return fh_run(&scenario);
}
