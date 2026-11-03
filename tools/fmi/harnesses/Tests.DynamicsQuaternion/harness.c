/* FMI harness: Tests.DynamicsQuaternion.
 *
 * Self-contained test model (no inputs): a force-free RigidBody6DOF spinning
 * at 2 rad/s about body z from an initial 90-degree roll. Its own asserts check
 * the closed-form attitude and velocity every step; checks.toml checks them
 * again on the trace. The initial asserts exercise quaternionToRotationMatrix,
 * whose per-element call asserts inside a loop comprehension. */
#include "fire_harness.h"

int main(void) {
  const fh_scenario_t scenario = {
      .title = "Tests.DynamicsQuaternion",
      .stop_s = 1.0,
      .step_s = 0.001,
      .print_s = 0.25,
      .inputs = NULL,
      .input_count = 0,
      .outputs = NULL,
      .output_count = 0,
  };
  return fh_run(&scenario);
}
