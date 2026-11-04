/* FMI harness: Tests.ChassisMassProperties.
 *
 * A closed, parameter-only test model: three ChassisAssembly instances
 * (assembled mode with a core, one 90-deg-yawed arm and a payload with a full
 * inertia tensor; aggregate mode with conflicting constituents; a core-only
 * assembly with empty part arrays) and initial-equation asserts on the
 * combined properties. There are no inputs; the scenario runs the model's
 * own StopTime (0.01 s) so the trace records the initialized values.
 *
 * checks.toml repeats the model's own assertions as independent trace checks
 * (mass 4 kg, centre {1,-2,3}, the full tensor
 * [6.6,-3.95,-4; -3.95,13.6,-2; -4,-2,14.9] recomputed by hand/numpy, aggregate
 * pass-through, core-only pass-through), so a failure is located per value
 * rather than as one assertion message.
 *
 * The model's own "Incorrect rotated/parallel-axis full inertia tensor"
 * assertion fired under a Rumoca defect (products of inertia about the origin
 * instead of the combined CG) fixed as of a4dd197b. */
#include "fire_harness.h"

static const fh_output_t outputs[] = {
    FH_OUTPUT(totalmass),
};

int main(void) {
  const fh_scenario_t scenario = {
      .title = "Tests.ChassisMassProperties",
      .stop_s = 0.01,
      .step_s = 0.001,
      .print_s = 0,
      .inputs = NULL,
      .input_count = 0,
      .outputs = outputs,
      .output_count = FH_COUNT(outputs),
  };
  return fh_run(&scenario);
}
