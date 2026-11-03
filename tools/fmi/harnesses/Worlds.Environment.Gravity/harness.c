/* FMI harness: Worlds.Environment.Gravity.
 *
 * Uniform gravity source: gravity = {0, 0, g} in the NED world frame, with the
 * parameter g bound by default to Utilities.Constants.g = 9.80665 m/s2.
 * A binding to a package constant is where compile-time folding is most likely, so
 * world.toml overrides g with 3.71 m/s2 (Mars surface gravity). The checks establish
 * that the override (not the constant) reaches the output, the NED sign convention
 * (gravity points Down = +z, nothing horizontal) and that the field is constant in
 * time. The default case is covered by a native `rumoca sim` of the class, which
 * gives {0, 0, 9.80665}. */
#include "fire_harness.h"

static const fh_output_t outputs[] = {
    FH_OUTPUT(gravity_1),
    FH_OUTPUT(gravity_2),
    FH_OUTPUT(gravity_3),
};

int main(void) {
  const fh_scenario_t scenario = {
      .title = "Worlds.Environment.Gravity",
      .stop_s = 1.0,
      .step_s = 0.01,
      .print_s = 0.5,
      .inputs = NULL,
      .input_count = 0,
      .outputs = outputs,
      .output_count = FH_COUNT(outputs),
  };
  return fh_run(&scenario);
}
