/* FMI harness: Tests.ImuStatistical (closed test model, no inputs).
 *
 * Scenario: the test itself, 1 s at its 1 ms interval. Five StatisticalSensor
 * instances: a delayed one (10 ms period, latency 2, a_w = (t, 0, 0)), two
 * with the same seed and stream (exact replay), one on another stream, and
 * one with scale, cross-axis and g-sensitivity calibration errors.
 * checks.toml restates the model's asserts as closed forms, checks the
 * delivery queue at every row and the noise statistics of the streams. */
#include "fire_harness.h"

int main(void) {
  const fh_scenario_t scenario = {
      .title = "Tests.ImuStatistical",
      .stop_s = 1,
      .step_s = 0.001,
      .print_s = 0.1,
      .inputs = NULL,
      .input_count = 0,
      .outputs = NULL,
      .output_count = 0,
  };
  return fh_run(&scenario);
}
