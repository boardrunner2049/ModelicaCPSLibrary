/* FMI harness: Tests.ImuDevice (closed test model, no inputs).
 *
 * Scenario: the test itself, 50 ms at its 0.1 ms interval. Four DeviceSensor
 * instances on a 4 ms output clock decimating 1 ms ADC ticks: a 10 m/s2 step
 * through a 10 ms analog response, the same through a 2 ms IIR, ADC clipping
 * and rounding, and a temperature coefficient. checks.toml restates the
 * model's asserts and the analog response at every sample. */
#include "fire_harness.h"

int main(void) {
  const fh_scenario_t scenario = {
      .title = "Tests.ImuDevice",
      .stop_s = 0.05,
      .step_s = 0.0001,
      .print_s = 0.01,
      .inputs = NULL,
      .input_count = 0,
      .outputs = NULL,
      .output_count = 0,
  };
  return fh_run(&scenario);
}
