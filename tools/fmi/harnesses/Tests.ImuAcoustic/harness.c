/* FMI harness: Tests.ImuAcoustic (closed test model, no inputs).
 *
 * Scenario: the test itself, 1 s at its 1 ms interval. AcousticSensor modes at
 * 100 Hz excited by a 1 Pa, 110 Hz tone (linear and quadratic readout), a
 * silent AcousticSensor and a DeviceSensor with the same readout. checks.toml
 * restates the model's asserts and checks the resonance amplitude, aliasing
 * and rectification against closed forms. */
#include "fire_harness.h"

int main(void) {
  const fh_scenario_t scenario = {
      .title = "Tests.ImuAcoustic",
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
