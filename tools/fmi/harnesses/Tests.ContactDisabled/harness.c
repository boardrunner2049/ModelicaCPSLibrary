/* FMI harness: Tests.ContactDisabled.
 *
 * Closed test model (no inputs): a MultirotorWithSensors with its landing
 * gear disabled, dropped from CG Down -0.12 m (feet 2 cm above ground) with
 * zero rotor wrench. The scenario is the run itself: 0.6 s (the model's
 * StopTime) at 1 ms. checks.toml checks the closed-form free fall
 * -0.12 + g t^2/2 straight through the ground plane, zero gear wrench and a
 * zero accelerometer reading. */
#include "fire_harness.h"

int main(void) {
  const fh_scenario_t scenario = {
      .title = "Tests.ContactDisabled",
      .stop_s = 0.6,
      .step_s = 0.001,
      .print_s = 0.06,
      .inputs = NULL,
      .input_count = 0,
      .outputs = NULL,
      .output_count = 0,
  };
  return fh_run(&scenario);
}
