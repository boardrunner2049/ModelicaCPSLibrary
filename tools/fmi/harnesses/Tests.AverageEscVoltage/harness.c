/* FMI harness: Tests.AverageEscVoltage.
 *
 * Closed bench (no inputs): five AverageVoltage ESCs with commands
 * {-0.5, 0, 0.25, 1, 2} share a supply of 16 V (t < 0.1 s), 8 V
 * (0.1-0.2 s) and 0 V afterwards; a five-channel PwmInput with per-channel
 * calibration ranges maps pulse widths {500,1100,1350,1700,1800} us to the
 * same clamped values {0, 0, 0.25, 1, 1}. The model's asserts (clamp, PWM
 * calibration, supply scaling within 1e-12) run inside the FMU.
 * checks.toml checks the clamp, the per-channel PWM calibration and the
 * supply scaling at each supply level (native rumoca sim agrees exactly). */
#include "fire_harness.h"

int main(void) {
  const fh_scenario_t scenario = {
      .title = "Tests.AverageEscVoltage",
      .stop_s = 0.3,
      .step_s = 0.001,
      .print_s = 0.05,
      .inputs = NULL,
      .input_count = 0,
      .outputs = NULL,
      .output_count = 0,
  };
  return fh_run(&scenario);
}
