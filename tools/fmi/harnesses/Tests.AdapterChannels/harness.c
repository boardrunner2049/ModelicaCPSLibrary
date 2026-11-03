/* FMI harness: Tests.AdapterChannels (closed test model, no inputs).
 *
 * Scenario: an eight-channel PwmInput fed {900,1100,...,1600,2100} us and an
 * OctoX SensorValues adapter with the IMU mounted yawed +90 deg and the
 * magnetometer flipped about x, fed constant IMU/magnetometer readings and a
 * GNSS track {t,2t,-3t}. Run to 0.98 s (experiment: 0.035 s) at the
 * experiment's 0.7 ms interval.
 *
 * checks.toml establishes: every PWM channel's value, including clipping at
 * both ends (900 -> 0, 2100 -> 1) and no truncation of channels 7-8; all three
 * axes of the IMU and gyro mount rotation (not transposed) and of the
 * magnetometer flip; GNSS position and velocity pass through in NED without
 * a second conversion, continuously (off-grid instant and end).
 *
 * The model's tunable parameters (PWM ranges, mount matrices) are pinned by
 * its own asserts, so no overrides are applied. No outputs exist, so the
 * console prints only time; values are in the trace. */
#include "fire_harness.h"

int main(void) {
  const fh_scenario_t scenario = {
      .title = "Tests.AdapterChannels",
      .stop_s = 0.98,
      .step_s = 0.0007,
      .print_s = 0.14,
      .inputs = NULL,
      .input_count = 0,
      .outputs = NULL,
      .output_count = 0,
  };
  return fh_run(&scenario);
}
