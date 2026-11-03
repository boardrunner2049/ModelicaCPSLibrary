/* FMI harness: Tests.SensorPhysics (closed test model, no inputs).
 *
 * Scenario: the test itself. Three IMU.Sensor instances with constant inputs:
 * freefall (a_w = g, CG mount), supported (a_w = 0, CG mount), and a lever-arm
 * sensor at r_b = {1,0,0} yawed +90 deg in the body with biases {.1,.2,.3},
 * body rate {1,2,3} rad/s and angular acceleration {0,0,3} rad/s^2. Run to the
 * experiment's 25 ms at its 1 ms interval; the model asserts at 5 ms. */
#include "fire_harness.h"

int main(void) {
  const fh_scenario_t scenario = {
      .title = "Tests.SensorPhysics",
      .stop_s = 0.025,
      .step_s = 0.001,
      .print_s = 0.005,
      .inputs = NULL,
      .input_count = 0,
      .outputs = NULL,
      .output_count = 0,
  };
  return fh_run(&scenario);
}
