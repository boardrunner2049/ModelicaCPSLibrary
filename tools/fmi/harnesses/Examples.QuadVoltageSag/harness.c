/* FMI harness: Examples.QuadVoltageSag.
 *
 * Closed model, no inputs: MultirotorWithSensors (QuadX, 1.5 kg, landing gear
 * disabled, start NED {0,0,-1}), AverageVoltage ESCs at the fixed hover
 * modulation command c = sqrt(m g/(4 kT))/1000 = 0.60642, voltage-to-speed
 * motors (Kv 62.5 rad/(s.V), tau 0.03 s, started at hover 606.42 rad/s),
 * QuadraticBlade rotors (kT 1e-5, kQ 1.5e-7). The DC bus is 16 V before 1 s
 * and 12 V after (the model's documented supply drop).
 *
 * Run: experiment StopTime 2 s at its own Interval (2 ms step). At 16 V the
 * quad hovers exactly; the drop lowers the equivalent voltage to 12 c, the
 * speed settles at 3/4 of hover (454.82 rad/s), thrust at 9/16 of the weight,
 * and the vehicle sinks with 7/16 g (no gear: it passes z = 0 freely).
 * checks.toml uses the closed-form speed w_h (3/4 + 1/4 exp(-(t-1)/tau)) and
 * an independent 1-D integration (scipy, rtol 1e-12) of z'' = g - 4 kT w^2/m
 * for altitude and sink rate.
 */
#include "fire_harness.h"

static const fh_output_t outputs[] = {
    FH_OUTPUT(dcvoltage),
    FH_OUTPUT(altitude),
};

int main(void) {
  const fh_scenario_t scenario = {
      .title = "Examples.QuadVoltageSag",
      .stop_s = 2.0,
      .step_s = 0.002,
      .print_s = 0.1,
      .inputs = NULL,
      .input_count = 0,
      .outputs = outputs,
      .output_count = FH_COUNT(outputs),
  };
  return fh_run(&scenario);
}
