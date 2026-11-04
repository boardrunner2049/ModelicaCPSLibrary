/* FMI harness: Tests.ImuElectromechanical (closed test model, no inputs).
 *
 * Scenario: the test itself, 1 s (its StopTime). An ElectromechanicalSensor
 * at constant specific force (1, -2, -g) and body rate (0.2, -0.3, 0.1) rad/s,
 * where the rate is recovered from the Coriolis displacement of each vibratory
 * gyro, and a separate VibratoryGyro starting at 10 % drive amplitude with its
 * mechanics detuned to 1003 Hz against a 1000 Hz reference (AGC/PLL startup).
 * Communication step 0.25 ms instead of the 1 ms experiment interval: with
 * Rumoca 6058a2bb a 1 ms step fails in the first step ("Comb overlap
 * exceeded; electrode model invalid": the 1 kHz drive mode diverges inside
 * the FMU's integration of that step); 0.5 ms and below run. */
#include "fire_harness.h"

int main(void) {
  const fh_scenario_t scenario = {
      .title = "Tests.ImuElectromechanical",
      .stop_s = 1.0,
      .step_s = 0.00025,
      .print_s = 0.1,
      .inputs = NULL,
      .input_count = 0,
      .outputs = NULL,
      .output_count = 0,
  };
  return fh_run(&scenario);
}
