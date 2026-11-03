/* FMI harness: Tests.VoltageMultirotor (closed test model, no inputs).
 *
 * Scenario: two identical QuadX MultirotorWithSensors vehicles (1.5 kg) start
 * 2 cm above the ground with remapped channels (actuatorIndex {3,2,4,1},
 * spinSign {-1,1,-1,1}) and asymmetric commands {0.1,0.2,0.3,0.4}. P0 is
 * driven by ESC PassThrough + FirstOrderSpeed, P1 by a 16 V AverageVoltage ESC
 * + FirstOrderVoltageToSpeed. 3 N of thrust cannot lift 14.7 N, so both fall,
 * touch down at ~64 ms and settle on the landing gear while the yaw torque
 * turns them slowly. 1 s (experiment: 0.2 s) at a 1 ms step.
 *
 * checks.toml establishes: the channel remap (rotor i <- command[actuatorIndex
 * [i]]); closed-form rotor speeds and thrusts for both drive models; the
 * airborne wrench (thrust, roll/pitch/yaw signs and sizes) and acceleration;
 * no contact before touchdown (gaps, flags, zero force); unilateral bounded
 * contact; settled state on all four legs: vertical balance, equal diagonal
 * loads, static closed-form leg forces (and tight native values), zero net
 * moment and acceleration, IMU reading +1 g, steady yaw rate from viscous
 * leg friction; and P0/P1 equivalence throughout, touchdown included.
 *
 * No outputs exist, so the console prints only time; values are in the trace. */
#include "fire_harness.h"

int main(void) {
  const fh_scenario_t scenario = {
      .title = "Tests.VoltageMultirotor",
      .stop_s = 1.0,
      .step_s = 0.001,
      .print_s = 0.1,
      .inputs = NULL,
      .input_count = 0,
      .outputs = NULL,
      .output_count = 0,
  };
  return fh_run(&scenario);
}
