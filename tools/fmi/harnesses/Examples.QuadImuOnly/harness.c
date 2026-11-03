/* FMI harness: Examples.QuadImuOnly.
 *
 * Closed model, no inputs: MultirotorPlant (QuadX default geometry, 1.5 kg,
 * start NED {0,0,-1} facing East, landing gear 0.9 m above the ground) with a
 * directly composed IMU (r_b = imuPosition - cg = 0, R_bs = R_bImu =
 * diag(1,-1,-1)) instead of SensorSuite; FirstOrderSpeed motors (tau 0.03 s,
 * omegaMax 1000) started at and commanded through PassThrough ESCs to the
 * hover speed sqrt(m g/(4 kT)) = 606.42 rad/s; QuadraticBlade rotors (kT 1e-5,
 * kQ 1.5e-7).
 *
 * Run: experiment StopTime 1 s at its own Interval (2.5 ms step). The model is
 * an exact hover equilibrium; its own `when time >= 0.01` assert requires the
 * IMU to read {0,0,g} and zero rate within 1e-8 (the FMU would stop with an
 * error otherwise, so a completed run already means the assert held).
 * checks.toml establishes the equilibrium (speeds, ESC command, per-rotor and
 * total thrust, moment balance, pose held), the IMU mount (r_b, R_bs) and its
 * output equations against truth on every row (specific force
 * R_bs^T R_wb^T (a_w - g_w), gyro R_bs^T omega_b), and the East-facing
 * attitude from q_start. Any parameter perturbation would move the vehicle
 * and violate the model's own assert, so none is applied.
 */
#include "fire_harness.h"

static const fh_output_t outputs[] = {
    FH_OUTPUT(acceleration_1), FH_OUTPUT(acceleration_2), FH_OUTPUT(acceleration_3),
    FH_OUTPUT(gyro_1),         FH_OUTPUT(gyro_2),         FH_OUTPUT(gyro_3),
    FH_OUTPUT(truth_p_w_3),
};

int main(void) {
  const fh_scenario_t scenario = {
      .title = "Examples.QuadImuOnly",
      .stop_s = 1.0,
      .step_s = 0.0025,
      .print_s = 0.1,
      .inputs = NULL,
      .input_count = 0,
      .outputs = outputs,
      .output_count = FH_COUNT(outputs),
  };
  return fh_run(&scenario);
}
