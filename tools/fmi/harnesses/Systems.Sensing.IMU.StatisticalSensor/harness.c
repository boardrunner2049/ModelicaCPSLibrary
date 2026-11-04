/* FMI harness: Systems.Sensing.IMU.StatisticalSensor (I1 statistical IMU).
 *
 * Motion (lib/imu_motion.h): level and at rest in gravity for 1 s, then a
 * constant 0.5 rad/s roll about body x for 2 s. world.toml sets a 2.5 ms
 * output period (not a multiple of the 1 ms communication step, so samples are
 * FMU time events inside steps), scale errors on accel z (+0.5 %) and gyro x
 * (+1 %), fixed biases, per-sample white noise (0.05 m/s2, 0.003 rad/s) and a
 * slow Gauss-Markov bias (std 0.02 m/s2 / 0.001 rad/s, tau 1000 s, so it is
 * an almost constant random offset over the run).
 * Sensor law (PartialSampledSensor, latency 0):
 *   acceleration = (I + diag(scale))*f_b + accelBias + biasState + noiseStd*n
 *   gyro         = (I + diag(scale))*omega_b + gyroBias + biasState + noiseStd*n
 * with f_b = (0, -g sin phi, -g cos phi), phi = 0.5 (t - 1) while rolling.
 * checks.toml: sample epochs on the 2.5 ms grid, the stationary/rolling
 * readings, and the residual statistics against the configured noise. */
#include "fire_harness.h"
#include "imu_motion.h"

static const fh_input_t inputs[] = {
    IMU_MOTION_INPUTS,
    FH_INPUT(sensortemperature, FH_CONSTANT(293.15)),
};

static const fh_output_t outputs[] = {
    FH_OUTPUT(sampletime),     FH_OUTPUT_INT(sequence),   FH_OUTPUT(acceleration_2),
    FH_OUTPUT(acceleration_3), FH_OUTPUT(gyro_1),         FH_OUTPUT(gyro_3),
};

int main(void) {
  const fh_scenario_t scenario = {
      .title = "Systems.Sensing.IMU.StatisticalSensor",
      .stop_s = 3.0,
      .step_s = 0.0004,
      .print_s = 0.5,
      .inputs = inputs,
      .input_count = FH_COUNT(inputs),
      .outputs = outputs,
      .output_count = FH_COUNT(outputs),
  };
  return fh_run(&scenario);
}
