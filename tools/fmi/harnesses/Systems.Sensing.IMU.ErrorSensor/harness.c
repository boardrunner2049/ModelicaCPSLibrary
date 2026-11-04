/* FMI harness: Systems.Sensing.IMU.ErrorSensor (I2: errors through the device readout).
 *
 * Motion (lib/imu_motion.h): level and at rest for 1 s, then 0.5 rad/s roll.
 * world.toml combines the StatisticalSensor error set (scale +0.5 % accel z,
 * +1 % gyro x, fixed biases, per-tick noise 0.05 m/s2 / 0.003 rad/s, slow
 * Gauss-Markov bias) with temperature coefficients (0.01 m/s2/K on accel x,
 * 0.001 rad/s/K on gyro z; the device runs 10 K above reference) and the
 * DigitalSensor readout (2 ms analog, ADC LSB, 3 ms IIR at 1 ms).
 * equivalentError (vulnerability input, before the analog stage): accel y
 * +1 m/s2 on [0.5, 0.7) s.
 * Noise enters at the ADC, so the delivered noise is the IIR-filtered per-tick
 * noise: variance noiseStd^2 (1 - a)/(1 + a), a = exp(-1 ms / 3 ms). */
#include "fire_harness.h"
#include "imu_motion.h"

static const fh_input_t inputs[] = {
    IMU_MOTION_INPUTS,
    FH_INPUT(sensortemperature, FH_CONSTANT(303.15)),
    FH_INPUT(equivalenterror_1, FH_CONSTANT(0.0)),
    FH_INPUT(equivalenterror_2, FH_PULSE(0.5, 0.2, 0.0, 1.0)),
    FH_INPUT(equivalenterror_3, FH_CONSTANT(0.0)),
    FH_INPUT(equivalenterror_4, FH_CONSTANT(0.0)),
    FH_INPUT(equivalenterror_5, FH_CONSTANT(0.0)),
    FH_INPUT(equivalenterror_6, FH_CONSTANT(0.0)),
};

static const fh_output_t outputs[] = {
    FH_OUTPUT(sampletime),     FH_OUTPUT(acceleration_1), FH_OUTPUT(acceleration_2),
    FH_OUTPUT(acceleration_3), FH_OUTPUT(gyro_1),         FH_OUTPUT(gyro_3),
};

int main(void) {
  const fh_scenario_t scenario = {
      .title = "Systems.Sensing.IMU.ErrorSensor",
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
