/* FMI harness: Systems.Sensing.IMU.DeviceSensor (legacy I2: device readout with errors).
 *
 * Motion (lib/imu_motion.h): level and at rest for 1 s, then 0.5 rad/s roll.
 * world.toml: accel x misaligned towards y by 1 %, gyro x g-sensitivity
 * 0.001 (rad/s)/(m/s2) to accel z, fixed biases, per-tick white noise
 * (0.02 m/s2, 0.001 rad/s), temperature coefficient 0.005 m/s2/K on accel z
 * with the device 10 K below reference, ADC LSB 0.005 m/s2 / 0.0005 rad/s,
 * default 1 ms analog response, no IIR, 1 ms output period. */
#include "fire_harness.h"
#include "imu_motion.h"

static const fh_input_t inputs[] = {
    IMU_MOTION_INPUTS,
    FH_INPUT(sensortemperature, FH_CONSTANT(283.15)),
};

static const fh_output_t outputs[] = {
    FH_OUTPUT(sampletime),     FH_OUTPUT(acceleration_1), FH_OUTPUT(acceleration_2),
    FH_OUTPUT(acceleration_3), FH_OUTPUT(gyro_1),         FH_OUTPUT(gyro_3),
};

int main(void) {
  const fh_scenario_t scenario = {
      .title = "Systems.Sensing.IMU.DeviceSensor",
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
