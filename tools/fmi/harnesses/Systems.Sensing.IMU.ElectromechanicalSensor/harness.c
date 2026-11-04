/* FMI harness: Systems.Sensing.IMU.ElectromechanicalSensor (I3: physical MEMS axes).
 *
 * Motion (lib/imu_motion.h): level and at rest for 1 s, then 0.5 rad/s roll.
 * Default physics (Physical.Sensors.MEMS): per axis a vibratory gyro (1 kHz
 * drive, 1.5 kHz sense mode, AGC/PLL drive loop, I/Q sense demodulation with a
 * two-pole 3 ms low-pass) and a 2 kHz capacitive proof-mass accelerometer;
 * no noise, 1 ms output period, ADC clipping only.
 * Vulnerability input: accelForce z = 2e-8 N on [0.5, 0.7) s acts on the
 * 1e-8 kg proof mass, so the reading shifts by -F/m = -2 m/s2.
 * Communication step 0.25 ms: with Rumoca 6058a2bb a 1 ms step fails in the
 * first step ("Comb overlap exceeded", the drive mode diverges inside the
 * FMU's integration); 0.5 ms and below run. */
#include "fire_harness.h"
#include "imu_motion.h"

static const fh_input_t inputs[] = {
    IMU_MOTION_INPUTS,
    FH_INPUT(sensortemperature, FH_CONSTANT(293.15)),
    FH_INPUT(gyrodriveforce_1, FH_CONSTANT(0.0)),
    FH_INPUT(gyrodriveforce_2, FH_CONSTANT(0.0)),
    FH_INPUT(gyrodriveforce_3, FH_CONSTANT(0.0)),
    FH_INPUT(gyrosenseforce_1, FH_CONSTANT(0.0)),
    FH_INPUT(gyrosenseforce_2, FH_CONSTANT(0.0)),
    FH_INPUT(gyrosenseforce_3, FH_CONSTANT(0.0)),
    FH_INPUT(accelforce_1, FH_CONSTANT(0.0)),
    FH_INPUT(accelforce_2, FH_CONSTANT(0.0)),
    FH_INPUT(accelforce_3, FH_PULSE(0.5, 0.2, 0.0, 2e-8)),
};

static const fh_output_t outputs[] = {
    FH_OUTPUT(sampletime),     FH_OUTPUT(acceleration_2), FH_OUTPUT(acceleration_3),
    FH_OUTPUT(gyro_1),         FH_OUTPUT(gyro_2),         FH_OUTPUT_BOOL(gyrolocked_1),
};

int main(void) {
  const fh_scenario_t scenario = {
      .title = "Systems.Sensing.IMU.ElectromechanicalSensor",
      .stop_s = 3.0,
      .step_s = 0.00025,
      .print_s = 0.5,
      .inputs = inputs,
      .input_count = FH_COUNT(inputs),
      .outputs = outputs,
      .output_count = FH_COUNT(outputs),
  };
  return fh_run(&scenario);
}
