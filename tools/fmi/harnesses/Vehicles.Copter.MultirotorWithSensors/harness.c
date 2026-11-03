/* FMI harness: Vehicles.Copter.MultirotorWithSensors.
 *
 * The QuadX MultirotorPlant (1.5 kg, I = diag(0.02,0.02,0.04), start NED
 * {0,0,-1} facing East, legs 0.10 m below the CG over ground z = 0) plus the
 * ideal SensorSuite: IMU and magnetometer at the CG with R_bImu = R_bMag =
 * diag(1,-1,-1) (sensor axes are body FRD with y and z flipped), barometer
 * at the CG. Rotor i local +z thrust is body -z. Tunable-parameter override
 * (world.toml): geometry.gnssPosition_C = (0.1, 0, -0.05) m, a GNSS antenna
 * forward of and above the CG (default: at the CG).
 *
 * Scenario:
 *   0.0-0.2  level hover, each rotor f0 = m g / 4
 *   0.2-0.3  external body-y (South) push 1.5 N: 1 m/s^2 South
 *   0.4-0.9  tilt manoeuvre about the body axis n = (x+y)/sqrt(2): external
 *            moment (+0.2,+0.2,0) N.m on [0.4,0.5), -(..) on [0.5,0.6) ->
 *            tilted by Theta = 0.1414214 rad at rest rate, hold [0.6,0.7),
 *            then the mirror doublet back to level at 0.9. With Ixx = Iyy and
 *            no yaw rate the rate stays along n, so the tilt angle theta(t)
 *            is exactly piecewise quadratic; the collective is f0/cos(theta)
 *            (evaluated at the middle of each 1 ms hold) so the vertical
 *            thrust stays m g and the altitude is held, while the horizontal
 *            thrust component accelerates the vehicle South-West.
 *   1.5-2.0  collective 1.2 f0 (climb at 0.2 g)
 *   2.0-2.5  hover thrust (constant climb rate)
 *   2.5-2.6  rotor-1 local moment +0.2 N.m: yaw spin-up to -0.5 rad/s, which
 *            then persists (no aerodynamic damping)
 *   3.5-     thrust cut: free fall while spinning, touchdown near 4.34 s
 *            (CG reaches leg height z = -0.1), sliding South-West and
 *            spinning; leg friction stops both, rest on the legs
 *   ambient: magnetic field {20, 2, 45} uT NED; pressure ramps 101325 ->
 *            100000 Pa over [2,3] s; temperature steps 288.15 -> 300 K at 2 s
 * checks.toml establishes the sensor contract: the accelerometer measures
 * specific force (g up at hover, -1 m/s^2 on sensor y under the South push,
 * g/cos(Theta) on sensor z and nothing on x/y while tilted, 1.2 g in the
 * climb, zero in free fall, g again at rest), the gyro x/y/z follow the body
 * rates with the sensor y/z flips, the magnetometer follows yaw and tilt,
 * GNSS reports the NED antenna position/velocity p_w + R_wb r and
 * v_w + R_wb (omega x r) at every sample (the lever arm shows as +0.1 m East
 * at heading East, -0.05 m Down, and 0.05 m/s while spinning), including the
 * horizontal drift from the push and the tilt integrated in closed form,
 * the barometer reports
 * up-positive altitude/climb rate, and the ambient inputs pass through.
 * Tilt and landing values are cross-checked against native `rumoca sim` of
 * the same profile. */
#include "fire_harness.h"

#include <math.h>

#define MASS 1.5
#define G 9.80665
#define F0 (MASS * G / 4.0)
#define STEP 0.001
#define EPS 1e-6 /* communication times are not exact multiples */
#define TILT_M 0.2                     /* per-axis tilt moment [N.m] */
#define TILT_A (TILT_M * 1.4142135623730951 / 0.02) /* rad/s^2 about n */
#define TILT_D 0.1                     /* doublet half-length [s] */

static int between(double t, double a, double b) { return t >= a - EPS && t < b - EPS; }

/* Exact tilt angle about n for the moment schedule below. */
static double tilt_angle(double t) {
  const double a = TILT_A, d = TILT_D, full = a * d * d;
  if (t < 0.4)
    return 0.0;
  if (t < 0.5)
    return 0.5 * a * (t - 0.4) * (t - 0.4);
  if (t < 0.6)
    return full / 2 + a * d * (t - 0.5) - 0.5 * a * (t - 0.5) * (t - 0.5);
  if (t < 0.7)
    return full;
  if (t < 0.8)
    return full - 0.5 * a * (t - 0.7) * (t - 0.7);
  if (t < 0.9)
    return full / 2 - a * d * (t - 0.8) + 0.5 * a * (t - 0.8) * (t - 0.8);
  return 0.0;
}

static double thrust(double t) {
  if (between(t, 0.4, 0.9))
    return F0 / cos(tilt_angle(t + STEP / 2)); /* hold-midpoint value */
  if (between(t, 1.5, 2.0))
    return 1.2 * F0;
  if (t >= 3.5 - EPS)
    return 0.0;
  return F0;
}

static double push_y(double t) { return between(t, 0.2, 0.3) ? 1.5 : 0.0; }

static double tilt_moment(double t) {
  if (between(t, 0.4, 0.5) || between(t, 0.8, 0.9))
    return TILT_M;
  if (between(t, 0.5, 0.6) || between(t, 0.7, 0.8))
    return -TILT_M;
  return 0.0;
}

static double yaw_moment_r1(double t) { return between(t, 2.5, 2.6) ? 0.2 : 0.0; }

static const fh_input_t inputs[] = {
    FH_INPUT(gravity_w_1, FH_CONSTANT(0.0)),
    FH_INPUT(gravity_w_2, FH_CONSTANT(0.0)),
    FH_INPUT(gravity_w_3, FH_CONSTANT(G)),
    FH_INPUT(wind_w_1, FH_CONSTANT(0.0)),
    FH_INPUT(wind_w_2, FH_CONSTANT(0.0)),
    FH_INPUT(wind_w_3, FH_CONSTANT(0.0)),
    FH_INPUT(externalforce_b_1, FH_CONSTANT(0.0)),
    FH_INPUT(externalforce_b_2, FH_FUNCTION(push_y)),
    FH_INPUT(externalforce_b_3, FH_CONSTANT(0.0)),
    FH_INPUT(externalmoment_b_1, FH_FUNCTION(tilt_moment)),
    FH_INPUT(externalmoment_b_2, FH_FUNCTION(tilt_moment)),
    FH_INPUT(externalmoment_b_3, FH_CONSTANT(0.0)),
    FH_INPUT(rotorforce_r_1, FH_CONSTANT(0.0)),
    FH_INPUT(rotorforce_r_2, FH_CONSTANT(0.0)),
    FH_INPUT(rotorforce_r_3, FH_FUNCTION(thrust)),
    FH_INPUT(rotorforce_r_4, FH_CONSTANT(0.0)),
    FH_INPUT(rotorforce_r_5, FH_CONSTANT(0.0)),
    FH_INPUT(rotorforce_r_6, FH_FUNCTION(thrust)),
    FH_INPUT(rotorforce_r_7, FH_CONSTANT(0.0)),
    FH_INPUT(rotorforce_r_8, FH_CONSTANT(0.0)),
    FH_INPUT(rotorforce_r_9, FH_FUNCTION(thrust)),
    FH_INPUT(rotorforce_r_10, FH_CONSTANT(0.0)),
    FH_INPUT(rotorforce_r_11, FH_CONSTANT(0.0)),
    FH_INPUT(rotorforce_r_12, FH_FUNCTION(thrust)),
    FH_INPUT(rotormoment_r_1, FH_CONSTANT(0.0)),
    FH_INPUT(rotormoment_r_2, FH_CONSTANT(0.0)),
    FH_INPUT(rotormoment_r_3, FH_FUNCTION(yaw_moment_r1)),
    FH_INPUT(rotormoment_r_4, FH_CONSTANT(0.0)),
    FH_INPUT(rotormoment_r_5, FH_CONSTANT(0.0)),
    FH_INPUT(rotormoment_r_6, FH_CONSTANT(0.0)),
    FH_INPUT(rotormoment_r_7, FH_CONSTANT(0.0)),
    FH_INPUT(rotormoment_r_8, FH_CONSTANT(0.0)),
    FH_INPUT(rotormoment_r_9, FH_CONSTANT(0.0)),
    FH_INPUT(rotormoment_r_10, FH_CONSTANT(0.0)),
    FH_INPUT(rotormoment_r_11, FH_CONSTANT(0.0)),
    FH_INPUT(rotormoment_r_12, FH_CONSTANT(0.0)),
    FH_INPUT(magneticfield_w_1, FH_CONSTANT(20e-6)),
    FH_INPUT(magneticfield_w_2, FH_CONSTANT(2e-6)),
    FH_INPUT(magneticfield_w_3, FH_CONSTANT(45e-6)),
    FH_INPUT(pressure, FH_RAMP(2.0, 101325.0, 3.0, 100000.0)),
    FH_INPUT(temperature, FH_STEP(2.0, 288.15, 300.0)),
};

/* The SensorMeasurements record is exported with local causality, so the
 * console shows the plant outputs; the trace has every measurement. */
static const fh_output_t outputs[] = {
    FH_OUTPUT(force_b_3),
    FH_OUTPUT(moment_b_3),
};

int main(void) {
  const fh_scenario_t scenario = {
      .title = "Vehicles.Copter.MultirotorWithSensors",
      .stop_s = 6.0,
      .step_s = STEP,
      .print_s = 0.25,
      .inputs = inputs,
      .input_count = FH_COUNT(inputs),
      .outputs = outputs,
      .output_count = FH_COUNT(outputs),
  };
  return fh_run(&scenario);
}
