/* Shared rigid-body motion for the IMU harnesses (Systems.Sensing.IMU.*).
 *
 * NED world, FRD body, the sensor at the CG (r_b = 0) with an identity mount:
 *   [0, IMU_ROLL_START)  level and at rest in gravity g = (0, 0, G0)
 *   [IMU_ROLL_START, ..) constant roll rate IMU_ROLL_RATE about body x,
 *                        R_wb = Rx(phi), phi = IMU_ROLL_RATE * (t - IMU_ROLL_START),
 *                        omega_b = (IMU_ROLL_RATE, 0, 0), alpha_b = 0, a_w = 0.
 * The ideal sensor then reads
 *   gyro         = (W, 0, 0) while rolling, 0 before;
 *   acceleration = R_wb' (a_w - g) = (0, -G0 sin(phi), -G0 cos(phi)).
 * checks.toml evaluates these at the delivered sample's acquisition epoch
 * (m.sampleTime), so a wrong epoch shows up as an attitude error.
 *
 * IMU_MOTION_INPUTS expands to the 21 fh_input_t entries of the IMU input
 * vector (R_wb row-major, a_w, gravity_w, omega_b, alpha_b), using the aliases
 * new_harness.py gives them (r_wb_1.., a_w_1.., ...). */
#ifndef FIRE_IMU_MOTION_H
#define FIRE_IMU_MOTION_H

#include <math.h>

#define IMU_G0 9.80665
#define IMU_ROLL_START 1.0
#define IMU_ROLL_RATE 0.5

static inline double imu_phi(double t) {
  return t < IMU_ROLL_START ? 0.0 : IMU_ROLL_RATE * (t - IMU_ROLL_START);
}

/* Rx(phi) = [1 0 0; 0 c -s; 0 s c], row-major. */
static inline double imu_r(double t, int k) {
  const double c = cos(imu_phi(t)), s = sin(imu_phi(t));
  const double m[9] = {1, 0, 0, 0, c, -s, 0, s, c};
  return m[k];
}

static double imu_r1(double t) { return imu_r(t, 0); }
static double imu_r2(double t) { return imu_r(t, 1); }
static double imu_r3(double t) { return imu_r(t, 2); }
static double imu_r4(double t) { return imu_r(t, 3); }
static double imu_r5(double t) { return imu_r(t, 4); }
static double imu_r6(double t) { return imu_r(t, 5); }
static double imu_r7(double t) { return imu_r(t, 6); }
static double imu_r8(double t) { return imu_r(t, 7); }
static double imu_r9(double t) { return imu_r(t, 8); }
static double imu_roll_rate(double t) { return t < IMU_ROLL_START ? 0.0 : IMU_ROLL_RATE; }

#define IMU_MOTION_INPUTS                                                                          \
  FH_INPUT(r_wb_1, FH_FUNCTION(imu_r1)), FH_INPUT(r_wb_2, FH_FUNCTION(imu_r2)),                    \
      FH_INPUT(r_wb_3, FH_FUNCTION(imu_r3)), FH_INPUT(r_wb_4, FH_FUNCTION(imu_r4)),                \
      FH_INPUT(r_wb_5, FH_FUNCTION(imu_r5)), FH_INPUT(r_wb_6, FH_FUNCTION(imu_r6)),                \
      FH_INPUT(r_wb_7, FH_FUNCTION(imu_r7)), FH_INPUT(r_wb_8, FH_FUNCTION(imu_r8)),                \
      FH_INPUT(r_wb_9, FH_FUNCTION(imu_r9)), FH_INPUT(a_w_1, FH_CONSTANT(0.0)),                    \
      FH_INPUT(a_w_2, FH_CONSTANT(0.0)), FH_INPUT(a_w_3, FH_CONSTANT(0.0)),                        \
      FH_INPUT(gravity_w_1, FH_CONSTANT(0.0)), FH_INPUT(gravity_w_2, FH_CONSTANT(0.0)),            \
      FH_INPUT(gravity_w_3, FH_CONSTANT(IMU_G0)), FH_INPUT(omega_b_1, FH_FUNCTION(imu_roll_rate)), \
      FH_INPUT(omega_b_2, FH_CONSTANT(0.0)), FH_INPUT(omega_b_3, FH_CONSTANT(0.0)),                \
      FH_INPUT(alpha_b_1, FH_CONSTANT(0.0)), FH_INPUT(alpha_b_2, FH_CONSTANT(0.0)),                \
      FH_INPUT(alpha_b_3, FH_CONSTANT(0.0))

#endif
