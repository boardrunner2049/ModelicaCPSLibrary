/* FMI harness: Physical.Mechanical.Dynamics.RigidBody6DOF.
 *
 * m = 2 kg, I = [0.1 0.02 0; 0.02 0.2 0; 0 0 0.3] kg m^2 (body z principal),
 * start: p = (1,2,-50) NED, v_b = (1,0.5,-2), q_start = 2 x the unit
 * quaternion of ZYX (yaw 0.4, pitch -0.2, roll 0.3) (normalized at init),
 * omega = 0. Phases:
 *   0-1 s  ballistic: gravity (0,0,9.80665), no force/moment. Attitude
 *          fixed, p = p0 + R0 v_b0 t + g t^2/2, accelerometer reads zero.
 *   1-2 s  gravity off, body moment (0,0,0.3) about the principal z axis:
 *          omega_z = t-1, yaw-about-body-z psi = (t-1)^2/2, R = R0 Rz(psi);
 *          the world velocity stays constant (no force).
 *   2-3 s  moment off (spin 1 rad/s about body z), body force (2,0,0): the
 *          thrust rotates with the body, v_w = v_w(2) + R0 (sin psi -
 *          sin psi2, cos psi2 - cos psi, 0), psi = 0.5 + (t-2).
 *   3-3.1  force off, body moment (0.1,0.05,0) pulse: tumble.
 *   3.1-5  torque- and force-free tumble with asymmetric inertia: world
 *          angular momentum R I omega and rotational energy are conserved,
 *          v_w constant, quaternion stays unit.
 * The closed forms are evaluated in checks.toml (numbers from Python). */
#include "fire_harness.h"

static double g3(double t) { return t < 1.0 ? 9.80665 : 0.0; }
static double f1(double t) { return (t >= 2.0 && t < 3.0) ? 2.0 : 0.0; }
static double m1(double t) { return (t >= 3.0 && t < 3.1) ? 0.1 : 0.0; }
static double m2(double t) { return (t >= 3.0 && t < 3.1) ? 0.05 : 0.0; }
static double m3(double t) { return (t >= 1.0 && t < 2.0) ? 0.3 : 0.0; }

static const fh_input_t inputs[] = {
    FH_INPUT(force_b_1, FH_FUNCTION(f1)),
    FH_INPUT(force_b_2, FH_CONSTANT(0.0)),
    FH_INPUT(force_b_3, FH_CONSTANT(0.0)),
    FH_INPUT(moment_b_1, FH_FUNCTION(m1)),
    FH_INPUT(moment_b_2, FH_FUNCTION(m2)),
    FH_INPUT(moment_b_3, FH_FUNCTION(m3)),
    FH_INPUT(gravity_w_1, FH_CONSTANT(0.0)),
    FH_INPUT(gravity_w_2, FH_CONSTANT(0.0)),
    FH_INPUT(gravity_w_3, FH_FUNCTION(g3)),
};

static const fh_output_t outputs[] = {
    FH_OUTPUT(p_w_1), FH_OUTPUT(p_w_3), FH_OUTPUT(v_w_1), FH_OUTPUT(v_w_3),
    FH_OUTPUT(omega_b_1), FH_OUTPUT(omega_b_3), FH_OUTPUT(euler_3),
};

int main(void) {
  const fh_scenario_t scenario = {
      .title = "Physical.Mechanical.Dynamics.RigidBody6DOF",
      .stop_s = 5.0,
      .step_s = 0.001,
      .print_s = 0.25,
      .inputs = inputs,
      .input_count = FH_COUNT(inputs),
      .outputs = outputs,
      .output_count = FH_COUNT(outputs),
  };
  return fh_run(&scenario);
}
