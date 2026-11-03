/* FMI harness: Vehicles.Copter.MultirotorPlant.
 *
 * QuadX plant: 1.5 kg aggregate mass, CG at the chassis reference, starting at NED {0,0,-1} facing East
 * (q_start = yaw +90 deg, so body x = East, body y = South), with four
 * compliant landing legs 0.10 m below the CG over ground z = 0. Rotor i sits
 * at the QuadX arm tip and R_br = diag(1,-1,-1): rotor-local +z thrust is
 * body -z (up) and a rotor-local +z moment is a body -z (yaw-left) moment.
 * Tunable-parameter overrides (world.toml): aggregate.inertia =
 * diag(0.02, 0.03, 0.04) (default Iyy is 0.02; unequal roll/pitch inertia
 * exposes a swapped or transposed inertia) and landingGear.stiffness =
 * 4000 N/m (default 3000; sets the static leg compression).
 *
 * Scenario (rotor forces/moments and external loads drive the plant directly):
 *   0.0-1.0  hover: each rotor f0 = m g / 4; wind 3 m/s North on [0.5,1.0)
 *            (the plant has no drag: wind only shows in rotorAirVelocity_r)
 *   1.0-1.5  collective 1.2 f0: climb at 0.2 g
 *   1.5-2.0  f0 again (coast at constant climb rate); body-x (East) external
 *            force 3 N on [1.5,1.7)
 *   2.0-2.2  yaw doublet: rotor-1 local moment +0.2 then -0.2 N.m (0.1 s each)
 *   2.2-2.4  roll doublet: rotors 1,2 (left) +0.2 N, rotors 3,4 (right) -0.2 N,
 *            then reversed; total thrust unchanged
 *   2.4-2.6  thrust cut (free fall) with an external pitch doublet +-0.1 N.m
 *            (pitch rate 0.1/0.03*0.1 = 0.333 rad/s with the Iyy override)
 *   2.6-5.0  free fall onto the landing gear, touchdown, settling at rest
 * checks.toml establishes hover equilibrium, Newton's law for climb and
 * lateral push, moment arms / rotor mount signs (yaw and roll doublets give
 * closed-form angles), wind transport into rotor axes, free fall under
 * gravity (velocity change g dt), and the static landing-gear compression
 * m g/(4 k) at rest; the landing slide and final heading are cross-checked
 * against native `rumoca sim` of the same profile and parameters (agreement
 * 6.3e-7 m, 7.7e-7 rad). */
#include "fire_harness.h"

#define MASS 1.5
#define G 9.80665
#define F0 (MASS * G / 4.0) /* hover thrust per rotor */
#define EPS 1e-6           /* communication times are not exact multiples */

static int between(double t, double a, double b) { return t >= a - EPS && t < b - EPS; }

/* Collective plus the roll doublet; side = +1 for rotors 1,2, -1 for 3,4. */
static double thrust(double t, double side) {
  if (between(t, 1.0, 1.5))
    return 1.2 * F0;
  if (between(t, 2.2, 2.3))
    return F0 + side * 0.2;
  if (between(t, 2.3, 2.4))
    return F0 - side * 0.2;
  if (t >= 2.4 - EPS)
    return 0.0;
  return F0;
}
static double thrust_left(double t) { return thrust(t, 1.0); }
static double thrust_right(double t) { return thrust(t, -1.0); }

static double yaw_moment_r1(double t) {
  if (between(t, 2.0, 2.1))
    return 0.2;
  if (between(t, 2.1, 2.2))
    return -0.2;
  return 0.0;
}

static double pitch_moment(double t) {
  if (between(t, 2.4, 2.5))
    return 0.1;
  if (between(t, 2.5, 2.6))
    return -0.1;
  return 0.0;
}

static double wind_north(double t) { return between(t, 0.5, 1.0) ? 3.0 : 0.0; }
static double push_east(double t) { return between(t, 1.5, 1.7) ? 3.0 : 0.0; }

/* rotorForce_r / rotorMoment_r are [4,3] row-major: rotor i local z is 3i. */
static const fh_input_t inputs[] = {
    FH_INPUT(gravity_w_1, FH_CONSTANT(0.0)),
    FH_INPUT(gravity_w_2, FH_CONSTANT(0.0)),
    FH_INPUT(gravity_w_3, FH_CONSTANT(G)),
    FH_INPUT(wind_w_1, FH_FUNCTION(wind_north)),
    FH_INPUT(wind_w_2, FH_CONSTANT(0.0)),
    FH_INPUT(wind_w_3, FH_CONSTANT(0.0)),
    FH_INPUT(externalforce_b_1, FH_FUNCTION(push_east)),
    FH_INPUT(externalforce_b_2, FH_CONSTANT(0.0)),
    FH_INPUT(externalforce_b_3, FH_CONSTANT(0.0)),
    FH_INPUT(externalmoment_b_1, FH_CONSTANT(0.0)),
    FH_INPUT(externalmoment_b_2, FH_FUNCTION(pitch_moment)),
    FH_INPUT(externalmoment_b_3, FH_CONSTANT(0.0)),
    FH_INPUT(rotorforce_r_1, FH_CONSTANT(0.0)),
    FH_INPUT(rotorforce_r_2, FH_CONSTANT(0.0)),
    FH_INPUT(rotorforce_r_3, FH_FUNCTION(thrust_left)),
    FH_INPUT(rotorforce_r_4, FH_CONSTANT(0.0)),
    FH_INPUT(rotorforce_r_5, FH_CONSTANT(0.0)),
    FH_INPUT(rotorforce_r_6, FH_FUNCTION(thrust_left)),
    FH_INPUT(rotorforce_r_7, FH_CONSTANT(0.0)),
    FH_INPUT(rotorforce_r_8, FH_CONSTANT(0.0)),
    FH_INPUT(rotorforce_r_9, FH_FUNCTION(thrust_right)),
    FH_INPUT(rotorforce_r_10, FH_CONSTANT(0.0)),
    FH_INPUT(rotorforce_r_11, FH_CONSTANT(0.0)),
    FH_INPUT(rotorforce_r_12, FH_FUNCTION(thrust_right)),
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
};

static const fh_output_t outputs[] = {
    FH_OUTPUT(force_b_3),
    FH_OUTPUT(moment_b_1),
    FH_OUTPUT(moment_b_3),
    FH_OUTPUT(rotorairvelocity_r_2),
};

int main(void) {
  const fh_scenario_t scenario = {
      .title = "Vehicles.Copter.MultirotorPlant",
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
