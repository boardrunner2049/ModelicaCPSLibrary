/* FMI harness: Examples.HexaHover.
 *
 * Examples.QuadHover (which the other *Hover examples extend) is a closed
 * open-loop model: MultirotorWithSensors (aggregate mass 1.5 kg, inertia
 * diag(0.02,0.02,0.04), start NED {0,0,-1} facing East, legs 0.1 m below the
 * CG, ground at z = 0), FirstOrderSpeed motors (tau 0.03 s, omegaMax 1000)
 * started at and commanded through PassThrough ESCs to the per-rotor hover
 * speed sqrt(m g/(n kT_i)), QuadraticBlade rotors (kT 1e-5, kQ 1.5e-7). It
 * has no inputs. At its defaults every quantity is a constant, so this
 * harness perturbs it through tunable parameters set before initialization
 * ([Models.m.Parameters] in world.toml); the hover speed, motor start speed
 * and ESC command all follow from the overridden parameters through
 * calculated parameters, so they must be recomputed for the run to start in
 * balance. (Motor tau is never exercised: motors start at their commanded
 * speed by construction, whatever the parameters.)
 *
 * Scenario (HexaX, 6 arms radius 0.30 m at 30 deg + k 60 deg), override:
 *   geometry.armMount  arm 1 moved out to radius 0.31 m (same direction),
 *                      a non-symmetric frame; arms 2-6 at the preset values
 * Each rotor still carries m g/6, so the longer arm leaves a moment
 * 0.01 m * T about the horizontal axis (0.5, 0.866, 0): the vehicle tips about
 * that fixed body axis (Ixx = Iyy) at alpha = 1.2258 rad/s^2, 0.61 rad after
 * 1 s, while the tilted thrust accelerates it South-West and down.
 * checks.toml establishes: every rotor's body moment with the moved arm 1;
 * net moment = dr x F; alpha_b and omega_b = alpha t; the body z axis in NED
 * (R_wb[:,3]) at 0.5 s and 1 s against the closed-form rotation; Newton's law
 * a_w = g D - g R_wb[:,3] at every sample, and velocity/position at 1 s from
 * quadrature of that closed form; IMU specific force stays g on sensor z
 * (gravity does not leak into x/y under tilt); gyro x = p, y = -q; the
 * magnetometer equals diag(1,-1,-1) R_wb^T B_w at every sample and the
 * closed-form tilted value at 1 s; barometer altitude = -p_w[3]. Native
 * `rumoca sim` of the same armMount agrees with the trace to < 1e-9.
 */
#include "fire_harness.h"

int main(void) {
  const fh_scenario_t scenario = {
      .title = "Examples.HexaHover",
      .stop_s = 1.0,
      .step_s = 0.005,
      .print_s = 0.0,
      .inputs = NULL,
      .input_count = 0,
      .outputs = NULL,
      .output_count = 0,
  };
  return fh_run(&scenario);
}
