/* FMI harness: Examples.CoaxialHover.
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
 * Scenario (CoaxialX8: 4 QuadX arms radius 0.25 m, rotors 1-4 upper at
 * z = -0.025 m, 5-8 lower at +0.025 m, spinSign {1,-1,1,-1,-1,1,-1,1}),
 * override:
 *   vehicle.omega_start = {0.2,0,0}  initial roll rate
 * Moments cancel and x is principal, so the vehicle rolls at a constant
 * 0.2 rad/s for 2 s (0.4 rad) and the thrust vector, tipping toward South,
 * gives closed-form velocity and position.
 * checks.toml establishes: per-rotor speed, thrust, drag torque; every rotor's
 * body moment (each coaxial pair cancels in yaw); zero net moment; constant
 * roll rate and roll angle p t; closed-form v_w and p_w at 1 s and 2 s; the
 * upper/lower hub velocities differing by omega x (0,0,-0.05) at every
 * sample, the only observable of the +-0.025 m rotor heights (thrust is along
 * z, so the heights drop out of the moments); IMU g on sensor z; gyro x = p;
 * the magnetometer = diag(1,-1,-1) R_wb^T B_w at every sample and the closed
 * form at 2 s. Native `rumoca sim` of the same override agrees to < 1e-9.
 */
#include "fire_harness.h"

int main(void) {
  const fh_scenario_t scenario = {
      .title = "Examples.CoaxialHover",
      .stop_s = 2.0,
      .step_s = 0.005,
      .print_s = 0.0,
      .inputs = NULL,
      .input_count = 0,
      .outputs = NULL,
      .output_count = 0,
  };
  return fh_run(&scenario);
}
