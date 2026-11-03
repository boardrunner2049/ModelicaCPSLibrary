/* FMI harness: Examples.OctoHover.
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
 * Scenario (OctoX, 8 arms radius 0.35 m at 22.5 deg + k 45 deg), overrides:
 *   vehicle.aggregate.mass = 1.8   hover speed 469.73 rad/s, thrust m g/8
 *   geometry.spinSign = {1,1,-1,-1,1,1,-1,-1}  paired layout (still cancels)
 *   vehicle.omega_start = {0,0,-0.4}  initial yaw rate
 * Moments cancel, so the heavier vehicle holds its altitude while yawing at a
 * constant -0.4 rad/s for 10 s (heading pi/2 - 0.4 t); every hub moves on a
 * circle at 0.14 m/s.
 * checks.toml establishes: the mass override reaches the rigid body; per-rotor
 * hover speed, ESC command, thrust and drag torque at the new mass; the
 * blades' spinSign and every rotor's body moment follow the spin override;
 * zero net moment; constant yaw rate and heading closed form; altitude and
 * position hold; hub velocity omega x r_i (components for rotors 1, 3 and
 * the 0.14 m/s speed for all 8); IMU g with no centripetal term; gyro z =
 * 0.4; magnetometer {Be sin psi, -Be cos psi, -Bd} along the whole rotation.
 * Native `rumoca sim` of the same parameters agrees with the trace to < 1e-9.
 */
#include "fire_harness.h"

int main(void) {
  const fh_scenario_t scenario = {
      .title = "Examples.OctoHover",
      .stop_s = 10.0,
      .step_s = 0.005,
      .print_s = 0.0,
      .inputs = NULL,
      .input_count = 0,
      .outputs = NULL,
      .output_count = 0,
  };
  return fh_run(&scenario);
}
