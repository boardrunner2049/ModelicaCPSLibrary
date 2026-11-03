/* FMI harness: Examples.QuadHover.
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
 * Scenario (QuadX, 4 arms radius 0.25 m at 45 deg + k 90 deg, spinSign
 * {1,-1,1,-1}), overrides:
 *   rotors[1].kT = 1.25e-5  rotor 1 hovers slower (542.40 rad/s) for the same
 *                           thrust m g/4, so its drag torque is 20% smaller and
 *                           the yaw reactions no longer cancel: Mz = -0.0110 N.m
 *   motors[2].omegaMax = 1250  ESC 2 must command 606.42/1250
 * The vehicle stays level and in place but spins up about z at the constant
 * alpha = Mz/Izz = -0.2758 rad/s^2 for 4 s (heading turns by -2.2 rad).
 * checks.toml establishes: per-rotor speed, ESC command, thrust and drag
 * torque; every rotor's body moment r_i x (0,0,-T) + (0,0,spinSign_i Q_i) (so
 * a wrong arm position or spin sign on any rotor fails); the net moment; yaw
 * rate a t and heading pi/2 + a t^2/2; altitude/position hold; hub velocity
 * omega x r in rotor axes; IMU specific force g with no centripetal term at
 * the CG; gyro z = -yaw rate; magnetometer {Be sin psi, -Be cos psi, -Bd}
 * along the whole rotation (exercising the sensor y flip). Native `rumoca
 * sim` of the same parameters agrees with the trace to < 1e-8.
 */
#include "fire_harness.h"

int main(void) {
  const fh_scenario_t scenario = {
      .title = "Examples.QuadHover",
      .stop_s = 4.0,
      .step_s = 0.005,
      .print_s = 0.0,
      .inputs = NULL,
      .input_count = 0,
      .outputs = NULL,
      .output_count = 0,
  };
  return fh_run(&scenario);
}
