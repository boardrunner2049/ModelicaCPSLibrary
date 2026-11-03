/* FMI harness: Tests.DynamicsFullInertia (closed test model, no inputs).
 *
 * Scenario: a RigidBody6DOF with a full (off-diagonal) inertia driven by a
 * manufactured moment M = J alpha + w x (J w), so its rate must follow the
 * linear w(t) = {0.1+0.2t, 0.2-0.15t, -0.1+0.1t}; 1 s at a 1 ms step.
 * world.toml also overrides p_start, the body-frame v_start and the mass
 * (the in-model asserts constrain only omega_b and alpha_b), so the rotating
 * body translates as well.
 *
 * checks.toml establishes: w(t) at 0.5 s and 1 s (yaw rate crossing zero) and
 * constant alpha, i.e. Euler's equation with full J is inverted correctly;
 * quaternion and all three 3-2-1 Euler angles against an independent RK4 of
 * q' = q (x) (0,w)/2; unit quaternion norm; constant world velocity and the
 * uniform-motion CG path p_start + v_start t although v_b rotates; zero
 * specific force.
 *
 * No outputs exist, so the console prints only time; values are in the trace. */
#include "fire_harness.h"

int main(void) {
  const fh_scenario_t scenario = {
      .title = "Tests.DynamicsFullInertia",
      .stop_s = 1.0,
      .step_s = 0.001,
      .print_s = 0.1,
      .inputs = NULL,
      .input_count = 0,
      .outputs = NULL,
      .output_count = 0,
  };
  return fh_run(&scenario);
}
