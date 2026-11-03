/* FMI harness: Tests.DynamicsFreefall (closed test model, no inputs).
 *
 * Scenario: a RigidBody6DOF with zero applied force and moment under NED
 * gravity 9.81 m/s^2, run for 2 s (twice the experiment StopTime) at a 1 ms
 * step. world.toml perturbs the test through tunable parameters: mass 7.3 kg
 * (instead of 2), a full asymmetric inertia and omega_start = {2,-1,3} rad/s,
 * so the body tumbles while it falls. The model's own 1e-6 asserts on the
 * ballistic path therefore also cover a rotating body.
 *
 * checks.toml establishes: the ballistic closed form p(t), v(t) at 1 s and
 * 2 s, independent of mass; conserved horizontal velocity while the body-frame
 * velocity rotates (the -omega x v_b transport term); a_w = g and zero CG
 * specific force; torque-free invariants (kinetic energy, world angular
 * momentum, unit quaternion); and attitude/rates against an independent RK4.
 *
 * The model has no outputs, so the console prints only time; all values are
 * observed through the trace. */
#include "fire_harness.h"

int main(void) {
  const fh_scenario_t scenario = {
      .title = "Tests.DynamicsFreefall",
      .stop_s = 2.0,
      .step_s = 0.001,
      .print_s = 0.25,
      .inputs = NULL,
      .input_count = 0,
      .outputs = NULL,
      .output_count = 0,
  };
  return fh_run(&scenario);
}
