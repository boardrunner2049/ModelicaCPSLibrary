/* FMI harness: Tests.MultirotorFrames.
 *
 * A closed test model: a two-rotor MultirotorPlant with one rotor mounted
 * tilted (local thrust axis along body +x), a CG shifted to {0.03,-0.02,0.01}
 * in chassis axes, zero gravity, and constant rotor-local forces and hub
 * moments. Its equation-section asserts require the summed body force and
 * moment about the combined CG to match hand values. There are no inputs.
 *
 * Run length: 1 s at 1 ms. The constant wrench makes the vehicle translate
 * and tumble freely; at 3.63 s the drift brings a landing leg into ground
 * contact, the gear adds a wrench and the model's own asserts fire (native
 * `rumoca sim` agrees, so this is the model's scope, not an FMU fault). The
 * model's own StopTime is 0.1 s; 1 s gives a substantial tumble while staying
 * well inside the contact-free window.
 *
 * checks.toml establishes: the force/moment sums and per-rotor rows (frame
 * mapping, CG rebasing, hub moment added once), Newton's law in world axes,
 * |a| = |F|/m under rotation, Euler's rotational equation at every sample,
 * the initial angular acceleration I^-1 M, and that the aggregate inertia
 * passes through the mass-property function unchanged. The integrated state
 * is pinned two ways: each state's change over [0,1] s equals the Simpson
 * integral of its traced derivative (p from v_w, v_w from a_w, omega from
 * alpha, q from 1/2 q x omega), which is inertia-independent; and the 1 s
 * state is compared with ref_traj.py (this directory), an independent scipy
 * integration with the intended inertia. The inertia, I^-1 M and endpoint
 * checks fail under Rumoca bug A (products of inertia about the chassis
 * origin); ref_traj.py with the bug-A inertia reproduces the FMU to 1e-11. */
#include "fire_harness.h"

int main(void) {
  const fh_scenario_t scenario = {
      .title = "Tests.MultirotorFrames",
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
