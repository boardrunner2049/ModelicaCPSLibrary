/* FMI harness: Tests.MultirotorPayload.
 *
 * A closed test model: a MultirotorWithSensors vehicle in assembled-mass mode
 * (2 kg core with unit inertia, massless arms, a 1 kg payload at
 * {0.3,0,-0.15}) with zero gravity and zero rotor wrenches, plus
 * initial-equation asserts that the payload shifts the CG to {0.1,0,-0.05}
 * and that every rotor, leg and sensor mount is rebased to it. The model
 * itself only sits still; world.toml perturbs the closed example with
 * omega_start = (0.3,-0.5,0.8) rad/s and v_start = (0.2,0.1,0) m/s so that
 * the payload-shifted full inertia tensor drives a visible torque-free
 * tumble (1 s, 1 ms steps; the gear stays 1 m above ground).
 *
 * checks.toml establishes: mass 3 kg, CG, the full tensor about the CG, the
 * rebasing of all four rotor mounts and four legs (against literal values
 * from the library's default layout) and three sensors, uniform
 * translation, and conservation of rotational energy and world angular
 * momentum (using the FMU's own inertia, so a dynamics error shows
 * independently of the mass-property computation).
 *
 * Current status: initialization fails on the model's own "Payload full
 * inertia mismatch" assertion (also under native `rumoca sim`): the product
 * of inertia I_xz is computed about the chassis origin (suspected Rumoca
 * bug, see the report). diag/ runs this harness and checks.toml on an
 * assert-free copy of the model (diag/NoAssert.mo): 15/17 pass, failing only
 * the two checks that depend on the bug-A inertia. */
#include "fire_harness.h"

int main(void) {
  const fh_scenario_t scenario = {
      .title = "Tests.MultirotorPayload",
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
