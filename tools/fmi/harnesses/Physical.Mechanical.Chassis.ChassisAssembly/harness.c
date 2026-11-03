/* FMI harness: Physical.Mechanical.Chassis.ChassisAssembly (through
 * Wrapper.mo, which instantiates it twice with fixed part lists).
 *
 * The chassis is a pure parameter computation (no states, no inputs), so the
 * scenario is a short run that only lets the trace record the initialized
 * values. world.toml overrides the payload mass from 0.35 kg (Wrapper.mo) to
 * 0.5 kg before initialization, so the checks also establish that a tunable
 * part parameter set by the master propagates into the combined properties.
 *
 * checks.toml compares the assembled mass, centre of mass and full inertia
 * tensor about the CG (rotated part tensors plus parallel-axis terms) with an
 * independent numpy evaluation of the parts list, checks symmetry and
 * positive-definiteness/triangle inequalities of the result, and checks that
 * aggregate mode reports the aggregate budget unchanged and ignores the
 * constituent records. */
#include "fire_harness.h"

int main(void) {
  const fh_scenario_t scenario = {
      .title = "Physical.Mechanical.Chassis.ChassisAssembly",
      .stop_s = 0.01,
      .step_s = 0.001,
      .print_s = 0,
      .inputs = NULL,
      .input_count = 0,
      .outputs = NULL,
      .output_count = 0,
  };
  return fh_run(&scenario);
}
