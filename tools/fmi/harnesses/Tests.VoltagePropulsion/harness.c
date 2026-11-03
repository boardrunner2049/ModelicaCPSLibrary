/* FMI harness: Tests.VoltagePropulsion.
 *
 * Closed bench (no inputs). P0 path: PassThrough(0.4) -> FirstOrderSpeed
 * (tau 30 ms, omegaMax 1000). P1 path: AverageVoltage(0.4 x 16 V = 6.4 V)
 * -> FirstOrderVoltageToSpeed (Kv 62.5, tau 30 ms): both target 400 rad/s
 * from omega_start = 70 rad/s and must agree. A second P1 motor sees its
 * supply sag to 8 V at 0.15 s (target 200 rad/s); a third has a dead bus
 * (command 1, 0 V) and coasts down from 70 rad/s. Two QuadraticBlades
 * (kT 1e-5, kQ 1.5e-7) on the P0 and nominal P1 speeds must give the same
 * wrench. The model's asserts (closed forms within 1e-4, P0/P1 equivalence
 * within 1e-7) run inside the FMU; checks.toml re-checks them from the
 * trace (native rumoca sim agrees to 5e-7 rad/s). */
#include "fire_harness.h"

int main(void) {
  const fh_scenario_t scenario = {
      .title = "Tests.VoltagePropulsion",
      .stop_s = 0.5,
      .step_s = 0.001,
      .print_s = 0.05,
      .inputs = NULL,
      .input_count = 0,
      .outputs = NULL,
      .output_count = 0,
  };
  return fh_run(&scenario);
}
