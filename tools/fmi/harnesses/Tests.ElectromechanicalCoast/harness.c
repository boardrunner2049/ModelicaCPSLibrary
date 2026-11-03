/* FMI harness: Tests.ElectromechanicalCoast.
 *
 * Closed bench (no inputs). An AverageUnidirectional ESC on a 16 V bus
 * drives a DynamicDCEquivalent motor (R 0.2, L 1 mH, k 0.02, J+JLoad 2e-4)
 * with a quadratic blade load kQ = 1.5e-7, its coast output wired to the
 * motor's currentZero. Command schedule (model time function):
 *   0.0-0.1 0.515 (8.24 V, i = 1.2 A, 400 rad/s equilibrium start)
 *   0.1-0.5 0     cutoff: freewheel until the current reaches zero, then
 *                 open-circuit coast (terminal = back-EMF) with a quadratic
 *                 drag coast-down
 *   0.5-0.6 0.1   blocked command: 1.6 V < back-EMF, stays in coast
 *   0.6-0.9 0.8   restart: forward conduction
 *   0.9-1.1 0     second cutoff and coast
 * A second, unpowered drive (dcVoltage 0) sits on a viscous-only motor
 * spinning down from 200 rad/s with zero current.
 *
 * The model's own asserts (current continuity, energy accounting, coast
 * solution, hub reaction bookkeeping) run inside the FMU: any violation
 * fails the step. checks.toml re-derives the same physics from the trace
 * and also compares against a native `rumoca sim` run (dt 0.5 ms), which
 * agrees to < 1e-7 on current, speed and energies. The 0.5 ms
 * communication step matches the model's assert windows. */
#include "fire_harness.h"

int main(void) {
  const fh_scenario_t scenario = {
      .title = "Tests.ElectromechanicalCoast",
      .stop_s = 1.1,
      .step_s = 0.0005,
      .print_s = 0.1,
      .inputs = NULL,
      .input_count = 0,
      .outputs = NULL,
      .output_count = 0,
  };
  return fh_run(&scenario);
}
