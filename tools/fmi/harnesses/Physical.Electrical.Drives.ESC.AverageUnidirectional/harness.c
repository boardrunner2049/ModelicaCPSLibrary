/* FMI harness: Physical.Electrical.Drives.ESC.AverageUnidirectional
 * (through Wrapper.mo: the ESC drives a DynamicDCEquivalent motor, R 0.2,
 * L 1 mH, k 0.02, J_total 2e-4, b 1e-5, with quadratic drag 1.5e-7 w^2,
 * wired as documented: openCircuitVoltage = back-EMF, coast -> currentZero).
 *
 *   0.00-0.05  command 0, 16 V        rotor at rest: coast, nothing flows
 *   0.05-0.50  command 0.5            8 V forward conduction, spin-up to the
 *                                     loaded equilibrium (~388 rad/s)
 *   0.50-1.00  bus sags to 14 V       7 V < back-EMF: no regeneration; the
 *                                     current hits zero, the drive opens
 *                                     (coast) and the rotor drags down until
 *                                     k w < 7 V, then conduction resumes
 *   1.00-1.20  command 0              cutoff: freewheel (0 V) until the
 *                                     current reaches zero, then coast
 *   1.20-1.40  command 1.4 at 16 V    over-range command clamps to 1: restart
 *   1.40-1.50  bus 0 V, command 1.4   dead bus: freewheel then coast, no
 *                                     supply current
 *   1.50-1.60  bus 16 V, command -0.3 negative command clamps to 0: coast
 *
 * dcVoltage >= 0 is asserted by the ESC; a negative bus must fail the step
 * (exercised outside this scenario). Communication step 0.5 ms. */
#include "fire_harness.h"

static double command_profile(double t) {
  if (t < 0.05)
    return 0.0;
  if (t < 1.0)
    return 0.5;
  if (t < 1.2)
    return 0.0;
  if (t < 1.5)
    return 1.4;
  return -0.3;
}

static double bus_profile(double t) {
  if (t < 0.5)
    return 16.0;
  if (t < 1.0)
    return 14.0;
  if (t < 1.4)
    return 16.0;
  if (t < 1.5)
    return 0.0;
  return 16.0;
}

static const fh_input_t inputs[] = {
    FH_INPUT(esccommand, FH_FUNCTION(command_profile)),
    FH_INPUT(dcvoltage, FH_FUNCTION(bus_profile)),
};

static const fh_output_t outputs[] = {
    FH_OUTPUT(omega),
    FH_OUTPUT(current),
    FH_OUTPUT(backemf),
    FH_OUTPUT(storedenergy),
};

int main(void) {
  const fh_scenario_t scenario = {
      .title = "Physical.Electrical.Drives.ESC.AverageUnidirectional",
      .stop_s = 1.6,
      .step_s = 0.0005,
      .print_s = 0.05,
      .inputs = inputs,
      .input_count = FH_COUNT(inputs),
      .outputs = outputs,
      .output_count = FH_COUNT(outputs),
  };
  return fh_run(&scenario);
}
