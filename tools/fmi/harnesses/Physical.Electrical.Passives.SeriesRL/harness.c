/* FMI harness: Physical.Electrical.Passives.SeriesRL.
 *
 * Wrapper.mo puts two SeriesRL instances in series across an ideal voltage
 * source: rl (2 Ohm, 10 mH, fixed signed initial current -1 A) and rl2
 * (1 Ohm, 20 mH, fixedCurrentStart = false with an inconsistent start guess
 * of 0.7 A). Loop time constant tau = 30 mH / 3 Ohm = 10 ms.
 *
 * Voltage profile: 0 V for 20 ms (the -1 A initial current decays),
 * +10 V to 120 ms (rise to +3.333 A), -5 V to 220 ms (current reverses
 * through zero at 130.99 ms toward -1.667 A), then 0 V (free decay) to 300 ms.
 * checks.toml establishes: the fixed signed start and the dependent start of
 * the unfixed piece (taken from the network, not from i_start); the
 * closed-form first-order response at tau multiples and around the zero
 * crossing; no overshoot beyond the V/R extremes; each piece's
 * v = R*i + L*di/dt through the current-dependent voltage split
 * (rl.v = i + V/3, possible only because the pieces' L/R differ); reported
 * copper losses and magnetic energies at the plateau, steady-state power
 * balance V*i = sum of losses; and the integrated energy balance source =
 * integral of losses + change of stored energy at every step. */
#include "fire_harness.h"

static double voltage_profile(double t) {
  if (t < 0.020)
    return 0.0;
  if (t < 0.120)
    return 10.0;
  if (t < 0.220)
    return -5.0;
  return 0.0;
}

static const fh_input_t inputs[] = {
    FH_INPUT(voltage, FH_FUNCTION(voltage_profile)),
};

static const fh_output_t outputs[] = {
    FH_OUTPUT(current),
    FH_OUTPUT(terminalvoltage),
    FH_OUTPUT(energyerror),
};

int main(void) {
  const fh_scenario_t scenario = {
      .title = "Physical.Electrical.Passives.SeriesRL",
      .stop_s = 0.3,
      .step_s = 0.0001,
      .print_s = 0.005,
      .inputs = inputs,
      .input_count = FH_COUNT(inputs),
      .outputs = outputs,
      .output_count = FH_COUNT(outputs),
  };
  return fh_run(&scenario);
}
