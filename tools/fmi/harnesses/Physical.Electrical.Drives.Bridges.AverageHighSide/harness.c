/* FMI harness: Physical.Electrical.Drives.Bridges.AverageHighSide.
 *
 * Wrapper.mo puts the averaged high-side leg (Ron 10 mOhm, Rdiode 20 mOhm,
 * Vdiode 0.5 V) on an ideal 16 V bus whose negative rail dc_n is held 1 V
 * below ground, feeding a 0.5 Ohm + 1 mH load with a 10 Ohm bleed to ground.
 * The leg's mean voltage is Vm = d*Vdc - (1-d)*Vdiode (relative to dc_n)
 * behind Rm = d*Ron + (1-d)*Rdiode, conducting only forward current. A second
 * leg with enabled = false is wired in parallel and must stay dead.
 *
 * Duty profile d(t):
 *   [0, 10) ms     0     leg blocked: Vm = -0.5 V, nothing flows
 *   [10, 30) ms    0.5   CCM step response, L/R time constant ~1.94 ms
 *   [30, 50) ms    1.0   full on (no freewheel share, no dc_n current)
 *   [50, 70) ms    0.2   low duty, mostly diode freewheel loss
 *   [70, 90) ms    0     pure diode freewheel: phase clamped one diode drop
 *                        below dc_n (-1.5 V), current decays until the leg
 *                        blocks (iL < 1.5 V/Rbleed), then the bleed empties
 *                        the inductor
 *   [90, 120) ms   ramp 0 -> 0.6: blocked until Vm > 1 V (d > 0.0909,
 *                  94.55 ms), then conducts
 *   [120, 140) ms  0.6
 * checks.toml compares each plateau with the closed-form Thevenin steady
 * state, the d = 0.5 rise and d = 0 freewheel decay with their exponentials,
 * and checks blocking (leg current never negative) and its release at the
 * dc_n-referenced threshold (both sides), KCL across the three pins and the
 * return through the negative rail, the dc_n share (1-d)*i, the bus power
 * d*Vdc*i and the conduction loss (1-d)*Vdiode*i + Rm*i^2 from all three
 * pin potentials (power conservation through the leg), absolute vs
 * dc_n-referenced phase voltage, and that the disabled leg carries no
 * current on any pin while forward-biased. */
#include "fire_harness.h"

static double duty_profile(double t) {
  if (t < 0.010)
    return 0.0;
  if (t < 0.030)
    return 0.5;
  if (t < 0.050)
    return 1.0;
  if (t < 0.070)
    return 0.2;
  if (t < 0.090)
    return 0.0;
  if (t < 0.120)
    return 0.6 * (t - 0.090) / 0.030;
  return 0.6;
}

static const fh_input_t inputs[] = {
    FH_INPUT(modulation, FH_FUNCTION(duty_profile)),
};

static const fh_output_t outputs[] = {
    FH_OUTPUT(legcurrent),  FH_OUTPUT(loadcurrent), FH_OUTPUT(phasevoltage),
    FH_OUTPUT(buspower),    FH_OUTPUT(bridgeloss),  FH_OUTPUT(dcnegvoltage),
    FH_OUTPUT(idlecurrent), FH_OUTPUT(idleforwardbias),
};

int main(void) {
  const fh_scenario_t scenario = {
      .title = "Physical.Electrical.Drives.Bridges.AverageHighSide",
      .stop_s = 0.14,
      .step_s = 0.00001,
      .print_s = 0.01,
      .inputs = inputs,
      .input_count = FH_COUNT(inputs),
      .outputs = outputs,
      .output_count = FH_COUNT(outputs),
  };
  return fh_run(&scenario);
}
