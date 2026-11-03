/* FMI harness: Physical.Electrical.Drives.Bridges.SwitchingHalfBridge (through Wrapper.mo).
 *
 * One switched inverter leg (two switches Ron = 10 mOhm, Goff = 1e-5 S, body
 * diodes Rdiode = 20 mOhm, Vknee = 0.7 V) on an ideal 16 V bus, phase loaded
 * by R = 1 Ohm + L = 0.5 mH + back-EMF E. Gates are real 25 kHz PWM patterns
 * (period 40 us) generated on the 1 us communication grid:
 *    0   - 0.4 ms  all off, E = 2                 phase floats at E
 *    0.4 - 5.6 ms  async PWM d = 0.5, low off, E = 2  diode freewheel
 *    5.6 -10.8 ms  sync PWM d = 0.25 (low = complement, 1 us dead time
 *                  each side), E = 2               synchronous rectification
 *   10.8 -16.0 ms  same sync PWM, E = 6            current reverses: the
 *                                                  synchronous leg regenerates
 *   16.0 -20.0 ms  all off, E = 20 > Vdc + Vd      diode regeneration
 * checks.toml establishes: no shoot-through in the initialized state; cycle
 * means equal the averaged-leg closed form (5.5665 A at d = 0.5); ripple peak
 * and peak-to-peak (0.335 A); which device carries the current in each
 * interval (high switch, low body diode in the async off-time and dead
 * times, low switch in reverse during synchronous rectification, high body
 * diode with negative current); the dead-time voltage error and its sign
 * flip with current direction; volt-second balance; regeneration into the
 * bus; diode clamping to the rails. References: an exact piecewise-
 * exponential periodic-steady-state solution and a native 'rumoca sim' of
 * the same wrapper with sampled BooleanPulse gates (they agree to 4e-4 A).
 *
 * KNOWN FAILURE (suspected FMU bug, see the harness report): Boolean inputs
 * set in Step Mode never reach the discrete equations that read them
 * (m.leg.highGate / m.leg.lowGate stay false), so the leg never switches
 * and the t = 0 row shows both switches closed (off = false while
 * control = false). The floating phase before switching also reads
 * 2.0058 V against E + R*i = 2.0002 V with nonzero loadL.v in a DC state.
 * The checks pass against the native reference. */
#include "fire_harness.h"

#include <math.h>

#define PERIOD_US 40L

static long micros(double t) { return lround(t * 1e6); }

static double high_gate(double t) {
  const long us = micros(t), phase = us % PERIOD_US;
  if (us >= 400 && us < 5600)
    return phase < 20 ? 1.0 : 0.0; /* d = 0.5 */
  if (us >= 5600 && us < 16000)
    return phase < 10 ? 1.0 : 0.0; /* d = 0.25 */
  return 0.0;
}

static double low_gate(double t) {
  const long us = micros(t), phase = us % PERIOD_US;
  if (us >= 5600 && us < 16000)
    return (phase >= 11 && phase < PERIOD_US - 1) ? 1.0 : 0.0; /* 1 us dead time each side */
  return 0.0;
}

static double back_emf(double t) {
  const long us = micros(t);
  if (us < 10800)
    return 2.0;
  if (us < 16000)
    return 6.0;
  return 20.0;
}

static const fh_input_t inputs[] = {
    FH_INPUT_BOOL(highgate, FH_FUNCTION(high_gate)),
    FH_INPUT_BOOL(lowgate, FH_FUNCTION(low_gate)),
    FH_INPUT(backemf, FH_FUNCTION(back_emf)),
};

static const fh_output_t outputs[] = {
    FH_OUTPUT(phasecurrent),
    FH_OUTPUT(phasevoltage),
    FH_OUTPUT(dccurrent),
};

int main(void) {
  const fh_scenario_t scenario = {
      .title = "Physical.Electrical.Drives.Bridges.SwitchingHalfBridge",
      .stop_s = 0.02,
      .step_s = 0.000001,
      .print_s = 0.001,
      .inputs = inputs,
      .input_count = FH_COUNT(inputs),
      .outputs = outputs,
      .output_count = FH_COUNT(outputs),
  };
  return fh_run(&scenario);
}
