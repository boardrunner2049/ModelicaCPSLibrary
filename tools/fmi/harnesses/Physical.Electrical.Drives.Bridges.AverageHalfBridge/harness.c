/* FMI harness: Physical.Electrical.Drives.Bridges.AverageHalfBridge (through Wrapper.mo).
 *
 * One averaged inverter leg (Ron = 10 mOhm, Rdiode = 20 mOhm, Vdiode = 0.7 V)
 * on an ideal 16 V bus, phase loaded by R = 1 Ohm + L = 0.5 mH + back-EMF E.
 * Mean high side: v = d*Vdc - (1-d)*Vd - [d*Ron + (1-d)*Rd]*i, i >= 0, and
 * Idc = d*i. Segments (5 ms each, >= 10 load time constants):
 *    0- 1 ms  all off, E = 0                    no current
 *    1- 6 ms  high, d = 0.5, E = 2               motoring
 *    6-11 ms  d ramps to 1 (by 8 ms), E = 2      full duty
 *   11-16 ms  high off, E = 2                    freewheel via low diode,
 *                                               current reaches zero, blocks
 *   16-21 ms  low switch on, E = 6               braking: reverse current
 *   21-26 ms  all off, E = 6                     high diode returns current
 *   26-31 ms  all off, E = 20 > Vdc + Vd         diode regeneration
 *   31-36 ms  high, d = 0.5, E = 20              high side blocked (i >= 0)
 *   36-41 ms  high, d = 0.25, E = 0              light-duty motoring
 * checks.toml compares the closed-form steady states (phase current, phase
 * voltage, bus current), the freewheel decay and extinction time, the
 * high-diode return after braking (decaying current, clamp voltage above
 * dc_p, current returned to the bus, extinction), the bus current
 * direction during regeneration and leg power balance with a native
 * 'rumoca sim' run of the same wrapper (scenario as time functions) and the
 * initialized leakage divider (phase at Vdc/3 with every device off).
 *
 * The Boolean gate inputs and the leakage-held floating phase voltage were
 * Rumoca FMU defects when this harness was written; both are fixed as of
 * Rumoca a4dd197b (event iteration on input changes, stiff Co-Simulation
 * substeps). */
#include "fire_harness.h"

static double duty(double t) {
  if (t < 6e-3 - 1e-12)
    return 0.5;
  if (t < 8e-3 - 1e-12)
    return 0.5 + (t - 6e-3) / 2e-3 * 0.5;
  if (t < 31e-3 - 1e-12)
    return 1.0;
  if (t < 36e-3 - 1e-12)
    return 0.5;
  return 0.25;
}

static double high_selected(double t) {
  return ((t >= 1e-3 - 1e-12 && t < 11e-3 - 1e-12) || t >= 31e-3 - 1e-12) ? 1.0 : 0.0;
}

static double back_emf(double t) {
  if (t < 1e-3 - 1e-12)
    return 0.0;
  if (t < 16e-3 - 1e-12)
    return 2.0;
  if (t < 26e-3 - 1e-12)
    return 6.0;
  if (t < 36e-3 - 1e-12)
    return 20.0;
  return 0.0;
}

static const fh_input_t inputs[] = {
    FH_INPUT(modulation, FH_FUNCTION(duty)),
    FH_INPUT_BOOL(highselected, FH_FUNCTION(high_selected)),
    FH_INPUT_BOOL(lowgate, FH_PULSE(16e-3, 5e-3, 0.0, 1.0)),
    FH_INPUT(backemf, FH_FUNCTION(back_emf)),
};

static const fh_output_t outputs[] = {
    FH_OUTPUT(phasecurrent),
    FH_OUTPUT(phasevoltage),
    FH_OUTPUT(dccurrent),
};

int main(void) {
  const fh_scenario_t scenario = {
      .title = "Physical.Electrical.Drives.Bridges.AverageHalfBridge",
      .stop_s = 0.041,
      .step_s = 0.00001,
      .print_s = 0.001,
      .inputs = inputs,
      .input_count = FH_COUNT(inputs),
      .outputs = outputs,
      .output_count = FH_COUNT(outputs),
  };
  return fh_run(&scenario);
}
