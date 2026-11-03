/* FMI harness: Physical.Electrical.Drives.ESC.AverageThreePhase (through Wrapper.mo).
 *
 * Averaged six-step ESC (Ron = Rdiode = 5 mOhm, Vdiode = 0.5 V) on an ideal
 * 16 V bus driving a floating-star load (per phase R = 0.2 Ohm, L = 0.1 mH,
 * EMF e_j). The electrical angle is an input, so commutation is exercised
 * sector by sector without a rotor model. Scenario (10 us step):
 *    0 -  2 ms  command 0, angle 0.1, i0 = 5 A      Hi-Z coast: the initial
 *               current is returned to the bus through the body diodes
 *    2 - 32 ms  command 0.5, angle stepped through all six sectors
 *               (0.1 + k*pi/3, 5 ms each)          each (high, low) pair in turn
 *   32 - 37 ms  angle -2.0                          negative angle wraps to sector 5
 *   37 - 42 ms  command 1.2                         modulation clamps to 1
 *   42 - 47 ms  command -0.3                        cutoff: latched low-side
 *               recirculation until the currents drain (1.71 ms), then Hi-Z
 *   47 - 52 ms  command 0, e = {0, 12, -9} V        diode rectification into the bus
 *   52 - 57 ms  command 0.5, angle -2.0 (sector 5: high 2, low 1),
 *               e = {1.5, -1.0, 0.5} V              asymmetric EMF: I = 25.0 A
 *   57 - 62 ms  angle 0.1 + pi/3 (sector 2: high 1, low 2), same EMF
 *                                                   reversed pair: I = 12.805 A
 * The run starts with a 5 A winding current (wrapper parameter i0 = 5,
 * overridden in world.toml) at zero command: the documented SixStep
 * behaviour is to start in Hi-Z (no latched recirculation), so the current
 * is driven to zero through the body diodes against the bus in ~56 us.
 * checks.toml establishes, against closed forms confirmed by a native
 * 'rumoca sim' of the same wrapper: the sector -> (high, low) phase table
 * (every phase appears as source, sink and floating, so a transposed phase
 * or table would fail), the averaged steady currents 18.90 A (d = 0.5) and
 * 39.02 A (d = 1), bus current d*I, load and ESC power balance, the
 * command clamp, the recirculation latch and its closed-form extinction
 * time, Hi-Z leakage-divider star voltage, regeneration through the body
 * diodes with energy returned to the bus, the Hi-Z start with an initial
 * winding current, sector currents that differ in magnitude under an
 * asymmetric EMF (25.0 vs 12.8 A for the same pair in the two orders), and
 * ESC passivity with its peak loss bounded by the cutoff closed form.
 *
 * BLOCKED: export is refused ("FMI C profile: the C profile cannot execute an
 * initial event yet", from SixStep's when {initial(), ...}); world.toml was
 * written by hand. The checks pass 78/78 on the native trace. */
#include "fire_harness.h"

#include <math.h>

static double command(double t) {
  if (t < 2e-3 - 1e-12)
    return 0.0;
  if (t < 37e-3 - 1e-12)
    return 0.5;
  if (t < 42e-3 - 1e-12)
    return 1.2;
  if (t < 47e-3 - 1e-12)
    return -0.3;
  if (t < 52e-3 - 1e-12)
    return 0.0;
  return 0.5;
}

static double angle(double t) {
  const double sixth = 3.14159265358979323846 / 3.0;
  if (t < 7e-3 - 1e-12)
    return 0.1;
  if (t < 32e-3 - 1e-12)
    return 0.1 + sixth * floor((t - 2e-3 + 1e-12) / 5e-3);
  if (t < 57e-3 - 1e-12)
    return -2.0;
  return 0.1 + sixth;
}

/* Per-phase EMF: zero, then {0, 12, -9} for rectification, then the
 * asymmetric {1.5, -1.0, 0.5} for the last two motoring segments. */
static double emf_value(double t, double rect, double asym) {
  if (t < 47e-3 - 1e-12)
    return 0.0;
  if (t < 52e-3 - 1e-12)
    return rect;
  return asym;
}
static double emf1(double t) { return emf_value(t, 0.0, 1.5); }
static double emf2(double t) { return emf_value(t, 12.0, -1.0); }
static double emf3(double t) { return emf_value(t, -9.0, 0.5); }

static const fh_input_t inputs[] = {
    FH_INPUT(esccommand, FH_FUNCTION(command)),
    FH_INPUT(electricalangle, FH_FUNCTION(angle)),
    FH_INPUT(emf_1, FH_FUNCTION(emf1)),
    FH_INPUT(emf_2, FH_FUNCTION(emf2)),
    FH_INPUT(emf_3, FH_FUNCTION(emf3)),
};

static const fh_output_t outputs[] = {
    FH_OUTPUT(phasecurrent_1), FH_OUTPUT(phasecurrent_2), FH_OUTPUT(phasecurrent_3),
    FH_OUTPUT(dccurrent),      FH_OUTPUT_INT(sector),     FH_OUTPUT_BOOL(coast),
};

int main(void) {
  const fh_scenario_t scenario = {
      .title = "Physical.Electrical.Drives.ESC.AverageThreePhase",
      .stop_s = 0.062,
      .step_s = 0.00001,
      .print_s = 0.001,
      .inputs = inputs,
      .input_count = FH_COUNT(inputs),
      .outputs = outputs,
      .output_count = FH_COUNT(outputs),
  };
  return fh_run(&scenario);
}
