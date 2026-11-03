/* FMI harness: Physical.Electrical.Drives.Control.SixStep.
 *
 * Ideal-position six-step commutation. The harness plays the rotor
 * (electrical angle) and the current sensor (phase currents); the default
 * currentThreshold = 1e-3 A. The angle starts at -pi + 0.01 (sector 4) and
 * from 0.1 s turns at one electrical revolution per 0.15 s (25 ms per
 * sector) to the end; the 0.01 rad offset keeps samples off sector edges.
 * Phase currents are a balanced set A(t) sin(theta - 2 pi (k-1)/3).
 *
 *   0.00-0.10  command 0, A = 1.5     initial Hi-Z despite winding current
 *   0.10-0.40  command 0.6, A = 2     commutation through 12 sectors
 *   0.40-0.45  command 0, A = 2       cutoff: latch the last low leg
 *   0.45-0.50  A ramps 2 -> 0         recirculation holds until drained
 *   0.50-0.55  command 0, A = 0       Hi-Z (coast)
 *   0.55-0.60  command -0.3           negative command: stays disabled
 *   0.60-0.65  command 1.5            modulation clamps to 1
 *   0.65-0.70  command 0, A = 0       cutoff with no current: Hi-Z at once
 *   0.70-0.75  command 1e-9, A = 2    any positive command enables
 *   0.75-0.80  command -0.5, A = 2    cutoff by a negative jump: recirculate
 *   0.80-0.85  A = 0.0008             below threshold (but nonzero): release
 *   0.85-0.90  A = 0.5                current returns: no re-latch, stays Hi-Z
 */
#include "fire_harness.h"

#include <math.h>

#define PI 3.14159265358979323846

static double angle(double t) {
  const double w = 2.0 * PI / 0.15;
  return -PI + 0.01 + (t < 0.1 ? 0.0 : w * (t - 0.1));
}

static double amplitude(double t) {
  if (t < 0.1)
    return 1.5;
  if (t < 0.45)
    return 2.0;
  if (t < 0.5)
    return 2.0 * (0.5 - t) / 0.05;
  if (t < 0.7)
    return 0.0;
  if (t < 0.8)
    return 2.0;
  if (t < 0.85)
    return 0.0008;
  return 0.5;
}

static double command_profile(double t) {
  if (t < 0.1)
    return 0.0;
  if (t < 0.4)
    return 0.6;
  if (t < 0.55)
    return 0.0;
  if (t < 0.6)
    return -0.3;
  if (t < 0.65)
    return 1.5;
  if (t < 0.7)
    return 0.0;
  if (t < 0.75)
    return 1e-9;
  return -0.5;
}

static double current1(double t) { return amplitude(t) * sin(angle(t)); }
static double current2(double t) { return amplitude(t) * sin(angle(t) - 2.0 * PI / 3.0); }
static double current3(double t) { return amplitude(t) * sin(angle(t) + 2.0 * PI / 3.0); }

static const fh_input_t inputs[] = {
    FH_INPUT(command, FH_FUNCTION(command_profile)),
    FH_INPUT(electricalangle, FH_FUNCTION(angle)),
    FH_INPUT(phasecurrent_1, FH_FUNCTION(current1)),
    FH_INPUT(phasecurrent_2, FH_FUNCTION(current2)),
    FH_INPUT(phasecurrent_3, FH_FUNCTION(current3)),
};

static const fh_output_t outputs[] = {
    FH_OUTPUT(modulation),
    FH_OUTPUT_INT(sector),
    FH_OUTPUT_BOOL(highselected_1),
    FH_OUTPUT_BOOL(highselected_2),
    FH_OUTPUT_BOOL(highselected_3),
    FH_OUTPUT_BOOL(lowselected_1),
    FH_OUTPUT_BOOL(lowselected_2),
    FH_OUTPUT_BOOL(lowselected_3),
    FH_OUTPUT_BOOL(recirculating),
    FH_OUTPUT_BOOL(coast),
};

int main(void) {
  const fh_scenario_t scenario = {
      .title = "Physical.Electrical.Drives.Control.SixStep",
      .stop_s = 0.9,
      .step_s = 0.001,
      .print_s = 0.025,
      .inputs = inputs,
      .input_count = FH_COUNT(inputs),
      .outputs = outputs,
      .output_count = FH_COUNT(outputs),
  };
  return fh_run(&scenario);
}
