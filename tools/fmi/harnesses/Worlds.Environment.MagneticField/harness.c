/* FMI harness: Worlds.Environment.MagneticField.
 *
 * Constant world-frame (NED) magnetic field source, run with the shipped default
 * magneticField0 = {0, 21e-6, -43e-6} T, so the checks pin what users of the
 * library actually get: exactly that vector at every sample (constant in time),
 * which equals the ENU->NED migration W = [0 1 0; 1 0 0; 0 0 -1] of
 * docs/architecture.md applied to the legacy ENU value {21e-6, 0, 43e-6}.
 * The native `rumoca sim` of the class gives the same vector.
 * Note (reported as a library data issue, not hidden): in NED this default points
 * magnetic East and Up (no North component, upward dip). The legacy ENU/FLU value
 * {21e-6, 0, 43e-6} is already East 21 uT / Up 43 uT, so the migration W was applied
 * faithfully and the implausible direction predates it; its origin is uncertain.
 * That magneticField0 is a tunable parameter that reaches the output (not folded)
 * was confirmed separately: the same FMU with magneticField0 = (20,-4,47) uT in
 * [Models.m.Parameters] outputs exactly that vector. This harness keeps the default
 * because pinning the shipped value is the more useful regression check. */
#include "fire_harness.h"

static const fh_output_t outputs[] = {
    FH_OUTPUT(magneticfield_1),
    FH_OUTPUT(magneticfield_2),
    FH_OUTPUT(magneticfield_3),
};

int main(void) {
  const fh_scenario_t scenario = {
      .title = "Worlds.Environment.MagneticField",
      .stop_s = 1.0,
      .step_s = 0.01,
      .print_s = 0.5,
      .inputs = NULL,
      .input_count = 0,
      .outputs = outputs,
      .output_count = FH_COUNT(outputs),
  };
  return fh_run(&scenario);
}
