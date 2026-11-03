/* FMI harness: Examples.QuadGroundCycle.
 *
 * Closed model, no inputs: MultirotorWithSensors (mass 1.5 kg, default QuadX
 * geometry, landing gear enabled: four legs 0.10 m below the CG at
 * (+-0.17, +-0.17), 3000 N/m and 150 N.s/m each, ground at z = 0), started
 * with the CG at NED z = -0.12 so the feet are 2 cm above the plane, motors
 * FirstOrderSpeed (tau 0.03 s, omegaMax 1000, start 0) behind PassThrough
 * ESCs, QuadraticBlade rotors (kT 1e-5, kQ 1.5e-7). The ESC command is 0
 * except on [2, 2.7) s, where it asks for 1.3 times weight in steady thrust
 * (omega_c = sqrt(1.3 m g/(4 kT)) = 691.43 rad/s).
 *
 * Run: experiment StopTime 6 s at its own Interval (2 ms communication step).
 * Sequence: free fall 2 cm, touchdown at 63.9 ms, overdamped settle to the
 * static deflection m g/(4k) = 1.226 mm, spin-up at 2 s, lift-off at 2.091 s,
 * powered climb, cutoff at 2.7 s, ballistic apex 0.8487 m at 2.90 s, second
 * touchdown at 3.2935 s with 3.83 m/s, settle again.
 *
 * The motion is purely vertical (symmetric legs and rotors, alternating spin
 * signs), so checks.toml compares against an independent 1-D reference
 * (scipy LSODA, rtol 1e-12, of m z'' = m g - 4 kT w^2 - 4 max(0, k pen + c v),
 * w' = (w_c - w)/tau): it agrees with this FMU to 1e-9 m in altitude, with
 * every communication step tried (2, 10, 50 ms). Native `rumoca sim` agrees
 * with the FMU to 2e-7 m (its own tolerance 1e-8 is the larger error).
 */
#include "fire_harness.h"

static const fh_output_t outputs[] = {
    FH_OUTPUT(altitude),
    FH_OUTPUT(verticalvelocity),
    FH_OUTPUT(normalload),
    FH_OUTPUT_INT(contactcount),
    FH_OUTPUT(acceleration_frd_3),
};

int main(void) {
  const fh_scenario_t scenario = {
      .title = "Examples.QuadGroundCycle",
      .stop_s = 6.0,
      .step_s = 0.002,
      .print_s = 0.25,
      .inputs = NULL,
      .input_count = 0,
      .outputs = outputs,
      .output_count = FH_COUNT(outputs),
  };
  return fh_run(&scenario);
}
