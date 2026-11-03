/* FMI harness: Worlds.Environment.Atmosphere.
 *
 * A constant-property atmosphere source: temperature, pressure and density are
 * parameters copied to the outputs. The model computes nothing (no altitude
 * dependence, no equation of state), so the only behaviour to establish is that
 * the tunable parameters reach the outputs unchanged and stay constant.
 * The declared defaults (288.15 K, 101325 Pa, 1.225 kg/m3) would be a weak test --
 * a compiler folding the bindings at export would pass -- so world.toml overrides
 * all three with the ISA state at 1000 m (281.65 K, 89874.5625 Pa, 1.111642 kg/m3).
 * The override was chosen ideal-gas consistent (rho = p/(R_air T)) so the run is a
 * physically meaningful state, but that consistency is a property of the chosen
 * inputs, not something the FMU establishes. */
#include "fire_harness.h"

static const fh_output_t outputs[] = {
    FH_OUTPUT(temperature),
    FH_OUTPUT(pressure),
    FH_OUTPUT(density),
};

int main(void) {
  const fh_scenario_t scenario = {
      .title = "Worlds.Environment.Atmosphere",
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
