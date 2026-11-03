/* FMI harness: Systems.Propulsion.Motors.DynamicDCEquivalent (via Wrapper.mo).
 *
 * The component cannot run with open pins: an unconnected top-level pin pins
 * the winding current to zero, so its currentZero reinit has no state to act
 * on (Rumoca refuses it with ES017; OpenModelica refuses it too). Wrapper.mo
 * feeds it from a commanded voltage source instead.
 *
 * Scenario: 12 V from t = 10 ms spins the rotor up towards V/k = 600 rad/s; a
 * drive-reported current zero at t = 100 ms reinitializes the winding current
 * to 0 (it then recovers); a 5 mN.m load from t = 200 ms settles the speed
 * lower. checks.toml holds the values `rumoca sim` computes for the same
 * scenario (NativeRig in the comment there). */
#include "fire_harness.h"

static const fh_input_t inputs[] = {
    FH_INPUT(voltage, FH_STEP(0.01, 0.0, 12.0)),
    FH_INPUT_BOOL(currentzero, FH_PULSE(0.1, 0.01, 0.0, 1.0)),
    FH_INPUT(loadtorque, FH_STEP(0.2, 0.0, 0.005)),
};

static const fh_output_t outputs[] = {
    FH_OUTPUT(current),
    FH_OUTPUT(omega),
    FH_OUTPUT(electromagnetictorque),
    FH_OUTPUT(storedenergy),
};

int main(void) {
  const fh_scenario_t scenario = {
      .title = "Systems.Propulsion.Motors.DynamicDCEquivalent",
      .stop_s = 0.4,
      .step_s = 0.0001,
      .print_s = 0.05,
      .inputs = inputs,
      .input_count = FH_COUNT(inputs),
      .outputs = outputs,
      .output_count = FH_COUNT(outputs),
  };
  return fh_run(&scenario);
}
