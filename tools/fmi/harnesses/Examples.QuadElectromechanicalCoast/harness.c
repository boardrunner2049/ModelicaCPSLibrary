/* FMI harness: Examples.QuadElectromechanicalCoast.
 *
 * Closed model, no inputs: MultirotorWithSensors (QuadX, 1.5 kg, gear
 * disabled, start NED {0,0,-1}); per rotor an AverageUnidirectional ESC on a
 * 16 V bus, a DynamicDCEquivalent motor (R 0.2 ohm, L 1 mH, k 0.02, J + JLoad
 * = 2e-4 kg.m^2) started at hover (606.42 rad/s, 2.758 A), QuadraticBlade
 * rotors (kT 1e-5, kQ 1.5e-7) and HubReaction (spin-acceleration reaction).
 * ESC command: hover (0.79251) before 0.3 s, 0 on [0.3, 0.6), 0.95 after.
 *
 * Run: experiment StopTime 1 s; 0.5 ms communication step (the model's
 * Interval is 0.1 ms; the electrical time constant L/R is 5 ms). Phases:
 * exact electromechanical hover; at cutoff the ESC freewheels (0 V) and the
 * winding current falls to zero in 0.22 ms against the back-EMF, the ESC
 * reports coast at the zero crossing (t = 0.30022 s), the motor reinits its
 * current to 0 and the terminal follows the back-EMF, so the rotors coast down
 * under blade drag only, w = w_z/(1 + kQ w_z (t - t_z)/J); the vehicle sinks;
 * at 0.6 s the restart drives 15.2 V, current peaks at 20.4 A, the speed
 * overshoots hover and the vehicle bottoms out at 0.899 m (0.83 s) and climbs.
 *
 * The four drive trains are identical and the motion is vertical, so
 * checks.toml uses an independent reference (scipy LSODA, rtol 1e-12) of
 * L i' = V - R i - k w, J w' = k i - kQ w^2, z'' = g - 4 kT w^2/m with the
 * zero-crossing projection; it agrees with this FMU to 2e-8 A / 1e-8 rad/s.
 * Native `rumoca sim` agrees with the FMU to 3e-7 relative.
 */
#include "fire_harness.h"

static const fh_output_t outputs[] = {
    FH_OUTPUT(altitude),
};

int main(void) {
  const fh_scenario_t scenario = {
      .title = "Examples.QuadElectromechanicalCoast",
      .stop_s = 1.0,
      .step_s = 0.0005,
      .print_s = 0.05,
      .inputs = NULL,
      .input_count = 0,
      .outputs = outputs,
      .output_count = FH_COUNT(outputs),
  };
  return fh_run(&scenario);
}
