/* FMI harness: Systems.Propulsion.Motors.ThreePhasePM.
 *
 * Wrapper.mo drives the floating-star PM motor (0.2 Ohm, 0.1 mH, ke 0.01,
 * 7 pole pairs, sinusoidal back-EMF, J 8e-6 + JLoad 2e-6 kg m^2, b 1e-5,
 * theta0 0.3 rad) with an ideal sine-commutated voltage aligned with the
 * back-EMF (q axis).
 *
 * Scenario: the motor starts spinning at omega_start = 50 rad/s with
 * non-zero winding currents i_start = {1, -0.4, -0.6} A; for 0.1 s the drive
 * applies 0 V (phases shorted through the sources), so the initial currents
 * decay with tau_e = L/R and back-EMF braking plus friction spin the rotor
 * down. Then +6 V amplitude (spin-up from near stall to the
 * no-load speed), a 0.01 N m load torque on [0.6, 1.0) s, then -6 V with no
 * load (reversal through zero to the negative no-load speed) until 1.8 s.
 * Throughout, a 2 V / 50 Hz common-mode voltage rides on all three phases:
 * the floating star must reject it (the star point follows it and every
 * closed-form current/torque check still holds).
 * checks.toml establishes the initial state (i_start honoured, phase 3 from
 * floating-star KCL, omega_start, electrical angle = theta0, torque from
 * i_start and the theta0-shifted shape, stored energy) and the spin-down,
 * and, against the closed-form dq steady state
 *   V = (R + (p*omega*L)^2/R)*Iq + ke*omega,  1.5*ke*Iq = b*omega + loadTorque,
 * the stall current rise with tau_e = L/R, the steady speeds and torques,
 * the copper loss 1.5*R*(Iq^2 + Id^2) including the d-axis current that the
 * uncompensated inductance draws, each phase current's waveform
 * Iq*sin(theta_j) + Id*cos(theta_j) (a swapped or mis-shifted phase would
 * fail), sign symmetry of the reversal, the floating-star KCL and star-point
 * potential, and the integrated energy balance. */
#include "fire_harness.h"

static double voltage_profile(double t) {
  if (t < 0.1)
    return 0.0;
  if (t < 1.0)
    return 6.0;
  return -6.0;
}

static const fh_input_t inputs[] = {
    FH_INPUT(voltageamplitude, FH_FUNCTION(voltage_profile)),
    FH_INPUT(loadtorque, FH_PULSE(0.6, 0.4, 0.0, 0.01)),
    FH_INPUT(commonmode, FH_SINE(0.0, 2.0, 50.0, 0.0)),
};

static const fh_output_t outputs[] = {
    FH_OUTPUT(omega),          FH_OUTPUT(torque),      FH_OUTPUT(copperloss),
    FH_OUTPUT(starcurrentsum), FH_OUTPUT(starvoltage), FH_OUTPUT(storedenergy),
    FH_OUTPUT(energyerror),
};

int main(void) {
  const fh_scenario_t scenario = {
      .title = "Systems.Propulsion.Motors.ThreePhasePM",
      .stop_s = 1.8,
      .step_s = 0.00005,
      .print_s = 0.1,
      .inputs = inputs,
      .input_count = FH_COUNT(inputs),
      .outputs = outputs,
      .output_count = FH_COUNT(outputs),
  };
  return fh_run(&scenario);
}
