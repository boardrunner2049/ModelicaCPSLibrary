# IMU I0–I3: digital output, errors and electromechanics

The current fidelity definition is `digital_errors_electromechanical_v2`.
All outputs are sensor-local SI at the configured rigid mount. I0 remains the
unchanged default. Fidelity and acoustic exposure are separate selections.

| Level | Class / composer model | Behavior |
|---|---|---|
| I0 | `Sensor` / `ideal` | Continuous rigid-mount kinematics; optional fixed bias |
| I1 | `DigitalSensor` / `digital` | Deterministic analog bandwidth, ADC clip/quantization, IIR, decimation, delivery |
| I2 | `ErrorSensor` / `errors` | I1 plus calibration, device temperature, Gauss–Markov bias and white noise |
| I3 | `ElectromechanicalSensor` / `electromechanical` | Three vibratory gyros and three capacitive proof masses, electronics, common ADC/output chain |

## Selection and examples

Start from [quad_i1.toml](../../../configs/quad_i1.toml),
[quad_i2.toml](../../../configs/quad_i2.toml) or
[quad_i3.toml](../../../configs/quad_i3.toml). All physical coefficients are
illustrative; none is a calibrated ICM-42688-P or other commercial IMU profile.

```toml
[models.imu]
model = "digital" # ideal | digital | errors | electromechanical
```

Use `SensorSuite(redeclare model ImuModel=...DigitalSensor, imu(p(...)))`
for direct Modelica composition. Global acquisition stays continuous. Sensor
selection does not change propulsion, other sensors, or actuator clocks.

All sampled models require explicit `samplePeriod`, `adcDecimation`, `fullScale`,
`quantum` and `digitalTau` in composer settings. I1/I2 additionally require
`analogTau`. I2/I3 require `seed` and `streamId`. I1 rejects calibration/noise
settings instead of silently ignoring them. I3 rejects `analogTau`: its circuit
bandwidth belongs to `gyroPhysics.afeTau` and `accelPhysics.afeTau`.

I3 requires `[imu.gyroPhysics]` with `mass, driveFrequency, senseFrequency,
driveQ, senseQ, driveAmplitude` and `[imu.accelPhysics]` with `mass, frequency,
quality`. Remaining illustrative circuit/controller defaults are resolved into
the generated source and manifest. See [GyroParameters](../../../Physical/Sensors/MEMS/GyroParameters.mo)
and [AccelerometerParameters](../../../Physical/Sensors/MEMS/AccelerometerParameters.mo).
The same parameter record configures independent axes; custom Modelica assemblies
can use different records per axis. No device-specific shared die modes are assumed.

## I0–I2 equations and stage ownership

With `v_w=R_wb*v_b`, `v_b=R_bs*v_s` and CG-to-IMU lever arm `r_b`:

```text
f_s = transpose(R_bs)*(transpose(R_wb)*(a_w-gravity_w)
      + cross(alpha_b,r_b) + cross(omega_b,cross(omega_b,r_b)))
w_s = transpose(R_bs)*omega_b
```

I0 adds the optional fixed `accelBias/gyroBias` and delivers continuously.
I1 applies only deterministic readout to `[f_s;w_s]`:

```text
tau_a*der(u) + u = ideal + equivalentError
h = samplePeriod/adcDecimation
adc[n] = clip(quantum*floor(clip(u[n])/quantum + 0.5))
d[n] = exp(-h/tau_d)*d[n-1] + (1-exp(-h/tau_d))*adc[n]
output[k] = d[k*adcDecimation], delivered L output periods later
```

`quantum=0` bypasses rounding but retains clipping. `digitalTau=0` bypasses IIR.
Analog states initialize at their initial input; the first IIR state equals the
first ADC value. Decimation selects samples rather than averaging them.

I2 replaces `ideal` with calibrated values plus temperature offset:

```text
c_a = (I+diag(scale_a)+M_a)*f_s + accelBias
c_g = (I+diag(scale_g)+M_g)*w_s + G_g*f_s + gyroBias
analog drive = [c_a;c_g] + temperatureCoefficient*(sensorTemperature-referenceTemperature)
               + equivalentError
b[n+1] = phi*b[n] + biasStd*sqrt(1-phi^2)*xi[n+1], phi=exp(-h/biasTau)
ADC input = u[n] + b[n] + noiseStd*eta[n]
```

Bias starts in its stationary distribution. `noiseStd` is the standard deviation
per **ADC tick**, not a spectral density. This is ADC-equivalent noise, added after
the analog pole and before clipping/IIR. Noise spectra at the delivered rate depend
on the ADC period, filtering and decimation. Changing ODE solver steps does not
change random draws; each IMU owns its seed/stream state. Distinct stream IDs must
be assigned to independent IMUs. I1 does not advance the RNG. Setting all I2 error
coefficients to zero recovers I1 with the same readout and queue settings.

## I3 mechanics and electronics

`ElectromechanicalSensor` computes mount kinematics once, sends true rate/specific
force into physical components, and reads their electrical outputs. True rate is
never added directly to the measured rate. Gyro mechanics use a right-handed
translational drive/sense pair with one effective mass:

```text
m*xdd + cd*xd + kd*x + kxy*y = F_drive + F_external_drive + 2*m*Omega*yd
m*ydd + cs*yd + ks*y + kxy*x = F_external_sense - 2*m*Omega*xd
kd=m*(2*pi*f_drive)^2, cd=m*(2*pi*f_drive)/Q_drive
```

The reciprocal Coriolis terms exchange energy between modes without adding net
mechanical power. `kxy` is symmetric stiffness coupling and must preserve positive
definiteness. Internal Euler/centrifugal terms, nonlinear mechanical stiffness,
package modes and cross-axis die modes are outside this two-mode approximation.

The drive actuator uses differential comb capacitances `C±=C0*(1±x/length)` and
`F=sum(V²*dC/dx)/2`, with electrode voltages `biasVoltage±command`. Readout comb
capacitances give `DeltaC=2*C0*q/length`; the carrier-averaged C/V voltage is
`vIdeal=-carrierVoltage*DeltaC/Cf`. A first-order circuit response and voltage rails
produce the detected voltage. Electrode stroke assertions prevent use outside the
positive-capacitance range. This is an averaged electrical block model, not a
transistor netlist or a resolved RF carrier/switching circuit. Readout back-action
and full circuit supply-power accounting are not included.

Measured drive displacement is synchronously detected into two-pole I/Q filters.
AGC compares detected amplitude with `driveAmplitude` and controls bounded drive
voltage. PLL compares detected phase with the nominal calibrated C/V phase and
controls oscillator frequency. Both use PI controllers with anti-windup. Neither
controller reads the mechanical natural frequency at runtime. `referenceFrequency`
is a fixed electrical calibration and may differ from mechanical `driveFrequency`.
Default `startupFraction=1` represents an already oscillating drive; set it lower
for startup analysis. The nominal drive must have voltage headroom.

The controller equations, away from the anti-windup limits, are:

```text
I_d=LPF2(2*x_measured*cos(phi)), Q_d=LPF2(2*x_measured*sin(phi))
A_hat=sqrt(I_d^2+Q_d^2)*sqrt(1+(w_ref*afeTau)^2)
e_A=1-A_hat/A_ref, der(z_A)=agcKi*e_A
U=clip(U0*(1+agcKp*e_A+z_A), 0, maxDriveVoltage)
e_phi=wrapToPi(atan2(I_d,Q_d)-phaseSet), phaseSet=-atan(w_ref*afeTau)
der(z_phi)=pllKi*e_phi
w=clip(w_ref+pllKp*e_phi+z_phi, w_ref*(1-pllRange), w_ref*(1+pllRange))
der(phi)=w, commandVoltage=U*cos(phi)
```

Each integrator stops when its error would push an already saturated output
further into saturation. Phase detection is gated below a small drive amplitude.

Sense I/Q detection includes fixed nominal mechanical/C/V phase and scale:

```text
Hy(j*w)=1/(ks-m*w^2+j*cs*w)
phaseShift=arg(Hy(j*w_ref))-atan(w_ref*afeTau)+demodPhaseError
I_y=LPF2(2*y_measured*cos(phase+phaseShift))
rate=-I_y/(2*m*A_ref*w_ref*abs(Hy(j*w_ref))/sqrt(1+(w_ref*afeTau)^2))
```

This is **open-loop sense**. Force rebalance and automatic quadrature cancellation
are not implemented. Fixed scale/phase calibration deliberately preserves errors
from drive detuning or phase mismatch. `gyroLocked[3]` diagnoses AGC/PLL convergence
and readout saturation; it is separate from delivery validity.

Accelerometers use `m*zdd+c*zd+k*z=-m*f_s+externalForce`, the same capacitive
readout primitive, and `aMeasured=-(2*pi*frequency)^2*zMeasured`. They initialize
at the initial force equilibrium and do not contain gyro AGC/PLL loops.

I3 output calibration, temperature coefficients and clocked noise are optional
**residual** errors in the common readout. They default to zero. Do not add a second
surrogate for effects already represented physically. Explicit thermomechanical
Brownian noise, transistor noise and temperature-dependent stiffness are not
implemented; the residual model does not certify these mechanisms.

## Independent acoustic vulnerability

[Acoustic blocks and equations](../../../Vulnerabilities/Acoustic/README.md) describe
static pressure-to-force coupling and an inexpensive I1/I2 response. Examples:
[quad_i2_acoustic.toml](../../../configs/quad_i2_acoustic.toml),
[quad_i3_acoustic.toml](../../../configs/quad_i3_acoustic.toml).

- `modal_force` requires I3 and exports `imuAcousticPressure` [Pa instantaneous].
  The external vulnerability produces forces on MEMS modes; I3 handles resonance
  and electronics. No acoustic pressure port is intrinsic to the new I3 class.
- `equivalent_rate` requires I1/I2 and exports `imuAcousticRmsPressure` [Pa RMS,
  slowly varying envelope]. Its configured tone frequency/phase and device response
  produce a baseband rate error before the common readout. It models gyro only.
- Without acoustic configuration, I3 exports `imuGyroDriveForce[3]`,
  `imuGyroSenseForce[3]`, `imuAccelForce[3]` [N, sensor/modal axes] with start=0.
  They can be driven by another external coupling model. They are not body forces.

These are alternative paths for the same exposure; never add the equivalent rate
error again after injecting the physical force. An exposure turned off removes the
forcing, while existing mechanical/filter states are allowed to decay naturally.

## Timing and export contract

I1–I3 expose `imuSampleTime`, `imuDeliveryTime`, `imuSequence`, `imuValid`.
Before the first delivery: times=-1, sequence=0, valid=false, outputs=0.
Acquisition occurs at `k*samplePeriod`; delivery is delayed by
`latencySamples*samplePeriod`. Values and metadata hold between deliveries.
`sampleTime` denotes the ADC/decimation output epoch, not a compensation for
analog/IIR group delay. `valid` indicates that a sample has been delivered, not
that the drive loop is locked or that an estimator should trust the sample.
I2/I3 export `imuTemperature` [K] independently of ambient temperature. I3 also
exports `imuGyroLocked[3]`. The sampled core contract remains
`ned_frd_sampled_imu_v1`; the manifest records the revised fidelity definition.

Native I3 validation uses DASSL and resolves the mechanical carrier; a 20 us maximum
step is used for illustrative 1 kHz drive/2 kHz accelerometer examples. Verify step
convergence for changed parameters. FMI2 CS tests explicitly select CVODE with
`setCommandLineOptions("--fmiFlags=s:cvode")`; default forward Euler at a coarse
communication step is unsuitable for these oscillators. A waveform pressure input
must itself be supplied at a rate that resolves its carrier even with an adaptive
internal solver. RMS-envelope inputs to the surrogate do not require carrier-rate
communication. See [OpenModelica FMI guide](https://openmodelica.org/doc/OpenModelicaUsersGuide/v1.25.0/fmitlm.html).

OpenModelica 1.26.1 CVODE also failed at an exact ADC-event stop in the I2 acoustic
FMI2 test on a 20 us communication grid. That test passes on a non-aligned
`20/sqrt(2)` us grid; the cause remains unresolved. See
[validation details](../../../docs/validation.md#imu-i0i3-revised-fidelity-2026-10-26)
before choosing a host stepping policy; general FMI2 step compatibility is not
established.

## Legacy compatibility and validation

Old `StatisticalSensor/statistical`, `DeviceSensor/device` and
`AcousticSensor/acoustic` preserve their equations. Their manifest levels are now
`legacy_I1`, `legacy_I2`, `legacy_I3` rather than claiming the new fidelity semantics.
Preserved examples are `configs/legacy_quad_{statistical,device,acoustic}.toml`.
The old acoustic model remains an identified pressure-driven error approximation;
it is not the new electromechanical I3.

Run `FIRE_TEST_OMC=1 python3 -m unittest discover -s tools -p test_imu.py -v`.
The suite covers deterministic reduction, calibration/noise/replay, ADC timing,
physical rate scale/sign, AGC startup/disturbance rejection, PLL detuning, acoustic
units/zero exposure/surrogate comparison and generated-model/FMI2 boundaries.
Numerical evidence is recorded in `build/imu-validation`. These checks do not
establish hardware susceptibility, proprietary chip architecture, Rumoca/FMI3,
FastDyn firmware integration or H-Flow optical-flow integration.
