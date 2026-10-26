# Independent acoustic vulnerability

These blocks approximate exposure and package coupling independently of IMU
fidelity. Package dynamics, room acoustics and FEM are intentionally absent.
All example gains are illustrative or user-identified, not measured hardware data.

The starting reference is
[MFQuadrotor.mo / GyroAcousticAtk](https://github.com/mhcho1994/FIRE_CP_Glimpse/blob/main/models/MFQuadrotor.mo).
That block combines source, force projection, mechanical response and a demodulated
yaw-rate error. Here these responsibilities are separated. Its numerical constants
are not copied as a commercial gyro calibration.

## Source and static package coupling

`SoundPowerToPressure` uses acoustic (not electrical speaker) power:

```text
I = acousticPower/(4*pi*distance^2)
pRms = sqrt(density*soundSpeed*I)
p(t) = sqrt(2)*pRms*envelope*cos(2*pi*frequency*t+phase)
```

The first two equations assume isotropic free-field radiation and a progressive
wave. Direct measured pressure at the sensor avoids this propagation assumption.
`TonePressure` implements the third equation. RMS and peak amplitudes are distinct.
Zero power produces exactly zero RMS pressure without a logarithm floor. The
reference code divides by density*soundSpeed; this implementation uses the
pressure/intensity relation [I=pRms²/(rho*c)](https://animations.physics.unsw.edu.au/jw/sound-impedance-intensity.htm).

`PressureToForce` is the complete package model at this fidelity:

```text
F_drive[3] = effectiveArea * pressure * sin(incidenceAngle) * driveCoupling[3]
F_sense[3] = effectiveArea * pressure * cos(incidenceAngle) * senseCoupling[3]
F_accel[3] = effectiveArea * pressure * accelCoupling[3]
```

Angle zero is the sense direction, matching the reference. `accelCoupling` already
includes directional projection. Coefficients belong to a device/mount profile;
they are distinct from exposure settings. `effectiveArea` is independent of drive
amplitude, replacing the reference's `(2*dis_d)^2` assumption. The outputs are
MEMS generalized forces, not vehicle body forces.

## Sensor-specific response

For I3, connect these forces directly to `ElectromechanicalSensor`. The sensor
owns drive/sense resonance, capacitive readout, AGC/PLL and demodulation. This
coupling block contains no extra copy of these dynamics.

For I1/I2, `EquivalentGyroResponse` evaluates the steady two-mode frequency response
at the configured carrier, followed by baseband demodulation and its LPF:

```text
D_d = k_d-m*w_ac^2 + j*c_d*w_ac
D_s = k_s-m*w_ac^2 + j*c_s*w_ac
Y = (D_d*F_s-kxy*F_d)/(D_d*D_s-kxy^2)
Y_e = Y/(1+j*w_ac*afeTau)
deltaRate = -LPF2(real(Y_e*exp(j*((w_ac-w_ref)*t+phase-refPhase))))/nominalScale
```

The implementation uses real/imaginary arithmetic and `atan2` phase calibration.
It handles direct sense forcing and drive-to-sense stiffness coupling without
reproducing the reference's suspect repeated `X_d1^2` term. The no-exposure response
is zero; nominal structural bias is not an attack term. The reference's separate
constant structural term is therefore not injected.

The input is a slowly varying **RMS envelope**, not an instantaneous waveform.
Drive frequency/amplitude/phase are assumed nominal; AGC/PLL transients, fast
carrier sweeps, large-signal saturation and mechanical startup are not modeled.
Envelope changes pass through baseband filters but do not reproduce package or
mechanical ring-up. This is a reduced response for system studies, not an RF or
full MEMS simulation. Compare it to I3 or measurements within its intended range.

## Composer configuration

```toml
[vulnerabilities.acoustic]
model = "modal_force" # I3; equivalent_rate for I1/I2
effectiveArea = 1e-10
incidenceAngle = 0.0
driveCoupling = [0.0, 0.0, 0.0]
senseCoupling = [0.0, 0.0, 1.0]
```

`modal_force` exports `imuAcousticPressure` [Pa instantaneous]. It may also specify
`accelCoupling`; the default is zero. Its gyro response parameters are owned by I3.

`equivalent_rate` exports `imuAcousticRmsPressure` [Pa RMS]. It additionally requires
`frequency` [Hz] and a `[vulnerabilities.acoustic.gyroPhysics]` record describing the
omitted sensor response; optional `phase` is in radians. It rejects accelCoupling.
See [I2 example](../../configs/quad_i2_acoustic.toml) and
[I3 example](../../configs/quad_i3_acoustic.toml).

Both paths are explicitly selected; choosing I3 alone does not enable an exposure.
Do not attach both equivalent rate and modal force for the same exposure. The
composer rejects mismatched tier/capability combinations and unused source fields.
