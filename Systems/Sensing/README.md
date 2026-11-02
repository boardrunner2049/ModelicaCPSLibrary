# Sensors: implemented IMU, GNSS and range fidelity

## Implemented continuous baseline

The default I0 IMU and ideal mag/GNSS/barometer evaluate outputs at the current simulation
time. There is no sample period, `sampleTime`, hold, device response, stochastic
noise or transport state. Fixed biases default to zero in the output frame.
The old Measurement/Response arrays, first-order filters and sampled wrappers are
archived under `deprecated/legacy_enu_flu/fire_modelica_models`; they are not active APIs.

- IMU: `transpose(R_bs) * (transpose(R_wb)*(a_w-gravity_w) + alpha_b x r_b + omega_b x (omega_b x r_b))`; gyro is `transpose(R_bs)*omega_b`.
- Magnetometer: `transpose(R_bs)*transpose(R_wb)*magneticField_w`, in Tesla.
- GNSS: antenna position and velocity already resolved in NED.
- Barometer: ambient Pa/K and geometric Up-positive altitude/climb rate; no pressure-height inversion.

`R_ab` converts components by `v_a=R_ab*v_b`. Standalone `R_bs=identity(3)`
aligns the sensor with body FRD. Vehicle geometry defaults to
`R_bImu=R_bMag=diag(1,-1,-1)`: local hover z is +g while the body-FRD output is -g.
`SensorSuite` is optional; `Examples.QuadImuOnly` uses a standalone IMU.
The suite owns GNSS/barometer rigid-mount position/velocity conversion and the
barometer NED-to-Up sign change. The IMU owns its lever-arm acceleration.
No mount correction is applied twice.

## Proposal status and composition rules

The architecture below was proposed on 2026-10-26. **IMU I0–I3, GNSS G0–G2 and
range L0/L1 now have implementations**. IMU details are documented in
[IMU usage, parameter conventions and limits](IMU/README.md) and section 7.14 of
the parent FastDyn interface document v0.11.
I0 remains the default. I1/I2/I3 select deterministic digital, calibration/statistical-error and
electromechanical compositions. GNSS G1/G2 and range L0/L1 are selected by Modelica
composition; see [GNSS implementation](GNSS/README.md), [range implementation](Range/README.md)
and `Examples.GnssRange`. GNSS defaults to G0; mag/barometer remain ideal.
G3, flow and range L2/L3 remain proposals.
The equations below describe the broader design; the IMU README specifies the
implemented signal-chain placement, clocks and reduced-model limitations.

Fidelity labels name recommended combinations, as propulsion labels do. The
independent axes remain measurement physics, response, error, acquisition,
transport and susceptibility. Select families independently; a higher label is
neither an accuracy certificate nor a compatibility check. Require actual port,
frame, time and capability contracts. Use behavior names for future classes.

Physical replacements may supersede lower-level surrogates. Do not stack a
second filter, noise process, lever arm, clock error or delay for the same effect.
Keep measured errors and attack ground truth in diagnostics, separate from
receiver-reported quality/covariance. A disturbance must not automatically tell
the estimator that an attack is active.

## IMU compositions and equations

| Profile | Composition | Purpose |
|---|---|---|
| I0 | Rigid-mount ideal specific force/rate; optional fixed bias | Baseline geometry and dynamics |
| I1 | Deterministic bandwidth, ADC/IIR, decimation and delivery | Digital output contract |
| I2 | I1 plus calibration, temperature, Gauss–Markov bias and noise | Navigation and controller response |
| I3 | MEMS drive/sense and proof mass, capacitive electronics, AGC/PLL and sense demodulation | Electromechanical response; illustrative parameters |

Let `f_s` and `omega_s` be the current ideal measurements. A proposed input-referred
error model, with separate coefficients for acceleration and angular rate, is:

$$
u_a=(I+S_a+M_a)f_s+b_a(T,t)+n_a+d_a,\qquad
u_g=(I+S_g+M_g)\omega_s+G_g f_s+b_g(T,t)+n_g+d_g.
$$

`S` is scale error, `M` cross-axis/nonorthogonality error, `G_g` gyro acceleration
sensitivity, `n` intrinsic noise and `d` an equivalent disturbance. For each axis:

$$
b(T,t)=b_0+k_T(T-T_0)+\tilde b(t),\qquad
\tilde b_{k+1}=\phi\tilde b_k+\sigma_b\sqrt{1-\phi^2}\xi_k,\quad
\phi=e^{-\Delta t/\tau_b},\quad \xi_k\sim\mathcal N(0,1).
$$

Here `sigma_b` is the stationary standard deviation. A random-walk alternative is
`b[k+1]=b[k]+q_b*sqrt(dt)*xi[k]`. Neither model automatically reproduces the entire
Allan-deviation curve; choose/fix parameters from device data. Temperature is
sensor temperature; a thermal lag from ambient is optional and must be explicit.

A first response approximation is `tau*der(z)+z=u`. Detailed profiles identify the
actual ordering of amplifier saturation, analog anti-alias filtering, ADC and
digital filtering. With a one-sided amplitude spectral density `N`,
`variance=N^2*B_ENBW` after filtering. Do not combine analog filtered noise and an
equivalent sampled-noise model for the same source. Random draws occur on an
explicit reproducible clock, never on each ODE residual evaluation.

The new I3 owns gyro drive/sense mechanics and electronics. Its true rate appears
only in reciprocal Coriolis forces; electrical demodulation generates the measured
rate. Accelerometers have separate proof-mass dynamics and capacitive readout.
See [implemented I3 equations and boundaries](IMU/README.md#i3-mechanics-and-electronics).

Acoustic exposure and static package coupling are separate under
`Vulnerabilities.Acoustic`. Pressure maps to generalized modal forces through an
identified effective area, signed coupling coefficients and incidence angle. No
package dynamics or FEM is required. I3 consumes force; I1/I2 can instead use a
steady carrier-response/baseband surrogate. [Equations and source reference](../../Vulnerabilities/Acoustic/README.md).
Do not also inject an equivalent rate for the same I3 exposure. The earlier
pressure-driven six-mode `AcousticSensor` remains a legacy model, not the current I3.

Sampling may alias surviving carriers; internal ADC rate, sensor ODR, bus reads and
FMI communication steps are distinct. The new I3 explicitly models demodulation
before output ADC. Resolve the carrier when simulating I3 or supplying pressure
waveforms. The envelope surrogate avoids carrier-rate communication but does not
model startup, PLL/AGC transients or large-signal response. P4 motor switching does
not generate acoustic input without an explicit transmission path.

## GNSS compositions and equations

| Profile | Composition | Purpose |
|---|---|---|
| G0 | Ideal local NED antenna position/velocity; optional fixed bias | Current baseline |
| G1 | Statistical PVT outputs, correlated errors, update rate/latency, fix/validity and uncertainty | Receiver-output effects on navigation |
| G2 | Satellite geometry, pseudorange/Doppler, clocks, atmosphere/multipath and navigation solution | Observable consistency and spoofing detection |
| G3 | Acquisition/tracking and signal competition, lock/reacquisition | RF capture and receiver-level defenses; deferred |

Preserve the existing suite's antenna kinematics:

$$
p_A=p_w+R_{wb}r_G,\qquad v_A=v_w+R_{wb}(\omega_b\times r_G).
$$

G1 samples `z_k=[p_A(t_k);v_A(t_k)]+b_k+L_k*xi_k`, where
`L_k*transpose(L_k)=Sigma_k`, then delivers that sample at `t_k+latency_k`.
Position and velocity share an acquisition epoch; Doppler-derived velocity errors
need not be the numerical derivative of noisy position. Covariance may include
cross-axis and position/velocity correlation. Quality presets are statistical
assumptions unless derived from a receiver model.

A G2 pseudorange model is:

$$
\rho_i=\|r_i^e-r_A^e\|+c(\delta t_r-\delta t_i)+I_i+T_i+m_i+\epsilon_i.
$$

Use consistent ECEF transmit/receive epochs, satellite/receiver clocks and Earth
rotation treatment. Estimate position/clock from ranges and velocity/clock drift
from Doppler or range rate. Define visibility, geometry rank and uncertainty.
Use WGS84/ECEF conversions where the current local spherical approximation is
insufficient. Do not add G1 position errors again if G2 already generates them.

Output-level spoofing uses a scenario displacement `q(t)=alpha(t)*d(t)`:

$$
p'=p_m+q,\qquad v'=v_m+\dot q,\qquad
\dot q=\dot\alpha\,d+\alpha\,\dot d.
$$

Evaluate the terms at the sample epoch and keep clock manipulation distinct from
delivery latency. Abrupt, drifting and deliberately inconsistent scenarios may
also be selected and labeled. This assumes a corrupted receiver output; it does
not prove RF capture. G2 injects `Delta_rho_i` and `Delta_rhodot_i` before the
navigation solution, with their consistency specified. A common range offset
primarily changes receiver clock bias, not an arbitrary position.
G3 is needed when signal takeover/lock behavior itself is the research question.

## H-Flow rangefinder and optical-flow compositions

Target: **Holybro H-Flow with PAA3905E1, AFBR-S50LV85D and ICM-42688-P**.
Downward mounting is the initial scenario, not a hard-coded body-axis convention.
Record the actual mounting pose, firmware revision and device parameters.

| Profile | AFBR rangefinder composition |
|---|---|
| L0 | Ideal unit-ray/plane intersection |
| L1 | Range errors, quantization, sampling/latency, validity and device status |
| L2 | Receiver pixel footprints, transmitter illumination, scene occlusion/reflectance and distance binning |
| L3 | ToF phase response, multipath, ambient light/saturation and measurement-mode behavior |

| Profile | PAA3905 optical-flow composition |
|---|---|
| F0 | Geometric integrated flow from a plane and sensor motion |
| F1 | Scale/bias/noise, integration windows, timing and quality |
| F2 | Multiple rays/depths, texture/lighting-dependent weighting and failure response |
| F3 | Rendered images, exposure/blur and image tracking, calibrated to the device |

The PAA3905 effective field of view is approximately 42 degrees. The AFBR receiver
has 32 pixels over 12.4 by 6.2 degrees, but its approximately 2 by 2 degree
transmitter normally illuminates only 1–3 pixels. Receiver field of view is not
the illuminated footprint. Range and flow can observe different surfaces at a
ledge. Share scene truth, but keep their optics and mounting offsets distinct.
Nominal range/lighting specifications are not guaranteed in every scene.

### Range equations and status

For a unit world-frame ray `d_s` from `p_s` and plane `n^T*x=h`:

$$
r=\frac{h-n^Tp_s}{n^Td_s},\qquad
r_{m,k}=Q\left(r(t_k)+b_r(T)+\sigma_r(r,\varrho,E_{\rm amb})\epsilon_k\right).
$$

Reject near-parallel, behind-sensor and unsupported returns explicitly.
Slant range is not vertical altitude. `varrho` represents surface reflectance.
Keep no-return, too-close, too-far and device-error policies distinct; do not
silently turn invalid measurements into valid saturated ranges.

L2 associates geometry with transmitter/receiver overlap and per-pixel response,
then models the selected binning policy. The PX4 AFBR driver consumes vendor
`Bin.Range` and `Bin.SignalQuality`; H-Flow does not require a 64-beam scan output.
A candidate L3 multipath response is
`z_j(f)=sum_m A_jm*exp(i*4*pi*f*r_jm/c)+eta_j`, followed by a mode-specific
phase/range estimator. Frequency ambiguity, calibration and multi-frequency
processing must be specified. This does not claim an exact vendor SDK implementation.

### Flow equations, integration and IMU ownership

At the camera mount:
`p_s=p_w+R_wb*r_s` and `v_s=v_w+R_wb*cross(omega_b,r_s)`.
For a static scene point `q_c=(X,Y,Z)` expressed in camera axes with positive
optical depth `Z`:

$$
\dot q_c=-v_c-\omega_c\times q_c,\qquad
\dot u=\frac{\dot XZ-X\dot Z}{Z^2},\quad
\dot v=\frac{\dot YZ-Y\dot Z}{Z^2}.
$$

Convert these normalized-image rates to the device's calibrated angular axes,
aggregate over observed features and integrate over the acquisition window:

$$
\Delta\theta_{{\rm flow},k}
=\int_{t_k-\Delta t_k}^{t_k}\omega_{\rm image}(t)\,dt+b_k+\eta_k.
$$

Use scene truth depth to generate optical motion, not corrupted measured ToF
range. Scene depth is internal truth and must not leak to the flight controller.
A flat-plane velocity approximation has `v≈r*flow_rate_compensated`, so
`delta_v≈flow_rate_compensated*delta_r+r*delta_flow_rate_compensated`.
The approximation requires consistent axes, geometry and time alignment.

PAA3905 frame acquisition is mode dependent (datasheet bright/low-light 126 fps,
super-low-light 50 fps). Chip motion accumulation/readout, gyro integration and
CAN publication are separate time contracts. Model quality/SQUAL, exposure and
invalid/false-motion handling against the selected firmware, not as a constant
perfect-quality output. A generic image tracker is a surrogate for the chip's
internal algorithm and needs calibration; rendered detail alone is not fidelity.

Instantiate the module's ICM-42688-P separately from the main flight-controller
IMU. Use measured module gyro increments over the flow window, rather than ideal
body truth. Apply the rotational compensation in the designated consumer once.
Acoustic errors in the two IMUs may differ; correlated exposure does not mean
identical noise or susceptibility.

## Proposed boundaries, scheduling and disturbances

IMU I1–I3, GNSS G1/G2 and range L1 implement epoch/delivery/sequence/validity
fields; their READMEs specify exact names and limits. The table below remains a
broader boundary proposal, including unimplemented flow windows and host adapters.

| Boundary | Proposed quantities | Meaning |
|---|---|---|
| Sample metadata | `sampleTime`, optional `windowStart/windowEnd`, `sequence`, `valid` | Simulation-clock acquisition epoch/window and freshness |
| Transport | `deliveryTime`, delivery/drop/stale status | Separate from sensor validity and acquisition time |
| IMU | acceleration/rate plus sample metadata | Sensor-local SI, at its mount |
| GNSS | NED position/velocity, reported clock/fix/uncertainty | Antenna point; attack truth kept separately |
| Range | distance [m], measurement status, orientation/FOV and internal quality | Binned slant range; preserve backend-specific invalid semantics |
| Flow | `flowIntegral[2]` [rad], `gyroIntegral[2]` [rad], `integrationInterval` [s], quality | Co-timed raw flow and measured gyro; not precomputed horizontal velocity |
| Scene provider | pose/epoch/optics requests; depth/range, validity, surface and image data as needed | Only the quantities required by the selected profile |

PX4's H-Flow DroneCAN path publishes
`com.hex.equipment.flow.Measurement` and
`uavcan.equipment.range_sensor.Measurement`. The flow message carries the two
integrals, interval and quality. Range carries distance, timestamp, orientation,
FOV and reading type; internal signal quality is not necessarily a wire field.
Map metadata to the actual firmware/protocol; do not invent wire fields.

| Disturbance | Injection point / required capability | Scope |
|---|---|---|
| GNSS PVT spoofing | Output trajectory/clock | Receiver-output consequence |
| GNSS observable spoofing | Pseudorange/Doppler before navigation solution | Observable consistency; G2 capability |
| Acoustic equivalent error | I1/I2 pre-readout equivalentError | Implemented steady-response gyro surrogate |
| Acoustic pressure | Independent PressureToForce → I3 MEMS modal forces | Implemented static package coupling |
| Range bias/dropout/false return | Selected output, pixel or ToF stage | Label output faults separately from optical mechanisms |
| Flow bias/freeze/quality corruption | Output/integration stage | Fault consequence |
| Lighting/texture/optical interference | Scene/imaging/ToF front end | Requires relevant F2/F3 or L2/L3 physics and calibration |

Use independent acquisition, noise-update, internal ADC, image exposure and
delivery clocks. Flow is a windowed observation, not an instantaneous value
held at the end of a frame. Keep the continuous plant integrating between events.
Lockstep scene capture should preserve simulated epochs; GPU wall-clock time
must not silently become device latency. Record scene/assets, rendering settings,
seed, calibration and firmware versions for replay.

## Reuse of slam_web and current integration gaps

Reference snapshots inspected on 2026-10-26:

- `CogniPilot/slam_web`: `92d348464a563caa4b1726d6e0be319c9ca5be94`.
- Its `CogniPilot/modelica_models` submodule: `3e04d9ed9b29888bdb61ee91167d71b38df81389`.
- PX4 driver/output reference: `ae4fd97df876b5dbb0def67e46c6f0ec7e63213a`,
  not a claim about the user's installed H-Flow firmware.

The pinned Modelica library provides a nine-ray plane-flow generator and test
examples. It assumes ENU/FLU and a co-located nadir camera; adapt frames, lever
arms and output angular-axis conventions explicitly. It uses a field-aggregate
approximation, not PAA3905-specific image processing.
`slam_web` provides RGB/depth rendering and a GPU cubemap-based 64-beam instantaneous
LiDAR snapshot. The latter is not exact ray tracing, AFBR ToF physics or a rotating
scan. Its stereo-disparity depth noise must not be reused as AFBR ToF noise.
The staged RGBD patch tracker is not an integrated PAA3905 emulator.
Complete SLAM FMU export is roadmap work; inspect the rendering/Modelica boundary
rather than assuming a portable drop-in FMU.

Suggested ownership: FIRE owns sensor equations and state; a scene provider owns
geometry/images/material metadata; the host owns lockstep exchange and device
delivery. F0/L0 and statistical profiles can run without a renderer. At higher
levels use shared-memory/external functions or co-simulation only after defining
the selected backend's contract and demonstrating compiler/FMI support.

Current FIRE `SensorMeasurements` contains neither range nor flow metadata.
`SensorSuite.ImuModel` and `SensorSuite.GnssModel` are replaceable; mag/barometer retain concrete ideal instances.
Sampled IMU metadata is separate from the existing measurement value record. The current FastDyn FMU LiDAR callback returns no samples, and
the RPLidar scan format is not the H-Flow DroneCAN contract. The current IMU batch
copies one reading into 17 slots; new profiles need actual sample history.
GPS currently receives host simulation time and a truth-derived yaw through the
export path: preserve acquisition timestamps and prevent unintended truth feedback.
These host changes are prerequisites for end-to-end claims and remain outside the sensor library implementation.

## Implementation order and acceptance criteria

1. Preserve existing continuous regression; add I1/G1 and analytic L0/F0 contracts.
2. Add L1/F1, a separate H-Flow IMU, windowed flow/gyro and device output timing.
3. Add I2/I3, G2 and L2/F2 as required; calibrate cheaper surrogates against detailed
   models or logs. Defer G3, L3 and F3 until the research question needs them.

| Area | Required evidence when implemented |
|---|---|
| Baseline | No-error/no-delay limits and explicit acquisition bypass recover applicable continuous outputs |
| IMU | Freefall/support/mount checks, PSD/Allan behavior, bandwidth/phase, saturation and independent streams |
| GNSS | Antenna motion, joint position/velocity timing, observable geometry/clock behavior and reported uncertainty |
| Acoustic | Measured/identified frequency response, correct internal ADC aliasing, detailed/surrogate convergence |
| Range | Tilted-plane slant range, ledges/occlusion, transmitter footprint, invalid returns and binning |
| Flow | Pure translation, rotation and mount lever arm; slopes/depth variation; texture/light failures; co-timed gyro |
| Cross-sensor | ToF corruption changes downstream scale, not generated raw optical motion; module and FC IMUs remain distinct |
| Timing/backend | No missing/duplicated samples; correct integration windows, latency, reset/replay and actual host delivery |
| Attacks | Paired nominal/attack runs with matched normal-noise seeds; navigation error, innovation/NIS, detection/failsafe/recovery |

Parameter defaults must identify whether they are illustrative, datasheet-derived
or measured. The full matrix is a design target; implemented sensor evidence is recorded in
[validation](../../docs/validation.md). No firmware/hardware calibration or
validation of GNSS/range hardware or proposed flow models is claimed.

## References

- [Kalibr IMU noise model](https://github.com/ethz-asl/kalibr/wiki/IMU-Noise-Model):
  stochastic model conventions; verify density/bandwidth conventions when adapting.
- [GNSS-SDR observables](https://gnss-sdr.org/docs/sp-blocks/observables/):
  pseudorange, carrier and Doppler equations.
- [Rocking Drones](https://www.usenix.org/conference/usenixsecurity15/technical-sessions/presentation/son)
  and [WALNUT](https://spqrlab1.github.io/papers/trippel-IEEE-oaklawn-walnut-2017.pdf):
  device-specific acoustic mechanisms, not universal susceptibility parameters.
- [Holybro H-Flow](https://docs.holybro.com/peripherals/h-flow-dronecan/overview),
  [firmware](https://docs.holybro.com/peripherals/h-flow-dronecan/firmware),
  [PixArt datasheet, distributor-hosted](https://www.codico.com/en/mpattachment/file/download/id/1203/),
  [Broadcom AFBR-S50LV85D datasheet](https://docs.broadcom.com/doc/AFBR-S50LV85D-DS).
- [Plane-flow generator](https://github.com/CogniPilot/modelica_models/blob/3e04d9ed9b29888bdb61ee91167d71b38df81389/Vehicles/Rdd2/simulateOpticalFlowPlane.mo)
  and [plane-flow tests](https://github.com/CogniPilot/modelica_models/blob/3e04d9ed9b29888bdb61ee91167d71b38df81389/Tests/OpticalFlowPlaneTests.mo).
- [GPU LiDAR](https://github.com/CogniPilot/slam_web/blob/92d348464a563caa4b1726d6e0be319c9ca5be94/src/lidar.ts),
  [depth noise](https://github.com/CogniPilot/slam_web/blob/92d348464a563caa4b1726d6e0be319c9ca5be94/src/gpu-depth-noise.ts),
  [sensor clock](https://github.com/CogniPilot/slam_web/blob/92d348464a563caa4b1726d6e0be319c9ca5be94/src/sensor-clock.ts)
  and [staged patch tracker](https://github.com/CogniPilot/slam_web/blob/92d348464a563caa4b1726d6e0be319c9ca5be94/models/Vision/Matching/RGBDPatchTracking.mo).
- [PX4 PAA3905](https://github.com/PX4/PX4-Autopilot/blob/ae4fd97df876b5dbb0def67e46c6f0ec7e63213a/src/drivers/optical_flow/paa3905/PAA3905.cpp),
  [AFBR](https://github.com/PX4/PX4-Autopilot/blob/ae4fd97df876b5dbb0def67e46c6f0ec7e63213a/src/drivers/distance_sensor/broadcom/afbrs50/AFBRS50.cpp),
  [flow publisher](https://github.com/PX4/PX4-Autopilot/blob/ae4fd97df876b5dbb0def67e46c6f0ec7e63213a/src/drivers/uavcannode/Publishers/FlowMeasurement.hpp)
  and [range publisher](https://github.com/PX4/PX4-Autopilot/blob/ae4fd97df876b5dbb0def67e46c6f0ec7e63213a/src/drivers/uavcannode/Publishers/RangeSensorMeasurement.hpp).
