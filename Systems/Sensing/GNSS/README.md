# GNSS G0, G1 and G2

These are Modelica compositions. G0 is the backward-compatible default. Use
`Sensor`, `StatisticalSensor` or `ObservableSensor`; profile names describe scope,
not certified receiver accuracy. Parameters are illustrative, not device calibration.

| Profile | Class | Implemented behavior |
|---|---|---|
| G0 | `Sensor` | Continuous local NED antenna position/velocity with optional fixed bias |
| G1 | `StatisticalSensor` | Joint PVT acquisition, full correlated white/colored errors, covariance, fix loss and packet delay |
| G2 | `ObservableSensor` | Satellite visibility, pseudorange/range rate/Doppler, clocks, supplied path errors/corrections, weighted PVT, covariance and geometry failure |

## Mount and time contract

All three accept antenna-point `position_w`, `velocity_w` in the same fixed local
NED frame. `SensorSuite` applies `p_A=p_w+R_wb*rGnss_b` and
`v_A=v_w+R_wb*cross(omega_b,rGnss_b)` once. Its new replaceable `GnssModel`
allows G1/G2 without changing the existing measurement record:

```modelica
Systems.Sensing.SensorSuite sensors(
  redeclare model GnssModel=Systems.Sensing.GNSS.StatisticalSensor,
  gnss(samplePeriod=0.1,latencySamples=2,
       noiseFactor=diagonal({1,1,2,0.1,0.1,0.2})));
```

Supply the usual suite truth/environment inputs. G2 additionally needs satellite
state inputs on `sensors.gnss`. Direct composition is demonstrated in
[`Examples.GnssRange`](../../../Examples/GnssRange.mo).
Inputs with default bindings must be overridden by component modifications, for
example `satelliteAvailable=provider.available`; required position/velocity inputs
may be assigned by equations.

G1/G2 acquire at `startTime+k*samplePeriod` and deliver after
`latencySamples*samplePeriod`. Latency is a nonnegative integer number of output
periods, not a continuous transport delay. `startTime` must be at or after the
simulation start. Position, velocity, covariance, validity and fix type are queued
atomically. G2 clock, PDOP and satellite count use the same queue clock.

`sequence` counts every delivered packet, including invalid fixes. It starts at
zero; `sampleTime=deliveryTime=-1` and `valid=false` before the first delivery.
Outputs hold between deliveries. Invalid delivered PVT/covariance payloads are
zero; consumers must check `valid`/`fixType` (0=no fix, 3=3D fix).
There is no transport drop/reorder model. G0 has no clock or queue; reducing a
sampled period is not an exact continuous bypass.

## G1 errors and uncertainty

The six rows are N/E/D position [m], then N/E/D velocity [m/s]. `noiseFactor`
and `biasFactor` are arbitrary square-root factors, not covariance matrices:

```
phi[j] = exp(-samplePeriod/biasTau[j])
x[j,k] = phi[j]*x[j,k-1] + sqrt(1-phi[j]^2)*normal[j,k]
error = biasFactor*x + noiseFactor*white
reportedCovariance = biasFactor*transpose(biasFactor)
                   + noiseFactor*transpose(noiseFactor)
```

Latent bias modes start from independent N(0,1), so the stated covariance is
stationary from the first acquisition. The factors can correlate axes and
position with velocity. `biasTau` describes the independent latent modes.
Fixed biases are not included in reported uncertainty. `biasState` is diagnostic
truth, not a receiver-reported error estimate. `fixAvailable` controls scenario
availability; this is not a modeled RF lock detector. Randomness uses explicit
`seed` and `streamId`, only at acquisitions, with no global or wall-clock RNG.
`noiseFactor` is a per-sample scale, not an amplitude spectral density.

## G2 satellite geometry and ephemeris

Geometry is the receiver-to-satellite arrangement, not the ephemeris record itself.
Ephemeris is one way to calculate the required satellite states. G2 accepts
`satellitePosition_e` and `satelliteVelocity_e` in ECEF **at the reception epoch**,
plus clock bias [s], clock drift [s/s] and an availability mask. All states must
refer to the same GNSS time and datum. Do not preapply flight-time or Earth-rotation
corrections to these inputs.

`BroadcastConstellation` evaluates parsed `BroadcastEphemeris` records. Its
`epoch0+time` is GNSS seconds of week, not UTC. The GPS broadcast algorithm includes
Kepler iteration, harmonic orbit terms, node/inclination rates, half-week wrapping,
clock polynomial, eccentric-orbit relativity and GPS L1 TGD. Velocity and clock
drift use a centered 0.1 s difference. Health and `fitInterval` invalidate old data;
`fitInterval` is the full window centered on toe/toc. Records belong to the adjacent
GNSS week; week-number disambiguation and navigation-message updates are the host's
responsibility. The provider is GPS/L1-specific, not a GLONASS or multi-system decoder.
No RINEX/SP3 parser, live download or real constellation dataset is bundled.
The example uses a synthetic 24-satellite constellation, not measured ephemeris.

The WGS84 `Reference` gives geodetic latitude/longitude in radians and ellipsoidal
height. `referenceFrame` constructs its ECEF origin and NED-to-ECEF rotation.
Local positions are Cartesian offsets in that fixed tangent frame; this is not
latitude/longitude increment integration. `elevationMask` and `satelliteAvailable`
select observations. The mask uses an ellipsoid-normal approximation at the
antenna, appropriate near Earth's surface. Obstructions must be supplied externally.

`signalGeometry` iterates geometric flight time, linearly propagates the reception
satellite state back to transmission, and rotates transmission ECEF into reception
ECEF. This short-flight linear orbit approximation omits acceleration over roughly
0.07 s (millimetre-order orbit displacement for GPS); the provider itself evaluates
the full broadcast orbit at reception. Range gradients and range rates differentiate
this same implicit flight-time model, including Earth rotation. Satellite clock
bias is also propagated to transmission using its supplied drift.

Pseudorange includes receiver/satellite clocks, `ionosphereDelay`,
`troposphereDelay`, `multipath`, `rangeError` and independent per-acquisition
`rangeNoiseStd`. Range rate includes both clock drifts, `rateError` and
`rateNoiseStd`; Doppler is `-rangeRate*carrierFrequency/c`. For time-varying path
errors, supply their derivatives in `rateError` to make code and range rate
consistent. Path terms are explicit inputs, not built-in Klobuchar, weather or
scene models. Receiver `rangeCorrection`/`rateCorrection` are separate inputs;
truth delays are never automatically subtracted by the receiver.

`solvePvt` estimates ECEF position/clock with iterative weighted least squares,
then velocity/clock drift from range rate. It starts from the configured NED
`initialPosition`, never antenna truth. Fewer than four usable satellites, relative
normal-matrix pivot failure or nonconvergence yields no fix. Normal equations suit
this reduced model; `rankTolerance` rejects badly conditioned geometry rather than
regularizing a false solution. There is no integrity/RAIM, RTK, carrier phase, RF
tracking or spoof-capture model. Nonzero inherited PVT biases are rejected in G2;
inject errors through observables instead of double-counting G1 errors.

Reported covariance uses configured positive `rangeSigma`/`rateSigma` and estimated
geometry, not the actual injected error. Position/velocity cross-covariance is zero
under the independent code/rate noise assumption. PDOP is from the unweighted
geometry matrix. Invalid solutions have `pdop=-1` and zero clock estimates.
`pseudorange`, `rangeRate`, `doppler` and `visible` are acquisition diagnostics;
`elevation` is continuous geometry. These are not delayed receiver PVT outputs.
A common range offset changes estimated clock bias, while satellite-dependent
errors can change position. Clock manipulation and packet latency remain separate.

## Validation and integration

`FIRE_TEST_OMC=1 python3 -m unittest discover -s tools -p 'test_gnss_range.py'`
runs native numerical and stochastic checks. `tools/verify.py` also discovers all
four new physics tests. Tests cover analytic orbit/WGS84/rotation signs, numerical
range derivatives, PVT and clock recovery, rank/visibility failure, delayed fix
loss, mount motion, covariance, seed replay and solver-step independence.
See [validation scope](../../../docs/validation.md).

The current TOML `fire-compose` GNSS selector still supports only the ideal default;
G1/G2 are selected by Modelica composition/redeclaration. Existing FastDyn GNSS export
adapters still need explicit timestamp/quality plumbing for sampled receivers.
Native validation does not certify Rumoca/FMI3, firmware delivery or hardware accuracy.

References: [ESA broadcast orbit algorithm](https://gssc.esa.int/navipedia/index.php/GPS_and_Galileo_Satellite_Coordinates_Computation),
[transmit/reception ECEF convention](https://gssc.esa.int/navipedia/index.php/Satellite_Coordinates_Computation),
[local-frame transformations](https://gssc.esa.int/navipedia/index.php/Transformations_between_ECEF_and_ENU_coordinates),
[geometry and covariance](https://gssc.esa.int/navipedia/index.php?title=Positioning_Error).
