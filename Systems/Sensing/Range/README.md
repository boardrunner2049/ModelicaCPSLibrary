# Range L0 and L1

`Range.Sensor` (L0) evaluates an ideal ray/plane intersection continuously.
`Range.SampledSensor` (L1) adds bias, temperature error, per-sample noise,
quantization, availability/device status and atomic sample delivery. These are
single-ray surrogates; they do not implement AFBR pixel optics or the vendor SDK.

Inputs are sensor position `position_w`, unit ray `direction_w`, nonzero plane
normal `planeNormal_w` and `planeOffset` for `n*x=h`, all in world NED.
Defaults point Down toward the plane z=0. The normal need not be normalized, but
scale `planeOffset` with it. At a body mount use `position_w=p_w+R_wb*r_b` and
`direction_w=R_wb*R_bs*opticalAxis_s`; the models do not apply a second mount transform.
Default-bound inputs must be overridden in component modifications:

```modelica
Systems.Sensing.Range.SampledSensor rangefinder(
  direction_w=R_wb*{0,0,1},planeNormal_w={-0.1,0,1},planeOffset=0,
  minRange=0.1,maxRange=30,samplePeriod=0.02,latencySamples=1,
  bias=0.01,noiseStd=0.02,quantum=0.001);
equation
  rangefinder.position_w=p_w+R_wb*{0.1,0,0};
```

Range is `(h-n*p)/(n*d)`, a **slant distance**, not vertical altitude. Parallel rays
use the relative cosine threshold `parallelTolerance`; negative intersections are
behind the sensor. Limits are inclusive: exactly `minRange`/`maxRange` is valid.

| Status | Constant | Meaning |
|---|---|---|
| 0 | `Uninitialized` | L1 has delivered no packet |
| 1 | `Valid` | Supported return and measured value within range |
| 2 | `NoReturn` | Parallel/behind ray, or L1 `returnAvailable=false` |
| 3 | `TooClose` | Geometric or measured distance below minimum |
| 4 | `TooFar` | Geometric or measured distance above maximum |
| 5 | `DeviceError` | L1 explicit device-error input |

Invalid distance is zero; **always check `valid` or `status`**. L1 prioritizes device
error, then external no-return, geometry status, and measured range limits. It does
not clamp invalid returns into valid distances. Internal `quality` is sampled from
`signalQuality` [0,1] and zeroed for invalid returns; it does not reveal the injected
error or claim a particular DroneCAN wire field.

L1 acquires at `startTime+k*samplePeriod` and delivers after
`latencySamples*samplePeriod`. Delay is an integer number of acquisition periods.
`sequence` counts valid and invalid packets, starting at one on first delivery.
Before that, `valid=false`, `status=0`, `sequence=0`, timestamps=-1. Distance,
quality and status all refer to `sampleTime`, not the current geometry. Outputs
hold between deliveries. L0 is the explicit continuous option.

The error before rounding is
`bias + temperatureCoefficient*(sensorTemperature-referenceTemperature)
 + (noiseStd+noiseSlope*idealRange)*normal`.
Noise parameters are per-sample standard deviations, not spectral densities;
`seed`/`streamId` define a local reproducible stream. `quantum=0` bypasses rounding;
positive quantum rounds to nearest, with exact half-steps toward positive infinity.
All defaults are illustrative. Reflectance/ambient effects can be supplied through
availability/quality scenarios; no optical lighting model is implied.

[`Tests.RangePhysics`](../../../Tests/RangePhysics.mo) checks tilted planes,
normal scaling, invalid returns, temperature/bias/quantization, delayed status and
replay. [`Examples.GnssRange`](../../../Examples/GnssRange.mo) shows direct use.
Range is not added to the legacy four-sensor `SensorMeasurements` record or the
TOML composer. A host adapter must map status, mount orientation and sample timestamps
to its actual device protocol. L2/L3 optics, occlusion, multiple surfaces and flow
remain outside this implementation.
