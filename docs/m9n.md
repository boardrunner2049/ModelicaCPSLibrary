# M9N GNSS receiver

`GNSS.Device` connects the existing G0 `GNSS.Sensor` physics to the optional
`GNSS.VendorConfig.M9N` stage for u-blox NEO-M9N SPG 4.04. The sensor suite
applies mounting and G0 applies biases once. The receiver consumes measured NED position and
velocity. It owns sampling, output validity, static hold and live configuration.
SITL decodes requests and packs replies, using the existing virtual GPS UART
and the production ArduPilot u-blox driver.

## Selection and settings

In Compose, select **G0** physics and **u-blox NEO-M9N** in **Device**.
The device fields configure the receiver layer; physics bias and mounting
remain separate. The GUI starts scenario PDOP at 1.0; the Modelica parameter's
default remains 2. Restart a running Compose server after updating its catalog.

```toml
[models.gnss]
model = "ideal"
vendor = "m9n"

[gnss]
positionBias = [0.2, 0.0, 0.0] # Metres in local NED, applied by physics.

[gnss.m9n]
samplePeriod = 0.2
reportedPdop = 1.0
pdopLimit = 3.0
staticHoldSpeedThreshold = 1.0
staticHoldMaxDistance = 2.0
```

`configs/quad_m9n.toml` is the complete example. The binding supports explicit
G0 plus M9N on multirotor `vio` exports. G1 is rejected because it already owns
sampling/latency and would require a shared acquisition contract. Omitting
`vendor` leaves the original physics path unchanged. Part aliases do not select
this wrapper. In Modelica, redeclare `GNSS.Device.VendorConfigModel` as
`GNSS.VendorConfig.M9N`; physics parameters stay on `Device`, receiver parameters
on `vendorConfig`.

| Parameter | Initial value | Supported domain |
|---|---|---|
| `samplePeriod` | 0.2 s | At least 0.04 s; select before export |
| `reportedPdop` | 2 | Scenario PDOP, 0..655.35; satellite geometry is unmodeled |
| `pdopLimit` | 3 | Positive U2 in 0.1 steps: 0.1..6553.5 |
| `staticHoldSpeedThreshold` | 0 m/s | U1 in 0.01 m/s steps: 0..2.55; zero disables hold |
| `staticHoldMaxDistance` | 0 m | Integer U2: 0..65535; choose positive distance when hold is enabled |

Parameters initialize the receiver. Accepted delivered commands change live
settings; they do not rewrite parameters. Acquisition begins at simulation time
zero. iTOW is simulation time rounded to milliseconds modulo one GPS week;
week number and UTC date are unmodeled.

## Measurements

The output filter accepts only `reportedPdop < livePdopLimit`. Equality clears
`gnssFixOK`; receiver fix type stays 3D. Invalidity releases static hold. It
does not invent noise: with hold disabled the measured position/velocity remain
unchanged. Other navigation output filters are assumed to pass.

Static hold uses the integration manual's Figure 3 state transitions. The first
sample supplies velocity history; two subsequent quiet samples enter hold.
Entry requires speed below the threshold and acceleration below its entry limit.
Hold freezes all three position axes and reports zero velocity. It releases at
the current acquisition if speed, acceleration or displacement exceeds its exit
limit; equality remains held. Disabled hold or invalid output releases it too.

For speed accuracy `sAcc > 2*threshold`, entry/exit acceleration factors are
0.6/1 per second and exit speed is 3*threshold. Otherwise they are 0.4/0.7 per
second and exit speed is 2*threshold. Here sAcc is the fixed 0.1 m/s fixture.
Acceleration is a finite difference of measured 3D velocity; speed/displacement
use 3D norms. These are approximations of unpublished internal estimates, not
hardware-qualified receiver filtering. Positive distance is required because
the firmware-default distance behavior at zero is unmodeled.
Figure 3 also shows a 200 m displacement check; its interaction with the
configurable distance threshold is unresolved. Static-hold behavior is qualified
only below 200 m of unheld estimated displacement from the hold position.

Position, velocity, quality and time are acquired together. Quality fixtures are
10 satellites, 1 m horizontal/vertical accuracy, 0.1 m/s speed accuracy,
180 degree course accuracy and zero geoid separation. Course and horizontal
speed follow reported velocity, including the acquisition that releases hold.
Course is zero at zero speed; the proprietary low-speed course freeze is unmodeled.
Geodetic conversion retains the existing spherical approximation. Zero geoid
separation assumes ellipsoid and MSL heights coincide.

## Configuration commands

SITL validates framing/checksums and transfers decoded requests in order.
Modelica processes commands independently of navigation acquisition and replies
from the same live state used by measurements. Changes affect the next
acquisition. The UART retains later requests until completion.

| Request | Supported response/effect |
|---|---|
| Empty CFG-NAV5 poll | Complete readback of live settings and fixed fields |
| NAV5 mask 0x01 | Dynamic-model write/readback: 0 or 2..9; filtering unmodeled |
| NAV5 mask 0x02 | Minimum-elevation write/readback; satellite selection unmodeled |
| NAV5 mask 0x10 | PDOP limit; position-accuracy limit must remain 100 m |
| NAV5 mask 0x40 | Atomic static-hold speed/distance write |
| Empty CFG-PRT poll | Current UART1: 230400 bit/s, 8N1, UBX input/output, TX-ready disabled |
| Empty MON-VER poll | Synthetic CORE/SPG 4.04, protocol 32.01, hardware 00190000, NEO-M9N |

Unselected fields stay unchanged. Unsupported shapes/groups/settings apply no
partial writes and produce emulator status 4, not a receiver ACK or NAK. CFG
polls send readback then ACK; accepted writes ACK; MON-VER replies without ACK.
Readback mask zero and readback-before-ACK are unmeasured conventions. Replies
retry under backpressure without duplication. An incomplete advertised payload
has no parser timeout. Valid unsupported requests remain visible coverage gaps.

## Binding and layouts

The optional `gnss_ubx_pvt`, `gnss_ubx_command` and `gnss_ubx_response`
Float64 vectors belong to `gnss_v1` and must be exported
together. Ordinary G0/G1 exports omit them. Required `gps`/`vel_ned` and PVT
describe the same acquisition. Indices below are Modelica's one-based indices.

| `gnss_ubx_pvt[14]` index | Value |
|---|---|
| 1, 2, 3 | Acquisition sequence (0 before first), acquisition time [s], iTOW [ms] |
| 4, 5, 6 | Receiver fix type, complete PVT flags, satellites used |
| 7, 8, 9, 10 | hAcc [m], vAcc [m], sAcc [m/s], PDOP |
| 11, 12, 13, 14 | Ground speed [m/s], course [deg], course accuracy [deg], ellipsoid-minus-MSL [m] |

`gnss_ubx_command[20]` and NAV5 response share this layout:

| Index | Value |
|---|---|
| 1, 2, 3, 4 | Request ID, UBX class, message ID, operation/status |
| 5, 6, 7 | NAV5 mask, dynamic model, fix mode |
| 8, 9, 10 | Fixed altitude [m], altitude variance [m2], minimum elevation [deg] |
| 11, 12, 13, 14 | Position/time DOP limits, position/time accuracy limits [m] |
| 15, 16, 17, 18 | Static speed [m/s], DGNSS timeout [s], CNO count, CNO threshold [dBHz] |
| 19, 20 | Static distance [m], UTC standard |

Operations are 0 poll, 1 write, 2 unsupported shape. Response status is 0 initial,
1 readback, 2 accepted write, 4 emulator coverage gap. ID/class/message echo the
request. Request IDs are exact integers 1..INT32_MAX; zero is initial idle only.
CFG-PRT response indices 5..12 contain portID, TX-ready, UART mode, baud, input
mask, output mask, flags and reserved. MON-VER indices 5..10 contain decimal
hardware digits, CORE major/minor, protocol major/minor and module family.
Remaining port/identity fields are zero.

On an initialized runtime call `cpv_set_gnss_ubx_command(vehicle, command, 20)`,
advance a positive step and read `gnss_ubx_response` with `cpv_get_field`.
Await the matching ID before replacing the command. The typed writer checks
the fixed GNSS endpoint, size and finite values; it advances no time. Other
runtime fields and the original linked-model/JSON descriptor layout are unchanged.

Apply `runtime/hosts/ardupilot/hooks.patch` then `gnss.patch` at the pinned
ArduPilot revision; `build.sh` handles these. Use a clean pinned tree or the
current patch set; migration from older experimental patches is unsupported.
Rebuild the model/runtime before
SITL, which statically links them. Use GPS1_TYPE=2 and SIM_GPS_TYPE=1. One
receiver, instance zero, uses model-owned UBX; stock paths retain their original
header/payload/checksum writes. Model-owned output bypasses generic SIM_GPS
noise, lock, offset, antenna, delay, jamming and byte loss to avoid double effects.
Each new acquisition publishes once; skipped/reset sequences or insufficient
PVT capacity are simulation errors. Observe more frequently than samplePeriod.

Periodic output is NAV-PVT only. Without NAV-DOP, ArduPilot 4.6.2 uses PVT PDOP
as its HDOP fallback: reportedPdop=2 becomes HDOP 200, above the usual arming
limit 140. Choose a healthy scenario explicitly when testing flight. A rejected
gnssFixOK with ideal P/V does not itself establish flight deterioration.

Full startup, rate/message scheduling, CFG-GNSS, save/load/reset, RF tracking,
proprietary filtering and other periodic messages are unmodeled. Supported NAV5
settings can be queried/corrected; complete receiver startup is not qualified.
SITL's configuration-complete shortcut prevents hardware-equivalent prearm
claims. Production drivers, estimators, prearm checks and flight logic are unchanged.

## Checks and sources

`tools/test_m9n.py` checks native Modelica behavior with FIRE_TEST_OMC=1 and
actual patched UBX publisher/listener code with FIRE_TEST_M9N_ENCODER=1.
The existing FMI harness is `Systems.Sensing.GNSS.Device`; runtime conformance
checks both descriptor and linked receiver commands. These checks do not
establish hardware fidelity or replace closed-loop flight validation.

ArduPilot pin: Copter 4.6.2, `1ebd4d996e8b6bbeb1462a9655c00aa861d5beef`.
Closest active FIRE: `2b7b3a35983777988b66bbacf90c91bdda72f311`, GNSS.Sensor
and SensorSuite; it has no equivalent vendor receiver boundary. Other controllers
require separate bindings.

- NEO-M9N Integration manual, UBX-19014286 R10, pp. 9–14 and Figure 3:
  acquisition, validity and static hold.
- M9 SPG 4.04 Interface description, UBX-21022436 R02, pp. 48/54, 65–66,
  73–75, 141–143, 203/217/236, 242/250–251: ACK/poll, NAV5, UART, PVT,
  configuration mappings/defaults.
- NEO-M9N-00B Data sheet, UBX-19014285 R08, p. 4: maximum navigation rate.
