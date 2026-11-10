# Parts: real devices as aliases of the sensor models

Each file here is one real device: its datasheet figures as published, with
the document they came from. A configuration names a device instead of
writing model parameters, and the composer (`tools/fire-compose`) picks the
level and fills its parameters from the figures:

```toml
[models.gnss]
part = "ublox_neo_m9n"   # an id, a name ("u-blox NEO-M9N"), a unique piece of one ("m9n"), or a path
level = "G1"             # optional; default the highest level a datasheet fills (I2, G1, B1, M1)
mode = "sbas"            # optional: gnss standalone | sbas | rtk; barometer low_noise | fast
tolerances = "draw"      # optional: draw offsets, scale and cross-axis errors within the
                         #   datasheet's ± tolerances, deterministically from [KIND].seed
[models.imu]
part = "tdk_icm42688p"
range = { accel_g = 8, gyro_dps = 1000 }   # optional: one of the part's ranges

[gnss]
samplePeriod = 0.1       # anything set explicitly wins over the part
```

See what a device becomes, at any level, with every parameter's origin, every
assumption, and every figure the level cannot represent:

```sh
tools/fire-compose/target/debug/fire-compose part gnss m9n
tools/fire-compose/target/debug/fire-compose part imu icm42688 --level I1
tools/fire-compose/target/debug/fire-compose part barometer bmp388 --tolerances draw --json
```

The same report goes into the composed model's manifest (`config.parts`), and
Compose (`tools/compose`) offers the devices under each sensor.

## The conversions

One converter per kind (`tools/fire-compose/src/parts.rs`); a new device is
just a new file.

| Kind | Figures | Parameters |
|---|---|---|
| GNSS (G1) | CEP 50 % (per mode), vertical median, 50 % velocity accuracy, max navigation rate | position σ = CEP / 1.1774, split into a Gauss-Markov bias (90 %, τ 60 s; RTK 10 s) and white noise; vertical σ = median / 0.6745 (1.5 × horizontal when not stated); velocity σ = 50 % / 0.6745, white. The sample rate is checked against the maximum. |
| IMU (I1, I2, I3) | range, ADC bits, noise densities, bias instability, ODR | `fullScale`; `quantum` = 2 × range / 2^bits; `noiseStd` per ADC tick = density × √(f_tick / 2); gyro `biasStd` = bias instability. The ADC rate is checked against the ODR. |
| Barometer (B1) | RMS noise per mode, resolution, relative accuracy, conversion time | `pressureNoiseStd`, `pressureQuantum`, drift σ = relative accuracy / 2 (τ 300 s), `temperatureQuantum`; the sample period is at least the conversion time. |
| Magnetometer (M1) | range, resolution, RMS noise, ODR at that noise | `fullScale`, `quantum`, `noiseStd`. |
| Tolerances (`tolerances = "draw"`) | zero-g / zero-rate offsets, sensitivity tolerance, cross-axis, offset tempcos, g-sensitivity; absolute pressure and temperature accuracy; magnetometer offset | `accelBias`, `gyroBias`, `scale`, misalignment, `temperatureCoefficient`, `gyroGSensitivity`; `pressureBias`, `temperatureBias`; `bias`, `softIron`: each drawn uniformly within ±. |

What the models do not have (acquisition time, constellation and signal
models, nonlinearity, scale temperature coefficients, ...) is listed per
figure, so the gaps say which models to add next. Lower levels keep only what
they can represent (`--level G0` of a GNSS receiver keeps nothing but reports
every figure it dropped).

## Powertrain parts: motors, propellers, batteries, ESCs, servos, gearmotors

A propulsion level is an assembly, so it names several parts in one table:

```toml
[models.propulsion]              # multirotor: sets models.esc/motor/rotor and fills
motor = "tmotor_mn3508_kv580"    #   [motor], [rotor], [power], [esc]
propeller = "apc_12x45mr"        # optional when the motor's bench table has a propeller
battery = "tattu_4s_5200_35c"    # or [power] dcVoltage
esc = "holybro_kotleta20"        # optional: limits; switch figures at P3/P4
level = "P2"                     # P0..P4 (default P2)
battery_state = "nominal"        # full | nominal | storage | empty

[models.servos]                  # plane control surfaces; a rover's [models.steering]
part = "kst_x08_v6"
level = "S1"
voltage = 6.0                    # optional: which rated voltage's speed

[models.drive]                   # rover
part = "pololu_37d_50to1_12v_4753"
count = 2                        # gearmotors on the axle
battery = "gensace_3s_2200_25c"  # optional; default the rated voltage
```

```sh
fire-compose part motor mn3508 --with propeller=12x45mr --with battery=tattu_4s --mass 2.0
fire-compose part servo sc1258 --with vehicle=rover
fire-compose part gearmotor 37d_50 --with count=2 --level D0
```

| Model | Figures | Parameters |
|---|---|---|
| Motor | KV, internal resistance, no-load current at V, poles, inductance and rotor inertia if stated | k = 60/(2π KV); R line-to-line (six-step, two phases conduct); b = k I0/ω0 (no-load loss as viscous friction); P3/P4 per phase R/2, L/2, ke = k/2, poles/2. Not stated: L = 0.2 ms × R, rotor inertia = 40 % of the mass on a ring scaled from a 2212 bell (assumed, reported). |
| Propeller | APC static rows (V = 0) of the PER3 data, or measured thrust/torque per rpm | kT, kQ by least squares over the rows up to the RPM limit; J = m D²/24 (assumed). Without a propeller part, the motor's bench table is fitted instead. |
| P0 / P1 | the above + supply | the loaded speed ω(V) solved from k(V − kω)/R = kQ ω² + bω, fitted linearly over 30–100 % of the supply (`omegaMax`, `Kv`); time constant J/(b + k²/R + 2kQω) at hover. |
| Battery | cells, cell voltages, internal resistance | dcVoltage = cells × cell voltage at `battery_state`; P3/P4 sourceResistance (DC before AC impedance; not stated: 15 mΩ·Ah per cell, assumed). |
| ESC (P3/P4) | per-phase inverter resistance or MOSFET Rds(on), diode Vf, PWM frequency, dead time, capacitor | Ron, Rdiode, Vdiode, pwmFrequency, deadTime, dcLinkCapacitance; each assumption is reported. |
| Servo (S1) | speed per 60° at a voltage, dead band, angle per µs | maxServoRate/maxSteerRate through the linkage that maps full command to maxAileron/maxSteerAngle; dead band as that fraction of the surface range. |
| Gearmotor (D0/D1) | output-shaft no-load speed, stall torque and current, ratio, rated voltage | motorNoLoadSpeed, motorStallTorque (× count, scaled to the supply), gearRatio, supplyVoltage. |

The report's `checks` set the result against the datasheets: full throttle
and hover (thrust/weight, throttle, current), the motor's, ESC's, battery's
and propeller's limits, and the model's speed, thrust and current at each row
of the manufacturer's bench table (by throttle, or at the row's current).

## Writing a part

Use only figures the datasheet states, in the units of the key name, with the
condition and page in a comment; omit what it does not state (the report
lists it as "not stated"). Put caveats in `notes`. The keys each converter
reads are the ones in the files here.

| Kind | Devices |
|---|---|
| gnss | u-blox NEO-M8N, NEO-M9N, ZED-F9P (standalone, SBAS, RTK) |
| imu | TDK InvenSense ICM-42688-P, ICM-20602; Bosch BMI088 |
| barometer | Bosch BMP388, TE MS5611-01BA03, Infineon DPS310 |
| magnetometer | iSentek IST8310, PNI RM3100, QST QMC5883L |
| rangefinder | Benewake TFmini Plus, TF02-Pro; LightWare SF11/C; Garmin LIDAR-Lite v3HP; MaxBotix MB1242 |
| airspeed | TE MS4525DO; Sensirion SDP33; Amphenol All Sensors DLVR-L10D |
| optical_flow | PixArt PMW3901, PAW3902; Matek 3901-L0X; ARK Flow; Cheerson CX-OF |
| rpm | Hobbywing RPM sensor; Honeywell SS41F; Melexis US5881 |
| power_module | Holybro PM02 V3, PM02D; Mauch HS-200-HV |
| temperature | TE TSYS01; Microchip MCP9600; Sensirion SHT31 |
| external_ahrs | VectorNav VN-100, VN-300; MicroStrain 3DM-GX5-25, 3DM-GQ7; Inertial Labs INS-U |
| visual_odometry | Intel RealSense T265; ModalAI VOXL 2 VIO; Luxonis OAK-D |
| beacon | Marvelmind Super-MP-3D; Nooploop LinkTrack P-A |
| precision_landing | IR-LOCK sensor + MarkOne; OpenMV Cam H7 R2 |
| gimbal | SIYI A8 mini; Gremsy Pixy U, T3V3 |
| gripper | NicaDrone OpenGrab EPM v3; Zubax FluxGrip FG40; Tarot TL2962 |
| winch | Okaya Daiwa winch; Viewpro DW-5 |
| parachute | Fruity Chutes Iris Ultra 48" compact; Skycat X55 |
| sprayer | Hobbywing combo pump 5 L, 8 L; TeeJet XR11001 nozzle |
| landing_gear | Tarot TL65B44, TL8X001 |
| tether | Elistair Safe-T 2; Hoverfly LiveSky Sentry |
| generator | Intelligent Energy IE-SOAR 800 W; Loweheiser HYBGEN 32 |
| frame | Holybro X500 V2, S500 V2; DJI Flame Wheel F450 (wheelbase: armLength) |
| arm_tube | Easy Composites carbon tube 16x14, 20x18, 25x23 mm (mass per metre, OD) |
| battery (smart) | Tattu Plus 16000 mAh 6S, 22000 mAh 6S (`smart = true`: SMBus) |

## Peripheral parts

The peripherals (`[peripherals.KIND] part = ...`, `Systems/Peripherals`) are
converted by `tools/fire-compose/src/peripherals.rs`: rates to sample periods
(at most 100 Hz, the drivers' polling), ± accuracies to 2-sigma noise or
drift, 1-sigma repeatability to noise, resolutions and LSBs to quanta,
ranges to limits, noise densities over the output bandwidth. As for the
sensors, every figure the level cannot represent and every figure the
datasheet does not state is reported:

```sh
tools/fire-compose/target/debug/fire-compose part rangefinder tfmini
tools/fire-compose/target/debug/fire-compose part external_ahrs vn_300 --json
```

Payloads (`[payloads.KIND] part = ...`, `Systems/Payloads`) are converted by
`tools/fire-compose/src/payloads.rs`: line speeds and lengths, canopy
diameters and drag coefficients, pump flows and PWM ranges, gimbal angle
ranges and rates, tether lengths and powers, generator ratings and fuel use.

