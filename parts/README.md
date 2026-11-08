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
the vehicle builder offers the devices under each sensor.

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
