# ESC_BLE_App

Android BLE autopilot and motor controller for twin-motor boats.  
Controls AC6328/AC6329C BLDC motor drivers via BLE, fuses heading from a
LC02H dual-antenna GNSS compass and MMC5603 magnetometer, and runs a PI
autopilot with adaptive deadband.

---

## Hardware

| Role | Device | BLE Service |
|------|--------|-------------|
| Motor controller + GNSS | AC6329C (GPS_PWM / ESC_PWM / BLDC_PWM) | ae00 or ae30 |
| IMU / Mag sensor (optional) | AC6329C (IMU_PWM) | ae00 or ae30 |
| Remote controller (optional) | BLE remote | ae00 or ae30 |

**Primary device** handles GNSS (A2/A3 packets) and receives motor PWM commands.  
**IMU sensor** sends IMU+Mag data (A1 packets) and is receive-only — no motor
commands. Place it away from motors for best magnetic accuracy.

---

## BLE Protocol

### Service UUIDs
- `ae00-...` or `ae30-...` (dual-service support)

### Characteristics

| UUID | Properties | Direction | Description |
|------|-----------|-----------|-------------|
| ae02 | NOTIFY | Device → App | Sensor data (A1/A2/A3) and motor echo |
| ae03 | WRITE_NO_RESPONSE | App → Device | Motor commands |
| ae04 | NOTIFY | Remote → App | Remote controller commands |
| ae10 | READ / WRITE | Both | Status / mode switch |

### ae02 Packet Formats

**0xA1 — IMU + Magnetometer (50 Hz, 20 bytes)**

| Bytes | Field | Scale |
|-------|-------|-------|
| 0 | 0xA1 | — |
| 1 | Sequence | — |
| 2–3 | ax | raw int16 |
| 4–5 | ay | raw int16 |
| 6–7 | az | raw int16 |
| 8–9 | gx | raw int16 |
| 10–11 | gy | raw int16 |
| 12–13 | gz | raw int16 |
| 14–15 | mx | raw int16 |
| 16–17 | my | raw int16 |
| 18–19 | mz | raw int16 |

**0xA2 — PQTMTAR Heading (1 Hz, 17 bytes)**

| Bytes | Field | Scale |
|-------|-------|-------|
| 0 | 0xA2 | — |
| 1–4 | timeMs | uint32 LE |
| 5 | quality | 0=none, 4=fixed, 6=float |
| 6–7 | solved baseline | uint16 LE × 0.001 m |
| 8–9 | pitch | int16 LE × 0.01° |
| 10–11 | roll | int16 LE × 0.01° |
| 12–13 | heading | uint16 LE × 0.01° (0–360°) |
| 14–15 | acc_heading | uint16 LE × 0.001° |
| 16 | usedSV | uint8 |

**0xA3 — GNRMC Position (0.2 Hz, 17 bytes)**

| Bytes | Field | Scale |
|-------|-------|-------|
| 0 | 0xA3 | — |
| 1–4 | timeMs | uint32 LE |
| 5–8 | rawLat | int32 LE, NMEA × 10000 |
| 9–12 | rawLon | int32 LE, NMEA × 10000 |
| 13–14 | speedKt | uint16 LE × 0.01 kt |
| 15–16 | COG | uint16 LE × 0.01° |

### ae03 Motor Command Format (5 bytes)

```
[CMD, port_lo, port_hi, stbd_lo, stbd_hi]
```

ESC mode: duty 500–1000 (maps 0–100% speed)  
BLDC mode: duty 0–10000

### ae04 Remote Controller Commands (2 bytes: [GROUP, VALUE])

| Group | Value | Action |
|-------|-------|--------|
| 0x10 | 0x01 / 0x02 | Both motors +5% / −5% |
| 0x10 | 0x03 | STOP |
| 0x10 | 0x04 / 0x05 | Both motors +1% / −1% |
| 0x11 | 0x01 / 0x02 | Course −1° / +1° |
| 0x11 | 0x0A / 0x0B | Course −10° / +10° |
| 0x12 | 0x01 / 0x02 / 0x03 | Autopilot ENGAGE / DISENGAGE / HOLD |
| 0x13 | 0–100 | Set both motors absolute % |
| 0x14 | 0x01 / 0x02 | PORT +5% / −5% |
| 0x14 | 0x04 / 0x05 | PORT +1% / −1% |
| 0x14 | 0x10+pct | PORT absolute % |
| 0x15 | same as 0x14 | STBD |
| 0x16 | 0x01 / 0x02 | Sync ON / OFF |

---

## App Architecture

```
MainActivity
  └── scan (all AC6329 devices shown in one list)
       ├── Tap device     → motor controller (primary)
       ├── Tap [🔬 IMU]  → IMU/mag sensor (optional, A1 only)
       └── Scan Remote    → BLE remote (optional)

ControlActivity          — manual throttle + sync + GPS card
AutopilotActivity        — PI autopilot + sea state + Kalman/CF switch
CalibrationActivity      — mag, gyro, LC02H baseline calibration
```

### Source Files

| File | Role |
|------|------|
| `SensorFusion.kt` | Pure Kotlin fusion engine — no Android imports |
| `GpsManager.kt` | Android wrapper: BLE routing, phone GPS, sensor2 |
| `AC6328BleManager.kt` | BLE GATT layer (ae00/ae30 dual service) |
| `MainActivity.kt` | BLE scanner, device role assignment |
| `ControlActivity.kt` | Manual motor control |
| `AutopilotActivity.kt` | PI autopilot |
| `AutopilotLogger.kt` | CSV session logging |
| `CalibrationActivity.kt` | Mag / gyro / baseline calibration |
| `RemoteBleManager.kt` | BLE remote controller client |
| `RemoteManager.kt` | Remote scan/connect helper |

---

## SensorFusion

Pure Kotlin — no Android imports. Safe to unit-test without a device.

### FusedState

```kotlin
data class FusedState(
    val headingDeg:            Float,    // fused heading, true north (0–360°)
    val speedKnots:            Float,
    val latDeg:                Double,
    val lonDeg:                Double,
    val hasHeading:            Boolean,
    val hasFix:                Boolean,
    val satellites:            Int,
    val headingConf:           Float,    // 0.0 (unreliable) … 1.0 (fully trusted)
    val seaState:              Float,    // 0 = calm … 1 = rough
    val tiltDeg:               Float,    // accel-derived tilt from vertical
    val pitchDeg:              Float,    // LC02H PQTMTAR pitch
    val rollDeg:               Float,    // LC02H PQTMTAR roll
    val solvedBaselineM:       Float,    // LC02H solved antenna baseline (m)
    val autoDeadbandDeg:       Float,    // sea-state-adjusted autopilot deadband
    val magCalibrated:         Boolean,
    val rawMagHeadingDeg:      Float,    // tilt-compensated mag, before declination
    val magDeclinationDeg:     Float,    // auto-computed from GPS position
    val magSpikeRejected:      Boolean,  // true when last A1 mag reading was rejected
    val tarMisalignDeg:        Float,    // LC02H mounting offset vs COG
    val tarMisalignCalibrated: Boolean,
    val source:                String,   // e.g. "kf:gnss+imu", "cf:imu+mag"
    val debugMsg:              String
)
```

### Data Flow

```
A1 (50 Hz) — QMI8658C + MMC5603
  ├── Gyro bias correction
  ├── Sea state estimate (az + gxy stddev)
  ├── Tilt-compensated mag heading
  ├── Hard-iron correction (manual cal)
  ├── GPS auto-cal bias
  ├── Magnetic declination (WMM-2020)
  ├── Mag spike rejection (gyro-gated)
  └── Kalman predict + mag update  OR  Complementary filter

A2 (1 Hz) — PQTMTAR heading + pitch/roll + solved baseline
  ├── LC02H mounting misalignment correction
  ├── Solved baseline gate (|solved − reference| > 0.15m → discard)
  ├── Tilt rejection (pitch/roll vs mounting baseline)
  ├── PQTMTAR weight (quality × accuracy × satellites × speed × tilt)
  ├── RMC COG blend
  ├── Heading confidence (signal quality × turn penalty × sea penalty × tilt penalty)
  └── Kalman GNSS update  OR  Complementary filter

A3 (0.2 Hz) — GNRMC position
  ├── Cache COG + speed for A2 blend
  ├── Magnetic declination update
  └── LC02H misalignment calibration

Phone GPS / NMEA
  ├── Position → declination update
  └── COG → heading (when BLE GPS unavailable)
```

### Heading Confidence

`headingConf` (0–1) is used by the autopilot to scale PI output:

```
GNSS source (A2):
  rawConf     = qual × accuracy × satellites × 0.8 + rmc_weight × 0.2
  turnPenalty = 1 − |gyroZ| / 40°/s        (min 0.2)
  seaPenalty  = 1 − seaState × 0.6         (min 0.4)
  tiltPenalty = 1 − tiltDeg / 60°          (min 0.5)
  headingConf = rawConf × turnPenalty × seaPenalty × tiltPenalty

Mag-only source (A1, no recent GNSS):
  base        = 0.6 (calibrated) or 0.3 (uncalibrated)
  headingConf = base × turnPenalty × seaPenalty × tiltPenalty  (max 0.6)
```

### Filter Switch

```kotlin
fusion.useKalman = false   // Complementary filter (default, proven)
fusion.useKalman = true    // Multirate Kalman (experimental)
```

Toggle from the autopilot tuning card at runtime. Switching resets both filters.

**Kalman tuning:**
```kotlin
fusion.kalman.sigmaGyro  = 0.5f    // gyro measurement noise (°/s)
fusion.kalman.sigmaDrift = 0.005f  // gyro bias random walk (/s)
fusion.kalman.sigmaMag   = 15f     // mag measurement noise (°)
fusion.kalman.sigmaGps   = 2f      // GPS base noise (°)
```

### Magnetic Declination

Auto-computed from GPS position using WMM-2020 simplified dipole model (±1°
accuracy). Updates when position changes >0.5°. Persisted to `calibration_prefs`
key `mag_declination_deg`. Sources: A3 BLE GPS and phone GPS NMEA.

Manual override:
```kotlin
fusion.setDeclination(-14.5f)   // degrees East positive
```

### LC02H Baseline System

Three tiers, all persisted in `calibration_prefs`:

| Variable | Meaning | Key |
|----------|---------|-----|
| `measuredBaselineM` | User-set physical separation (default 1.0 m) | `lc02h_measured_baseline` |
| `calibratedBaselineM` | Averaged from 60 static A2 samples | `lc02h_calibrated_baseline` |
| `referenceBaselineM` | = calibrated if solved, else measured | — |

**Gate rule:** if `|solvedBaseline − referenceBaseline| > 0.15 m` → `wTar = 0`
(PQTMTAR heading discarded for this A2 tick).

**To calibrate:** dock the boat, go to Calibration → LC02H ANTENNA BASELINE →
tap START. After 60 stable seconds the calibrated value is saved and displayed as
"Update LC02H config to: X.XXX m".

```kotlin
fusion.baselineToleranceM  = 0.15f   // adjust if too strict/loose
fusion.maxTiltForGnssDeg   = 25f     // tilt beyond which PQTMTAR is zeroed
fusion.pitchBaselineDeg    = 0f      // mounting pitch offset
fusion.rollBaselineDeg     = 0f      // mounting roll offset
```

### Mag Spike Rejection

```kotlin
fusion.magSpikeMultiplier = 2.5f   // max allowed delta = |gyroZ × dt × 2.5| + 2°
```

If a mag reading exceeds this bound AND the delta is >5°, the sample is rejected
and the previous accepted value is held. Shows `⚡SPIKE` in the debug log.

---

## Autopilot

PI controller with adaptive deadband and heading confidence scaling.

```
error  = targetHeading − actualHeading  (wrapped ±180°)

if |error| < deadband:
    integral × 0.8   (decay, don't reset — preserves bias compensation)
    motors = baseSpeed (equal)

else:
    integral = clamp(integral + error × dt, ±MAX_INTEGRAL)
    rawDiff  = (error × Kp + integral × Ki) × headingConf
    diff     = clamp(rawDiff × speedFactor, ±min(baseSpeed, 100−baseSpeed))
    PORT     = baseSpeed + diff
    STBD     = baseSpeed − diff

deadband = baseDeadbandDeg + seaState × 8°  (when autoDeadband enabled)
```

**Default gains** (Trimaran): Kp=0.5, Ki=0.02, deadband=2°  
**Default gains** (Monohull): Kp=1.2, Ki=0.08, deadband=4°

Gains are saved to `autopilot_prefs` and survive app restart.

---

## Autopilot Logging

Started automatically on ENGAGE, stopped on DISENGAGE.  
File: `Downloads/AP_yyyyMMdd_HHmmss.csv`

| Row type | Rate | Key columns |
|----------|------|-------------|
| `CTRL` | 5 Hz | target/actual/mag heading, error, deadband, in_deadband, P/I terms, integral, raw_diff, final_diff, port/stbd %, conf, sea_state, lat/lon, filter, kalman_bias |
| `SENS` | 10 Hz | ax/ay/az, gx/gy/gz, mx/my/mz, raw_mag_hdg, fused_hdg, gyro_z_dps, tilt_deg, sea_state |
| `GPS` | 1 Hz | PQTMTAR heading, RMC COG, blended, quality, acc_deg, satellites, speed, lat/lon, misalign |
| `CMD` | 5 Hz | port/stbd %, duty counts, ESC/BLDC mode |

All row types share one wide header row. Load in Python:
```python
import pandas as pd
df = pd.read_csv('AP_20250101_120000.csv')
ctrl = df[df.type == 'CTRL']
ctrl.plot(x='timestamp', y=['target_hdg', 'actual_hdg', 'heading_error'])
```

Sensor row rate:
```kotlin
apLogger.senseSampleHz = 10   // default; set 0 to disable, 50 for full rate
```

---

## Calibration

All calibration data persisted to `calibration_prefs` SharedPreferences.

### MMC5603 Magnetometer (Hard-Iron)

Rotate boat slowly 360° at dock. Records min/max on X and Y axes.  
Keys: `mag_hard_iron_x`, `mag_hard_iron_y`, `mag_cal_done`

### QMI8658C Gyro Bias

Hold sensor still for 5 seconds. Averages gz over samples.  
Keys: `gyro_bias_x`, `gyro_bias_y`, `gyro_bias_z`, `gyro_cal_done`

GPS auto-calibration of mag bias runs automatically at sea when speed >2 kt
and sea state is calm. No user action needed.

### LC02H Antenna Baseline

Hold boat still at dock for 60 seconds (gyro-gated, rejects movement).  
Key: `lc02h_calibrated_baseline`, `lc02h_baseline_solved`

After calibration, the UI shows the value to enter in the LC02H firmware config.

### Magnetic Declination

Auto-computed from GPS. Persisted: `mag_declination_deg`

---

## Status Bar Icons

| Icon | Device | Color |
|------|--------|-------|
| ⚡ | Motor controller (primary, always required) | Cyan |
| 🔬 | IMU/mag sensor (A1 only, optional) | Amber |
| 🕹 | BLE remote (optional) | Blue |

---

## Permissions Required

```xml
BLUETOOTH_SCAN
BLUETOOTH_CONNECT
ACCESS_FINE_LOCATION      <!-- required for BLE scan on Android 6–11 -->
WRITE_EXTERNAL_STORAGE    <!-- autopilot log to Downloads/ -->
```

---

## Build

- **minSdk:** 23  
- **Target:** 34  
- **Language:** Kotlin  
- **BLE library:** Nordic BLE Library v2.7  
- **UI:** ViewBinding + Jetpack (no Compose)  
- **Package:** `com.escbleapp`

```bash
./gradlew assembleDebug
```
