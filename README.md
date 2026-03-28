# ESC_BLE_App

Android BLE controller for dual **5062 BLDC** motors with GPS heading autopilot.  
Uses [Nordic Android-BLE-Library](https://github.com/NordicSemiconductor/Android-BLE-Library) v2.7.

**Min SDK:** 23 · **Target SDK:** 34 · **Kotlin**  
Requires a physical Android device with BLE + Location permission.

---

## Firmware Repositories

| Chip | Role | Repo |
|------|------|------|
| **AC6328** | ESC / BLDC motor PWM only | [AC6328_PWM](https://github.com/mikewen/AC6328_PWM) |
| **AC6329C** | PWM + LC02H GNSS + MMC5603 mag + QMI8658C IMU | [AC6329_PWM_GPS](https://github.com/mikewen/AC6329_PWM_GPS) · [AC6329_IIC](https://github.com/mikewen/AC6329_IIC) |

---

## Hardware

```
AC6328 / AC6329C
  IO_PORT_DM ──► ESC/BLDC driver ──► PORT motor (left)    JL_TIMER3
  IO_PORT_DP ──► ESC/BLDC driver ──► STBD motor (right)   JL_TIMER2

AC6329C additional sensors:
  UART ──► LC02H dual-antenna GNSS module
             $PQTMTAR  absolute heading (RTK fixed / dead-reckoning)
             $GNRMC    speed, course-over-ground, position
  I2C  ──► MMC5603 magnetometer
  SPI  ──► QMI8658C 6-DoF IMU (accel + gyro)
```

### PWM Duty Scale

Both ESC and BLDC use the **same 50 Hz PWM timer** — values are duty counts, not µs:

```
ESC:   duty 500–1000   →   500×2µs=1000µs (stop) … 1000×2µs=2000µs (full)
BLDC:  duty 0–10000    →   0% … 100%   (1 duty = 0.01%)
```

---

## BLE Profile (Service `ae00`)

| UUID | Properties | Direction | Role |
|------|-----------|-----------|------|
| `ae03` | WRITE_WITHOUT_RESPONSE | App → Chip | Motor command |
| `ae02` | NOTIFY | Chip → App | Sensor data + command echo |
| `ae10` | READ \| WRITE | Both | Status / mode switch |

### ae03 — Motor command (5 bytes, little-endian)

```
[CMD]  [port_lo]  [port_hi]  [stbd_lo]  [stbd_hi]
```

| CMD byte | Value | port/stbd range |
|----------|-------|----------------|
| `CMD_ESC_PWM`   | `0x01` | 500–1000 duty |
| `CMD_BLDC_DUTY` | `0x02` | 0–10000 duty  |
| `CMD_STOP`      | `0xFF` | ignored       |

### ae02 — Sensor stream + echo

The AC6329C sends two kinds of data on ae02:

**1. Motor command echo** — AC6329C echoes every ae03 command back as a 5-byte packet
with the same format `[CMD, port_lo, port_hi, stbd_lo, stbd_hi]`.  
The app detects these (CMD ∈ {0x01, 0x02, 0xFF}, size=5) and **discards silently**.

**2. Sensor fusion packets** — binary little-endian:

#### `0xA1` — IMU + Magnetometer (50 Hz, 20 bytes)

```c
ble_pkt[0]  = 0xA1;
ble_pkt[1]  = ble_seq++;
/* Accel */  ble_pkt[2..7]  = ax, ay, az  (int16 LE, QMI8658C LSB)
/* Gyro  */  ble_pkt[8..13] = gx, gy, gz  (int16 LE, QMI8658C LSB)
/* Mag   */  ble_pkt[14..19]= mx, my, mz  (int16 LE, MMC5603 LSB)
```

#### `0xA2` — LC02H PQTMTAR heading (1 Hz, 17 bytes)

| Offset | Type | Field | Scale |
|--------|------|-------|-------|
| 0 | u8 | `0xA2` | |
| 1–4 | u32 LE | UTC time ms | |
| 5 | u8 | quality | 0=none, 4=RTK fixed, 6=DR |
| 6–7 | — | reserved | |
| 8–9 | i16 LE | pitch | ÷100 → ° |
| 10–11 | i16 LE | roll | ÷100 → ° |
| 12–13 | u16 LE | heading | ÷100 → ° (0–360) |
| 14–15 | u16 LE | acc_heading | ÷1000 → ° accuracy |
| 16 | u8 | usedSV | satellites |

#### `0xA3` — LC02H GNRMC position (0.2 Hz, 17 bytes)

| Offset | Type | Field | Scale |
|--------|------|-------|-------|
| 0 | u8 | `0xA3` | |
| 1–4 | u32 LE | UTC time ms | |
| 5–8 | i32 LE | rawLat | NMEA DDMM.MMMM × 10000 |
| 9–12 | i32 LE | rawLon | NMEA DDDMM.MMMM × 10000 |
| 13–14 | u16 LE | speedKnots | ÷100 |
| 15–16 | u16 LE | cogDeg | ÷100 → ° course-over-ground |

> **A1 is optional.** Without it the app uses GNSS-only heading from A2+A3.
> Sea state and mag calibration are inactive; autopilot still functions.

### ae10 — Mode switch / status

```
WRITE:  0x01 = ESC mode    0x02 = BLDC mode
READ:   "M<mode>A<vbat_mv>T<uptime_min>"   e.g. "M1A3712T5"
```

---

## Code Structure

```
app/src/main/java/com/escbleapp/
│
├── SensorFusion.kt          ★ Portable pure-Kotlin fusion engine
├── GpsManager.kt            Android wrapper: BLE routing + phone GPS
│
├── AC6328BleManager.kt      BLE protocol layer
├── MainActivity.kt          BLE scanner + dual-device selection
├── ControlActivity.kt       Manual throttle UI + GPS card
├── AutopilotActivity.kt     PI autopilot + sea state + sensor status
├── CalibrationActivity.kt   Dock-side MMC5603 + QMI8658C calibration
├── MapPickerActivity.kt     OSMDroid waypoint picker
└── CompassView.kt           Custom compass rose widget
```

---

### `SensorFusion.kt` — Portable Fusion Engine

**No Android imports. Pure Kotlin math. Copy to any project.**

Inputs → one callback output (`onFusedHeading: (FusedState) -> Unit`):

```
processA1(ax,ay,az, gx,gy,gz, mx,my,mz, nowMs)   50 Hz IMU+Mag
processA2(tarHdg, pitch, roll, acc, quality, sats, nowMs)   1 Hz GNSS heading
processA3(lat, lon, speedKt, cogDeg, hasFix)   0.2 Hz position + COG
processCasicNav2Sol(x,y,z, vx,vy,vz, sAcc, fixOk, fixType)   CASIC binary
processNmeaRmc(speedKt, cogDeg, hasFix, lat, lon)   standard NMEA
processNmeaGga(sats, alt, fixQual, lat, lon)
```

`FusedState` fields:

| Field | Description |
|-------|-------------|
| `headingDeg` | Fused heading 0–360° |
| `speedKnots` | Speed over ground |
| `latDeg` / `lonDeg` | Position |
| `headingConf` | Confidence 0–1 |
| `seaState` | Sea roughness 0–1 (from az variance) |
| `autoDeadbandDeg` | Recommended autopilot deadband |
| `magCalibrated` | GPS auto-cal of MMC5603 complete |
| `tarMisalignDeg` | LC02H mounting offset (auto-detected) |
| `tarMisalignCalibrated` | Mounting offset calibrated |

#### Heading Fusion Logic

```
0xA1 (50Hz):
  gz × gyroScale → gyro yaw rate
  tilt-compensate MMC5603 mag using accel pitch/roll
  mag weight = 0.02 × tiltFactor × gnssFactor   (gyro-dominant)
  complementary filter: heading += gyroRate×dt + magWeight×(mag−heading)

0xA2 (1Hz):
  Apply install misalignment correction to PQTMTAR
  wTar = qualityFactor × accFactor × satFactor × speedFactor
  wRmc = speed ramp: 0 @ 0.5kt → 1.0 @ 2kt     (COG unreliable when slow)
  blended = normalize(wTar, wRmc) → wrap-safe weighted average
  filterWeight = 0.05 + confidence×0.15
  heading corrected toward blended at filterWeight per second

0xA3 (0.2Hz):
  Cache cogDeg + speedKt for A2 blending
  Update lat/lon/fix for trip distance + waypoint bearing
  Trigger LC02H misalignment auto-cal if speed > 4kt + calm sea
```

#### Sea State (#1)

Rolling 100-sample high-pass filtered variance of `az`:
```
az_hp = az - LPF(az, α=0.99)      // remove gravity DC
seaState = clamp((stddev(az_hp) − 200) / 1800, 0, 1)
```
0 = calm, 1 = rough. Updated at 50 Hz from A1 packets.

#### GPS Auto-Calibration of MMC5603 (#2)

Conditions: `speed > 2kt` AND `seaState < 0.15` AND `gpsConf > 0.75`
```
bias = GPS_heading − mag_heading   (wrap-safe)
accumulate 30 samples
if stddev(bias) < 5°:  magBiasEstimate = mean → magCalibrated = true
```
Once calibrated: mag base filter weight increases 0.02 → 0.05.

#### Automatic Deadband (#3)

```kotlin
autoDeadbandDeg = baseDeadbandDeg + seaState × seaDeadbandScale
                = 2° + seaState × 8°    // calm=2°, rough=10°
```
`AutopilotActivity.onGpsUpdate()` reads this and updates `deadbandDeg` each GPS tick.

#### LC02H Install Misalignment Auto-Cal (#4)

Conditions: `speed > 4kt` AND `seaState < 0.15`
```
bias = COG − PQTMTAR_heading   (wrap-safe)
accumulate 30 samples
if stddev < 3° AND |mean| < 15°:  tarMisalignEstimate = mean
```
`|mean| < 15°` guard: large offsets (rotated/flipped antenna) need physical correction.

---

### `GpsManager.kt` — Android Wrapper

Routes BLE bytes to `SensorFusion` and manages phone GPS:

```
feedAe02Bytes(bytes):
  size=5 AND cmd∈{0x01,0x02,0xFF} → motor echo, discard
  bytes[0]=0xA1/A2/A3             → parseAcPacket() → fusion.processA1/A2/A3()
  bytes[0]=0xBA,0xCE              → CASIC state machine → fusion.processCasicNav2Sol()
  '$'                             → NMEA buffer (phone GPS sentences only)

FusedLocationProvider (phone GPS):
  LocationRequest: 1s interval, setMinUpdateDistanceMeters(0f),
                   setWaitForAccurateLocation(false)
  hasFix = accuracy < 200m
  → fusion.processNmeaRmc()
```

Trip accumulation (haversine), CSV logging to Downloads/, source preference (phone vs BLE).

---

### `AC6328BleManager.kt` — BLE Protocol

```
sendEscPwm(port, stbd)    → ae03 [0x01, port_lo, port_hi, stbd_lo, stbd_hi]
sendBldc(port, stbd)      → ae03 [0x02, ...]
stopMotors()              → ae03 [0xFF, 0, 0, 0, 0]
setEscMode()              → ae10 write 0x01
setBldcMode()             → ae10 write 0x02
readStatus()              → ae10 read → parses "M<m>A<mv>T<min>"
ae02 NOTIFY               → onAe02Raw callback → GpsManager.feedAe02Bytes()
```

---

### `MainActivity.kt` — BLE Scanner + Dual Device

Single device: **tap** a scan result → `ControlActivity` with one device.

Dual device:
1. **Long press** sensor device → highlighted orange, stored as `secondDevice`
2. **Tap** the motor controller → launches with both as extras
3. `ControlActivity` connects both; ae02 of each feeds same `GpsManager`

---

### `ControlActivity.kt` — Manual Control

**Send-on-change:** one BLE packet per user action, no continuous loop.

```
Hold ▲▼      → ramp: ESC 5 duty/tick (1%), BLDC 100/tick, 250ms interval
⏹ STOP      → activeBle.stopMotors() × 3 (staggered 50ms)
SYNC switch  → PORT mirrors STBD
⚡ D1/D2    → toggle motor commands between primary and secondary BLE device
✈ AUTO       → AutopilotActivity
```

`activeBle` property routes motor commands:
```kotlin
val activeBle get() = if (motorOnDevice2) bleManager2 ?: bleManager else bleManager
```

Dual BLE: both `bleManager` and `bleManager2` independently receive ae02 notifications,
both feeding `gpsManager.feedAe02Bytes()`. Motor commands go only to `activeBle`.
If device 2 disconnects, `motorOnDevice2` resets to false (safety fallback).

---

### `AutopilotActivity.kt` — PI Autopilot

```
5 Hz control loop:
  error = target − actual   (wrapped ±180°)
  if |error| < deadbandDeg: motors equal, reset integral
  integral = clamp(integral + error×dt, ±20)
  diff = clamp((error×Kp + integral×Ki) × headingConf, ±30%)
  port = base + diff,  stbd = base − diff

onGpsUpdate():
  deadbandDeg ← fusion.getState().autoDeadbandDeg   (sea state auto-deadband)
  shows: "2.3 kt  c:87%  sea:12%  🧭cal"
```

Gains (Kp, Ki, deadband) saved to `SharedPreferences("autopilot_prefs")`.

---

### `CalibrationActivity.kt` — Dock Calibration

```
MMC5603 hard-iron:
  rotate boat 360° → collect raw mx,my → find min/max circle
  hard_iron_X = (max_x + min_x) / 2
  hard_iron_Y = (max_y + min_y) / 2
  saved → calibration_prefs, loaded into SensorFusion on app start

QMI8658C gyro bias:
  hold still 5s → average(gx,gy,gz) → zero-rate offsets
  applied in processA1: gz_corrected = gz − gyroBiasZ
```

---

### `MapPickerActivity.kt` — Waypoint

OSMDroid map. **🛰 SAT / 🗺 MAP** toggle uses ESRI World Imagery (free, no API key).
Tap → drop pin → bearing + distance calculated. Passed back to `AutopilotActivity`.
In waypoint mode, bearing recalculates each A3 packet as the boat moves.

---

## Autopilot PI Controller

```
error      = target − actual   (wrapped ±180°)
if |error| < deadbandDeg:  correction = 0,  reset integral
integral   = clamp(integral + error × dt,  ±20)
rawDiff    = (error × Kp  +  integral × Ki) × headingConf
diff       = clamp(rawDiff, ±30%)
port_pct   = base_speed + diff
stbd_pct   = base_speed − diff
```

| Parameter | Default | Stored |
|-----------|---------|--------|
| Kp | 0.8 | `autopilot_prefs` |
| Ki | 0.05 | `autopilot_prefs` |
| Deadband | auto (2°+sea×8°) | `autopilot_prefs` |

---

## Sensor Calibration

**🧭 SENSOR CALIBRATION** button on main screen — dock-side only.

**MMC5603:** Rotate boat slowly 360° in calm water. Progress bar fills 36 sectors (10° each).  
**QMI8658C gyro bias:** Hold still 5 seconds. Zero-rate offsets saved.

Both loaded into `SensorFusion` on every app start via `CalibrationActivity.loadCalibration()`.

GPS auto-calibration of MMC5603 runs automatically at sea (speed>2kt, calm, conf>0.75).

---

## Build

```bash
git clone https://github.com/mikewen/ESC_BLE_App.git
# Open in Android Studio Giraffe+, sync Gradle, run on device
```

```gradle
implementation 'no.nordicsemi.android:ble:2.7.1'
implementation 'no.nordicsemi.android:ble-ktx:2.7.1'
implementation 'no.nordicsemi.android:log:2.3.0'
implementation 'com.google.android.gms:play-services-location:21.2.0'
implementation 'org.osmdroid:osmdroid-android:6.1.18'
```

---

## Safety

- **Send-on-change** — no continuous BLE flood
- `CMD_STOP` sent 3× staggered on STOP button and back press
- Mode (ESC/BLDC) locked at connect — cannot switch while running
- Dual BLE: if secondary motor device disconnects → fallback to primary automatically
- Deadband auto-widens in rough sea to prevent wave-fighting
- Misalignment auto-cal: `|bias| < 15°` guard prevents correcting gross installation errors

---

*AC6328 / AC6329C · JieLi JLAI SDK · LC02H dual-antenna GNSS · MMC5603 · QMI8658C · 5062 BLDC*

---

## Hardware Configuration Flags (`SensorFusion`)

These `var` properties match your specific PCB layout:

```kotlin
// In ControlActivity / AutopilotActivity setupGps():
gpsManager.fusion.gyroZFlipped    = true    // QMI8658C gz axis inverted — turning right increases heading
gpsManager.fusion.accelRotated180 = true    // QMI8658C ax/ay mounted 180° from MMC5603 frame
                                            // (set via gpsManager.accelRotated180)
```

Set `gyroZFlipped = false` if your board has gz in the standard orientation.

### Sea State Sensitivity Tuning

```kotlin
gpsManager.fusion.seaAzCalm   = 200f   // raise → less sensitive to accel heave noise
gpsManager.fusion.seaGxyCalm  = 100f   // raise → less sensitive to gyro vibration
// e.g. on a diesel hull with engine vibration: try seaGxyCalm = 300f
```

---

## Autopilot Screen Layout

```
┌─────────────────────────────────────────────┐
│  [ENGAGE]              [DISENGAGE]  [AUTO DB]│
├─────────────────────────────────────────────┤
│  ○compass  TARGET      SPEED                │
│            045°        3.2 kt               │
│            ACTUAL      MAG HDG              │
│            047° +2°    049°                 │
│            2.3kt c:87% sea:5%               │
├─────────────────────────────────────────────┤
│  COURSE ADJUST: -10  -1°  [045° N]  +1°  +10│
│  [HOLD COURSE]          [📍 TARGET]          │
└─────────────────────────────────────────────┘
```

- **ACTUAL** = fused heading (GNSS+IMU+Mag complementary filter output)
- **MAG HDG** = `fusionState.headingDeg` (mag-driven when no GNSS, GNSS-corrected when A2 arrives)
- **AUTO DB** toggle = sea state adds to deadband when on; manual base preserved either way
- DB display shows `3.0°(+1.4°)` = base 3°, sea adding 1.4°