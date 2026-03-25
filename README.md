# ESC_BLE_App

Android BLE controller for dual **5062 BLDC** motors with GPS autopilot.  
Uses [Nordic Android-BLE-Library](https://github.com/NordicSemiconductor/Android-BLE-Library) v2.7.

**Min SDK:** 23 · **Target SDK:** 34 · **Kotlin**  
Requires a physical Android device with BLE and Location permission.

---

## Firmware Repositories

| Chip | Role | Repository |
|------|------|-----------|
| **AC6328** | ESC / BLDC motor PWM controller | [AC6328_PWM](https://github.com/mikewen/AC6328_PWM) |
| **AC6329C** | Autopilot: PWM + LC02H GNSS + MMC5603 mag + QMI8658C IMU | [AC6329_PWM_GPS](https://github.com/mikewen/AC6329_PWM_GPS) · [AC6329_IIC](https://github.com/mikewen/AC6329_IIC) |

---

## BLE Profile (Service `ae00`)

| UUID | Properties | Role |
|------|-----------|------|
| `ae03` | WRITE_WITHOUT_RESPONSE | Motor command TX (5 bytes) |
| `ae02` | NOTIFY | Echo / AC6329C sensor fusion stream |
| `ae10` | READ \| WRITE | Status read · Mode switch write |

### ae03 — Motor command (5 bytes)

```
[CMD]  [port_lo]  [port_hi]  [stbd_lo]  [stbd_hi]   (little-endian 16-bit)
```

Both ESC and BLDC use the **same 50 Hz PWM timer** — values are duty counts, not µs.

| CMD | Value | Range | Notes |
|-----|-------|-------|-------|
| `CMD_ESC_PWM`   | `0x01` | 500–1000 | 500=1000µs stop, 1000=2000µs full |
| `CMD_BLDC_DUTY` | `0x02` | 0–10000  | 0=stop, 10000=100% duty |
| `CMD_STOP`      | `0xFF` | —        | Immediate stop, both motors |

> `duty × 2 = µs` at 50 Hz (period = 20 ms, 10000 counts)

### ae10 — Mode switch / status

```
WRITE:  0x01 = ESC mode    0x02 = BLDC mode
READ:   "M<mode>A<vbat_mv>T<uptime_min>"   e.g. "M1A3712T5"
```

### ae02 — AC6329C sensor fusion stream

Three binary packet types, all little-endian:

#### `0xA1` — IMU + Magnetometer (50 Hz, 20 bytes)

| Offset | Size | Field |
|--------|------|-------|
| 0 | 1 | `0xA1` |
| 1 | 1 | sequence counter |
| 2–7 | 3×int16 | ax, ay, az (QMI8658C accel LSB) |
| 8–13 | 3×int16 | gx, gy, gz (QMI8658C gyro LSB) |
| 14–19 | 3×int16 | mx, my, mz (MMC5603 mag LSB) |

#### `0xA2` — LC02H PQTMTAR heading (1 Hz, 17 bytes)

| Offset | Size | Field | Scale |
|--------|------|-------|-------|
| 0 | 1 | `0xA2` | |
| 1–4 | uint32 | UTC time ms | |
| 5 | uint8 | quality | 0=none, 4=RTK fixed, 6=DR |
| 6–7 | — | reserved | |
| 8–9 | int16 | pitch | ÷100 → degrees |
| 10–11 | int16 | roll | ÷100 → degrees |
| 12–13 | uint16 | heading | ÷100 → degrees (0–360) |
| 14–15 | uint16 | acc_heading | ÷1000 → degrees accuracy |
| 16 | uint8 | usedSV | satellites used |

#### `0xA3` — LC02H GNRMC position (0.2 Hz, 17 bytes)

| Offset | Size | Field | Scale |
|--------|------|-------|-------|
| 0 | 1 | `0xA3` | |
| 1–4 | uint32 | UTC time ms | |
| 5–8 | int32 | rawLat | NMEA DDMM.MMMM × 10000 |
| 9–12 | int32 | rawLon | NMEA DDDMM.MMMM × 10000 |
| 13–14 | uint16 | speedKnots | ÷100 |
| 15–16 | uint16 | course | ÷100 → COG degrees |

> **A1 is optional.** Without A1 the app uses GNSS-only heading from A2+A3 (no gyro smoothing, no mag, sea state fixed at 0). Autopilot still functions.

---

## Sensor Fusion (`SensorFusion.kt`)

Pure Kotlin — **no Android imports**, portable to any project.

### Data Flow

```
0xA1 (50Hz)  ──► tilt-compensated mag heading  ┐
                 gyro gz integration            ├──► complementary filter ──► filteredHeading
0xA2 (1Hz)   ──► PQTMTAR + RMC blend ──────────┘         ▲
0xA3 (0.2Hz) ──► cache RMC COG + speed ────────────────────┘
                 update lat/lon
```

### Heading Blend Weights (A2)

**PQTMTAR weight** = `qualityFactor × accFactor × satFactor × speedFactor`

| Factor | Formula | Rationale |
|--------|---------|-----------|
| quality | RTK=1.0, DR=0.5, none=0 | RTK geometry is more reliable |
| accuracy | `1 − acc/20°` | Low accuracy → reduce trust |
| satellites | `(sats−4)/4` clamped 0–1 | More sats = better geometry |
| speed | ramps 0.4→1.0 above 0.3 kt | PQTMTAR drifts stationary |

**RMC COG weight** = speed ramp: 0 at 0.5 kt → 1.0 at 2 kt  
(COG unreliable below 0.5 kt; PQTMTAR dual-antenna is more reliable there)

### Sea State

Rolling 100-sample high-pass filtered variance of az (vertical accel).  
`seaState = clamp((stddev − 200) / 1800, 0, 1)` · 0=calm, 1=rough

### Automatic Deadband

```
deadband = 2° + seaState × 8°
```
Calm=2°, rough=10°. Updated live each GPS tick. Prevents autopilot fighting waves.

### GPS Auto-Calibration of MMC5603

Conditions: `speed > 2kt` AND `seaState < 0.15` AND `gpsConf > 0.75`  
Accumulates `bias = GPS_heading − mag_heading` over 30 samples.  
When `stddev(bias) < 5°` → calibrated. Bias applied; mag filter weight increases 0.02→0.05.

---

## Autopilot PI Controller

```
error      = target − actual   (wrapped ±180°)
if |error| < deadbandDeg:  correction = 0,  reset integral
integral   = clamp(integral + error × dt,  ±20)
rawDiff    = (error × Kp  +  integral × Ki) × headingConfidence
diff       = clamp(rawDiff, ±30%)
port_pct   = base_speed + diff
stbd_pct   = base_speed − diff
```

Default gains (live-adjustable in tuning card, saved to SharedPreferences):

| | Default | Range |
|--|---------|-------|
| Kp | 0.8 | 0.1–5.0 |
| Ki | 0.05 | 0.0–1.0 |
| Deadband | auto (sea state) | 0–15° |

---

## Sensor Calibration

Accessible from the main screen via **🧭 SENSOR CALIBRATION**.

### MMC5603 Hard-Iron Cal

Rotate the boat slowly 360° in calm water at the dock.  
Progress bar fills as 10° sectors are covered (36 total). Auto-finishes at 100%.  
Offsets saved to `SharedPreferences("calibration_prefs")`, loaded on every app start.

### QMI8658C Gyro Bias Cal

Place sensor on a level surface. Hold still for 5 seconds.  
Gyro zero-rate offsets saved automatically.

---

## App Screens

### MainActivity — BLE Scanner
Scans for `ESC_PWM` / `BLDC_PWM`. **🧭 SENSOR CALIBRATION** button at bottom.

### ControlActivity — Manual Control
- **Send-on-change** — one packet per button press, no continuous loop
- Hold ▲▼ to ramp: ESC step=5 duty (1%), BLDC step=100, 250 ms tick
- **⏹** stop button — CMD_STOP sent 3×
- **SYNC** — PORT mirrors STBD
- Mode bar live PWM: `P:700(1400µs) S:700(1400µs)`
- GPS card: speed, heading, compass, trip distance, max speed, source toggle, CSV log
- **✈ AUTO** → AutopilotActivity

### AutopilotActivity — Hold Course
- ENGAGE / DISENGAGE (5 Hz PI loop)
- **HOLD COURSE** — lock current GPS heading
- **📍 TARGET** — map waypoint picker (bearing recalculates as boat moves)
- Course adjust: −10  −1°  [display]  +1°  +10
- Speed ▲▼ + slider
- Live motor output: PORT% · STBD% · error°
- Tuning card: Kp / Ki / DB (auto-updated from sea state)
- Status line: `2.3 kt  c:87%  sea:12%  🧭cal`

### MapPickerActivity — Waypoint
- OSMDroid map, **🛰 SAT / 🗺 MAP** tile toggle (ESRI satellite, no API key)
- Tap to drop pin, bearing + distance shown

### CalibrationActivity — Sensor Setup
- MMC5603 360° hard-iron calibration (sector progress)
- QMI8658C gyro bias (5 s countdown)
- Clear buttons to redo

---

## Hardware

```
AC6328 / AC6329C
  IO_PORT_DM ──► ESC/BLDC driver ──► PORT motor (left)     JL_TIMER3
  IO_PORT_DP ──► ESC/BLDC driver ──► STBD motor (right)    JL_TIMER2

AC6329C additional:
  UART ──► LC02H dual-antenna GNSS
             $PQTMTAR  heading / pitch / roll
             $GNRMC    position / speed / COG
  I2C  ──► MMC5603 magnetometer
  SPI  ──► QMI8658C 6-DoF IMU (accel + gyro)
```

### PWM Duty Scale

```
ESC:   duty 500–1000  →  1000–2000 µs  (1 duty = 2 µs)
BLDC:  duty 0–10000   →  0–100%        (1 duty = 0.01%)
```

---

## Project Structure

```
java/com/escbleapp/
  SensorFusion.kt         ★ Portable pure-Kotlin fusion engine (no Android imports)
  GpsManager.kt           Android wrapper: phone GPS + ae02 BLE routing → SensorFusion
  AC6328BleManager.kt     BLE protocol: command packets, ae02/ae10 parsing
  MainActivity.kt         BLE scanner + calibration entry
  ControlActivity.kt      Manual throttle + GPS card
  AutopilotActivity.kt    PI autopilot + sea state + sensor status
  CalibrationActivity.kt  MMC5603 + QMI8658C dock calibration
  MapPickerActivity.kt    OSMDroid waypoint picker
  CompassView.kt          Custom compass rose widget
```

### SensorFusion.kt Public API

```kotlin
val fusion = SensorFusion()
fusion.onFusedHeading = { state -> /* state.headingDeg, seaState, autoDeadbandDeg, … */ }

// Feed packets:
fusion.processA1(ax, ay, az, gx, gy, gz, mx, my, mz, nowMs)
fusion.processA2(tarHdg, pitch, roll, accDeg, quality, sats, nowMs)
fusion.processA3(lat, lon, speedKt, courseDeg, hasFix)
fusion.processCasicNav2Sol(x, y, z, vx, vy, vz, sAcc, fixOk, fixType)
fusion.processNmeaRmc(speedKt, cogDeg, hasFix, lat, lon)

// Manual calibration:
fusion.startManualMagCal()
fusion.feedManualMagSample(mx, my)
fusion.finishManualMagCal()   // returns true if enough coverage
fusion.startGyroBiasCal()
fusion.feedGyroBiasSample(gx, gy, gz)
fusion.finishGyroBiasCal()
```

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

- **Send-on-change** — no continuous BLE flood; ESC firmware watchdog not required
- `CMD_STOP` sent 3× on STOP button and back press
- Mode locked at connect — cannot switch while running
- Autopilot DISENGAGE returns both motors to base speed immediately
- Deadband auto-widens in rough sea to prevent wave-fighting

---

*AC6328 / AC6329C · JieLi JLAI SDK · LC02H dual-antenna GNSS · MMC5603 · QMI8658C · 5062 BLDC*