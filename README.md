# ESC_BLE_App

Android BLE controller for dual **5062 BLDC** motors — manual control and autopilot (hold course).  
Uses the [Nordic Android-BLE-Library](https://github.com/NordicSemiconductor/Android-BLE-Library) v2.7.

---

## Firmware Repositories

| Chip | Role | Repository |
|------|------|-----------|
| **AC6328** | ESC / BLDC motor PWM controller | [github.com/mikewen/AC6328_PWM](https://github.com/mikewen/AC6328_PWM) |
| **AC6329C** | GPS/GNSS + 9DoF autopilot controller | [github.com/mikewen/AC6329_PWM_GPS](https://github.com/mikewen/AC6329_PWM_GPS) |

### AC6328 — Simple motor controller
- 2 PWM outputs: `IO_PORT_DM` (port) · `IO_PORT_DP` (starboard)
- Timers: `JL_TIMER3` (port) · `JL_TIMER2` (starboard)
- ESC mode: 50 Hz, 1000–2000 µs
- BLDC mode: 1 kHz, 0–100% duty
- Advertises as `ESC_PWM` or `BLDC_PWM`
- App auto-detects mode from device name

### AC6329C — Autopilot controller (more GPIO)
- Same dual PWM outputs as AC6328
- Additional pins for GPS/GNSS UART or 9DoF IMU (SPI/I2C)
- Streams NMEA sentences or heading data via `ae02 NOTIFY`
- Enables hold-course autopilot with differential thrust
- Advertises as `ESC_PWM` or `BLDC_PWM` (same app, autopilot tab unlocks GPS data)

---

## BLE Profile (Service `ae00`)

| UUID | Handle | Properties | Role |
|------|--------|-----------|------|
| `ae03` | `0x000b` | WRITE_WITHOUT_RESPONSE | **Motor command TX** (5 bytes) |
| `ae02` | `0x0008` | NOTIFY | **Echo / NMEA GPS stream RX** |
| `ae10` | `0x0013` | READ \| WRITE | Status READ · Mode switch WRITE |

### ae03 Command Packet (5 bytes)

```
[CMD]  [port_lo]  [port_hi]  [stbd_lo]  [stbd_hi]
```

| CMD | Value | port/stbd range | Use |
|-----|-------|----------------|-----|
| `CMD_ESC_PWM`   | `0x01` | 1000–2000 µs | ESC pulse width |
| `CMD_BLDC_DUTY` | `0x02` | 0–10000 (0–100%) | BLDC duty cycle |
| `CMD_STOP`      | `0xFF` | ignored | Stop both motors |

### ae10 WRITE — Runtime mode switch
```
0x01 → ESC mode    0x02 → BLDC mode
```

### ae10 READ — Device status (ASCII)
```
"M<mode>A<vbat_mv>T<uptime_min>"    e.g. "M1A3712T5"
 M1 = ESC   M2 = BLDC
```

### ae02 NOTIFY — Two roles
1. **Motor echo** — 5-byte mirror of last ae03 command (confirmed PWM)
2. **NMEA GPS stream** (AC6329C only) — `$GPRMC`/`$GPGGA` sentences for autopilot

---

## App Features

### Manual Control Screen
- Hold **▲▼** buttons to ramp throttle (80ms tick, 20Hz BLE send)
- **BOTH ▲▼** centre buttons — move both motors simultaneously
- **Sync** toggle — PORT and STBD mirror each other symmetrically
- Fine sliders per motor
- ESC / BLDC auto-detected from device name at connect — **locked for session** (no mid-flight switching)
- **ARM** (ESC only) — sends 1000µs for arm sequence
- **MAX** — full throttle with confirm dialog

### GPS Card
- **Phone GPS** (FusedLocationProvider) and **BLE GPS** (NMEA via ae02) — auto-switches
- Speed in **knots / km/h** (tap to toggle)
- Compass rose with heading needle
- **Trip distance** (nm or km) with Haversine accumulation
- **Max speed** recorded this session
- RESET button for trip stats

### Autopilot Screen (AC6329C)
- **ENGAGE / DISENGAGE** — starts 5Hz proportional heading controller
- **SET CURRENT HEADING** — locks current GPS bearing as target
- **Course adjust:** −10° −1° +1° +10° buttons
- **Speed ▲▼** hold buttons + slider (0–100% both motors)
- Live motor output: PORT % · STBD % · heading error · differential
- Proportional controller: `differential = error × Kp` → port faster when turning right

### Autopilot Algorithm
```
heading_error  = target_heading - actual_heading   (wrapped ±180°)
differential   = error × Kp   (Kp=0.8, clamped ±30%)
port_pct       = base_speed + differential
stbd_pct       = base_speed - differential
```
Tune `Kp` in `AutopilotActivity.kt` for your vessel's response.

---

## Build

```bash
git clone https://github.com/mikewen/ESC_BLE_App.git
cd ESC_BLE_App
# Open in Android Studio Giraffe+, sync Gradle, run on physical device
```

**Min SDK:** 23 · **Target SDK:** 34 · **Language:** Kotlin  
**Requires:** Physical Android device with BLE · Location permission for GPS

### Dependencies
```gradle
implementation 'no.nordicsemi.android:ble:2.7.1'
implementation 'no.nordicsemi.android:ble-ktx:2.7.1'
implementation 'no.nordicsemi.android:log:2.3.0'
implementation 'com.google.android.gms:play-services-location:21.2.0'
```

---

## Project Structure

```
app/src/main/java/com/escbleapp/
  AC6328BleManager.kt   BLE protocol — command packets, NOTIFY parsing
  MainActivity.kt       BLE scanner — auto-filters ESC_PWM / BLDC_PWM
  ControlActivity.kt    Manual throttle control + GPS card
  AutopilotActivity.kt  Hold-course autopilot with differential thrust
  GpsManager.kt         Dual-source GPS — phone FLP + BLE NMEA parser
  CompassView.kt        Custom compass rose widget
  JoystickView.kt       (legacy — retained, unused in current UI)
```

---

## Hardware Wiring

```
AC6328 / AC6329C
  IO_PORT_DM ──► ESC / BLDC driver ──► PORT motor (left)
  IO_PORT_DP ──► ESC / BLDC driver ──► STBD motor (right)

AC6329C additional (autopilot):
  UART_RX/TX ──► GPS/GNSS module   (NMEA → ae02 NOTIFY)
       or
  SPI/I2C    ──► 9DoF IMU          (heading → ae02 NOTIFY)
```

---

## Safety

- Motors stop automatically if no BLE packet received within **2 seconds** (firmware watchdog)
- Mode (ESC/BLDC) locked at connect — cannot switch while running
- Autopilot DISENGAGE sends immediate stop to both motors
- MAX throttle button requires tap-to-confirm dialog
- Back button from autopilot disengages before returning to manual control

---

*AC6328 / AC6329C · Jieli JLAI SDK · 5062 BLDC dual-motor · ESC_BLE_App*