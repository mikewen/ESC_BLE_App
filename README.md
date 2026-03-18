# ESC_BLE_App

Android BLE controller for **AC6328** SoC driving dual **5062 BLDC** motors via ESC.  
Uses the [Nordic Android-BLE-Library](https://github.com/NordicSemiconductor/Android-BLE-Library) v2.7.

---

## Hardware

| Component | Detail |
|-----------|--------|
| BLE MCU | AC6328 (Jieli JLAI SDK) |
| Motors | 5062 BLDC (port + starboard) |
| ESC | Standard RC ESC — 50 Hz, 1000–2000 µs |
| PWM pins | `IO_PORT_DM` (port) · `IO_PORT_DP` (starboard) |
| Timers | `JL_TIMER3` (port) · `JL_TIMER2` (starboard) |

## BLE Profile  (Service `ae30`)

| UUID | Handle | Properties | Role |
|------|--------|-----------|------|
| `ae03` | `0x000b` | WRITE_WITHOUT_RESPONSE | **Primary PWM TX** |
| `ae04` | `0x000d` | NOTIFY | **PWM echo / confirmation RX** |
| `ae10` | `0x0013` | READ / WRITE | Battery + status READ, fallback TX |
| `ae02` | `0x0008` | NOTIFY | UART relay / debug |
| `ae05` | `0x0010` | INDICATE | Extended status |

### Why ae03 for PWM?

`WRITE_WITHOUT_RESPONSE` has zero ATT acknowledgment overhead — correct for real-time 20 Hz PWM streaming. `ae10` WRITE requires an ATT acknowledgment round-trip per packet which adds latency.

---

## TX Packet Format

```
Bytes: [port_lo] [port_hi] [starboard_lo] [starboard_hi]
Type:   uint16 little-endian     uint16 little-endian
Range:  1000 - 2000 us each
```

Example: both motors at 1750 us (75% throttle) = `E6 06 E6 06`

---

## PWM Feedback — Three Channels

### 1. ae10 READ  (polled every 2 s by app)
Firmware ASCII response format: `"A<vbat_mv>B<vbat_mv/10>T<uptime_min>"`
Example: `"A3712B371T5"` = 3712 mV battery, 5 minutes uptime.
The app parses this and shows a live battery bar.

### 2. ae04 NOTIFY  (pushed after each ae03 write)
4-byte binary echo: `[port_lo, port_hi, starboard_lo, starboard_hi]`
App compares to last-sent values and shows PWM drift warning if delta > 10 us.

### 3. ae02 NOTIFY  (UART relay)
Raw bytes from device UART, shown as hex in the telemetry panel.

---

## App Features

- Dual independent vertical joysticks (multi-touch, one finger per stick)
- Fine-control SeekBars per motor
- Sync mode: mirrors port to starboard
- ARM button: sends 1000 us (ESC arm sequence)
- STOP button: sends 1500 us neutral to both motors immediately
- MAX button: 2000 us with confirm dialog
- Feedback panel: battery bar, confirmed PWM echo, TX hex, UART stream
- 20 Hz PWM send loop while joystick is active
- Firmware watchdog: device auto-stops 2 s after last packet

---

## Build

```bash
git clone https://github.com/mikewen/ESC_BLE_App.git
cd ESC_BLE_App
# Open in Android Studio Giraffe+, sync Gradle, run on physical device
```

Min SDK 23 (Android 6.0) · Target SDK 34 · Kotlin · Requires physical BLE device

### Dependencies (auto-downloaded by Gradle)

```gradle
implementation 'no.nordicsemi.android:ble:2.7.1'
implementation 'no.nordicsemi.android:ble-ktx:2.7.1'
implementation 'no.nordicsemi.android:log:2.3.0'
```

---

## Project Structure

```
app/src/main/java/com/escbleapp/
  AC6328BleManager.kt   BleManager subclass - BLE + feedback logic
  JoystickView.kt       Custom joystick widget
  MainActivity.kt       BLE scanner
  ControlActivity.kt    Motor control + feedback UI
```

---

*AC6328 / Jieli JLAI SDK · 5062 BLDC dual-motor ESC*
