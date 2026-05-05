package com.escbleapp

import android.bluetooth.BluetoothDevice
import android.bluetooth.BluetoothGatt
import android.bluetooth.BluetoothGattCharacteristic
import android.content.Context
import android.os.Handler
import android.os.Looper
import android.util.Log
import no.nordicsemi.android.ble.BleManager
import no.nordicsemi.android.ble.observer.ConnectionObserver
import java.util.UUID

/**
 * LOOKBON BLE Remote — AC6329C device on ae30/ae02.
 *
 * ── Protocol ─────────────────────────────────────────────────────────────────
 * Same ae30 service / ae02 characteristic as the motor controller.
 * Key packets are 1–2 raw bytes; Python keymap uses notify_data.hex().upper():
 *   Single byte 0xA1 → hex string "A1" → @ click
 *   Two bytes 0x44 0x33 → hex string "D3" → joystick left
 *
 * SENSOR DATA (0xA1/0xA2/0xA3 sensor packets, ≥17 bytes) are silently ignored —
 * they share the same characteristic but are routed to SensorFusion by GpsManager.
 *
 * ── Key map (notify_data.hex().upper()) ──────────────────────────────────────
 *   A1=@ click    B1=@ long-press    C1=@ long-press-release
 *   A2=A click    B2=A long-press    C2=A long-press-release   (thumb RIGHT)
 *   A3=B click    B3=B long-press    C3=B long-press-release   (thumb LEFT)
 *   A4=C click    B4=C long-press    C4=C long-press-release   (thumb UP)
 *   A5=D click    B5=D long-press    C5=D long-press-release   (thumb DOWN)
 *   A6=R click    B6=R long-press    C6=R long-press-release   (NEAR trigger)
 *   A7=L click    B7=L long-press    C7=L long-press-release   (FAR trigger)
 *   D0=joy none   D1=up  D2=down  D3=left  D4=right
 *   D5=up-left    D6=down-left  D7=up-right  D8=down-right
 *
 * ── Physical layout ──────────────────────────────────────────────────────────
 *   @ = top center button
 *   B = thumb LEFT,  A = thumb RIGHT,  C = thumb UP,  D = thumb DOWN
 *   R = NEAR trigger (index finger, closer to palm)
 *   L = FAR  trigger (index finger, further from palm)
 *
 * ── Command mapping ──────────────────────────────────────────────────────────
 *  AUTOPILOT MODE:
 *   @ click          → ENGAGE / DISENGAGE toggle
 *   @ long-press     → STOP
 *   L + R + @ click  → Emergency STOP
 *   B click          → Course −1°
 *   A click          → Course +1°
 *   R + B            → Course −10°
 *   R + A            → Course +10°
 *   C click          → Speed +1%
 *   D click          → Speed −1%
 *   R + C            → Speed +5%
 *   R + D            → Speed −5%
 *   L hold           → Manual Override mode
 *
 *  MANUAL OVERRIDE (while L held):
 *   B / A click      → differential turn port / stbd (±5%)
 *   C / D click      → both motors ±5%
 *   Joystick ←→     → differential ±1% (auto-repeat)
 *   Joystick ↑↓     → both ±1% (auto-repeat)
 *   L release        → return to AP mode
 */
class LookbonRemote(context: Context) : BleManager(context) {

    companion object {
        private const val TAG = "LookbonRemote"

        // ae30 service / ae02 characteristic — same as motor controller
        val SERVICE_UUID = UUID.fromString("0000ae30-0000-1000-8000-00805f9b34fb")
        val CHAR_UUID    = UUID.fromString("0000ae02-0000-1000-8000-00805f9b34fb")

        val REMOTE_NAME_FILTERS = listOf("LOOKBON", "lookbon")

        const val JOY_REPEAT_MS = 200L

        // Sensor packet headers — packets starting with these bytes AND ≥17 bytes
        // are AC6329C sensor data, not key events. Skip them.
        private val SENSOR_HEADERS = setOf(0xA1.toByte(), 0xA2.toByte(), 0xA3.toByte())
    }

    // ── Callbacks ─────────────────────────────────────────────────────────────
    var onRemoteCommand:  ((RemoteBleManager.RemoteCommand) -> Unit)? = null
    var onConnected:      (() -> Unit)?          = null
    var onDisconnected:   (() -> Unit)?          = null
    var onManualOverride: ((Boolean) -> Unit)?   = null

    // ── State ─────────────────────────────────────────────────────────────────
    private var rHeld     = false
    private var lHeld     = false
    private var apEngaged = false
    private val mainHandler = Handler(Looper.getMainLooper())

    private var joyRepeatRunnable: Runnable? = null

    // ── GATT ─────────────────────────────────────────────────────────────────
    private var notifyChar: BluetoothGattCharacteristic? = null

    override fun getGattCallback(): BleManagerGattCallback = GattCb()

    private inner class GattCb : BleManagerGattCallback() {
        override fun isRequiredServiceSupported(gatt: BluetoothGatt): Boolean {
            val svc = gatt.getService(SERVICE_UUID) ?: run {
                Log.w(TAG, "ae30 service not found — available: ${gatt.services.map { it.uuid }}")
                return false
            }
            notifyChar = svc.getCharacteristic(CHAR_UUID) ?: run {
                Log.w(TAG, "ae02 char not found in ae30 service")
                return false
            }
            Log.i(TAG, "Found ae30/ae02 for LOOKBON")
            return true
        }

        override fun initialize() {
            notifyChar?.let { c ->
                setNotificationCallback(c).with { _, data ->
                    val raw = data.value ?: return@with
                    // Skip sensor data packets (≥17 bytes starting with A1/A2/A3)
                    if (raw.size >= 17 && raw[0] in SENSOR_HEADERS) return@with
                    // Convert raw bytes to hex string — matches Python's notify_data.hex().upper()
                    val hex = raw.joinToString("") { "%02X".format(it) }
                    Log.d(TAG, "Key packet: $hex  (${raw.size} bytes)")
                    handleHex(hex)
                }
                enableNotifications(c).enqueue()
                Log.i(TAG, "LOOKBON notifications enabled on ae02")
            }
        }

        override fun onServicesInvalidated() { notifyChar = null }
    }

    // ── Packet parsing ────────────────────────────────────────────────────────

    private fun handleHex(hex: String) {
        when {
            // 2-char = single byte key event or joystick
            hex.length == 2 -> {
                val eventByte = hex[0]   // A/B/C or D
                val btnChar   = hex[1]   // 1-7 or 0-8
                when (eventByte) {
                    'D' -> handleJoystick(btnChar.digitToIntOrNull() ?: return)
                    'A', 'B', 'C' -> handleButton(eventByte,
                        btnChar.digitToIntOrNull() ?: return)
                }
            }
            // 4-char = 2-byte joystick (e.g. "D3" stored as 0x44 0x33)
            hex.length == 4 -> {
                val s = String(byteArrayOf(
                    hex.substring(0,2).toInt(16).toByte(),
                    hex.substring(2,4).toInt(16).toByte()
                ))
                if (s[0] == 'D') handleJoystick(s[1].digitToIntOrNull() ?: return)
            }
        }
    }

    private fun handleButton(event: Char, btn: Int) {
        when {
            // ── R modifier ───────────────────────────────────────────────────
            event == 'B' && btn == 6 -> { rHeld = true;  return }
            event == 'C' && btn == 6 -> { rHeld = false; return }

            // ── L modifier — Manual Override ─────────────────────────────────
            event == 'B' && btn == 7 -> {
                lHeld = true
                mainHandler.post { onManualOverride?.invoke(true) }
                return
            }
            event == 'C' && btn == 7 -> {
                lHeld = false
                stopJoyRepeat()
                mainHandler.post { onManualOverride?.invoke(false) }
                return
            }

            // ── @ button ─────────────────────────────────────────────────────
            event == 'A' && btn == 1 -> when {
                lHeld && rHeld -> emit(stopCmd())
                else           -> emit(toggleApCmd())
            }
            event == 'B' && btn == 1 -> emit(stopCmd())   // long-press = STOP

            // ── Thumb buttons ─────────────────────────────────────────────────
            event == 'A' && btn == 3 -> {  // B = thumb left
                if (lHeld) emit(manualTurnCmd(-5))
                else       emit(courseCmd(if (rHeld) -10f else -1f))
            }
            event == 'A' && btn == 2 -> {  // A = thumb right
                if (lHeld) emit(manualTurnCmd(+5))
                else       emit(courseCmd(if (rHeld) +10f else +1f))
            }
            event == 'A' && btn == 4 -> {  // C = thumb up
                if (lHeld) emit(speedBothCmd(+5))
                else       emit(speedBothCmd(if (rHeld) +5 else +1))
            }
            event == 'A' && btn == 5 -> {  // D = thumb down
                if (lHeld) emit(speedBothCmd(-5))
                else       emit(speedBothCmd(if (rHeld) -5 else -1))
            }
        }
    }

    // ── Joystick ──────────────────────────────────────────────────────────────

    private fun handleJoystick(dir: Int) {
        stopJoyRepeat()
        if (dir == 0) return

        val cmd = when (dir) {
            1 -> if (lHeld) speedBothCmd(+1)  else null          // up
            2 -> if (lHeld) speedBothCmd(-1)  else null          // down
            3 -> if (lHeld) manualTurnCmd(-1) else courseCmd(-1f)// left
            4 -> if (lHeld) manualTurnCmd(+1) else courseCmd(+1f)// right
            5 -> if (lHeld) manualTurnCmd(-1) else null          // up-left
            6 -> if (lHeld) manualTurnCmd(-1) else null          // down-left
            7 -> if (lHeld) manualTurnCmd(+1) else null          // up-right
            8 -> if (lHeld) manualTurnCmd(+1) else null          // down-right
            else -> null
        } ?: return

        emit(cmd)
        joyRepeatRunnable = object : Runnable {
            override fun run() { emit(cmd); mainHandler.postDelayed(this, JOY_REPEAT_MS) }
        }
        mainHandler.postDelayed(joyRepeatRunnable!!, JOY_REPEAT_MS)
    }

    private fun stopJoyRepeat() {
        joyRepeatRunnable?.let { mainHandler.removeCallbacks(it) }
        joyRepeatRunnable = null
    }

    // ── Command factories ─────────────────────────────────────────────────────

    private fun toggleApCmd() = if (apEngaged) {
        apEngaged = false
        RemoteBleManager.RemoteCommand(RemoteBleManager.GRP_AUTOPILOT, RemoteBleManager.AP_DISENGAGE)
    } else {
        apEngaged = true
        RemoteBleManager.RemoteCommand(RemoteBleManager.GRP_AUTOPILOT, RemoteBleManager.AP_ENGAGE)
    }

    private fun stopCmd() = RemoteBleManager.RemoteCommand(
        RemoteBleManager.GRP_SPEED, RemoteBleManager.SPD_STOP)

    private fun courseCmd(delta: Float) = RemoteBleManager.RemoteCommand(
        RemoteBleManager.GRP_COURSE, when (delta) {
            -10f -> RemoteBleManager.CRS_LEFT_10
            +10f -> RemoteBleManager.CRS_RIGHT_10
            -1f  -> RemoteBleManager.CRS_LEFT_1
            else -> RemoteBleManager.CRS_RIGHT_1
        })

    private fun speedBothCmd(step: Int) = RemoteBleManager.RemoteCommand(
        RemoteBleManager.GRP_SPEED, when {
            step >= 5  -> RemoteBleManager.SPD_UP
            step <= -5 -> RemoteBleManager.SPD_DOWN
            step > 0   -> RemoteBleManager.SPD_UP_1
            else       -> RemoteBleManager.SPD_DOWN_1
        })

    private fun manualTurnCmd(step: Int) = RemoteBleManager.RemoteCommand(
        RemoteBleManager.GRP_COURSE, if (step > 0) RemoteBleManager.CRS_RIGHT_1
                                     else           RemoteBleManager.CRS_LEFT_1)

    private fun emit(cmd: RemoteBleManager.RemoteCommand) {
        mainHandler.post { onRemoteCommand?.invoke(cmd) }
    }

    fun setApEngaged(engaged: Boolean) { apEngaged = engaged }

    // ── Connect ───────────────────────────────────────────────────────────────

    fun connectToDevice(device: BluetoothDevice) {
        setConnectionObserver(object : ConnectionObserver {
            override fun onDeviceConnecting(d: BluetoothDevice)    = Unit
            override fun onDeviceDisconnecting(d: BluetoothDevice) = Unit
            override fun onDeviceReady(d: BluetoothDevice)         = Unit
            override fun onDeviceConnected(d: BluetoothDevice) {
                Log.i(TAG, "LOOKBON connected"); mainHandler.post { onConnected?.invoke() }
            }
            override fun onDeviceFailedToConnect(d: BluetoothDevice, reason: Int) {
                Log.w(TAG, "LOOKBON failed: $reason"); mainHandler.post { onDisconnected?.invoke() }
            }
            override fun onDeviceDisconnected(d: BluetoothDevice, reason: Int) {
                Log.i(TAG, "LOOKBON disconnected"); mainHandler.post { onDisconnected?.invoke() }
            }
        })
        connect(device).useAutoConnect(false).enqueue()
    }

    override fun close() { stopJoyRepeat(); super.close() }
}
