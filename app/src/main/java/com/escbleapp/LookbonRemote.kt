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

/**
 * LOOKBON BLE Remote — joystick + 7 buttons.
 *
 * ── Physical layout ──────────────────────────────────────────────────────────
 *   Thumb buttons (face):
 *     @  = top/center button   (index: 1)
 *     B  = thumb LEFT          (index: 3)
 *     A  = thumb RIGHT         (index: 2)
 *     C  = thumb UP            (index: 4)
 *     D  = thumb DOWN          (index: 5)
 *   Trigger buttons (under joystick):
 *     R  = NEAR trigger (closer to palm, index finger) (index: 6)
 *     L  = FAR  trigger (further from palm)            (index: 7)
 *   Joystick: D0=none D1=up D2=down D3=left D4=right
 *             D5=up-left D6=down-left D7=up-right D8=down-right
 *
 * ── Raw packet format (2-byte hex string via GATT NOTIFY) ────────────────────
 *   Byte 0 = event type:  A=click  B=long-press  C=long-press-release
 *   Byte 1 = button index: 1–7 for buttons, or 'D' prefix for joystick
 *   E.g.  "A2" = A-button click,  "B6" = R-button long-press,  "D3" = joystick left
 *
 * ── Modifier state ───────────────────────────────────────────────────────────
 *   R held (B6 → C6): Coarse Adjust modifier — ×10 steps
 *   L held (B7 → C7): Manual Override mode
 *   L+R+@ held then @-click: Emergency STOP
 *
 * ── Mapping → RemoteCommand ──────────────────────────────────────────────────
 *
 *  AUTOPILOT MODE (default):
 *   @  click                          → AP ENGAGE / DISENGAGE (toggle)
 *   @  long-press 1.5s (B1 fired)     → STOP
 *   L + R + @ click                   → STOP (emergency)
 *   B  click                          → Course −1°
 *   A  click                          → Course +1°
 *   R + B click                       → Course −10°
 *   R + A click                       → Course +10°
 *   C  click                          → Speed +1%
 *   D  click                          → Speed −1%
 *   R + C click                       → Speed +5%
 *   R + D click                       → Speed −5%
 *   L  hold start (B7)                → Manual Override active
 *   L  release (C7)                   → Return to AP
 *
 *  MANUAL OVERRIDE MODE (while L held):
 *   B  click  → PORT −5  STBD +5   (turn port)
 *   A  click  → PORT +5  STBD −5   (turn stbd)
 *   C  click  → both +5
 *   D  click  → both −5
 *   Joystick left/right → differential ±1 per tick (auto-repeat while held)
 *   Joystick up/down    → both ±1 per tick (auto-repeat while held)
 *   L release → return AP mode
 *
 *  MANUAL OVERRIDE — emitted as RemoteCommand pairs (PORT, then STBD).
 *  Consumer calls handleRemoteCommand() twice — once for each.
 */
class LookbonRemote(context: Context) : BleManager(context) {

    companion object {
        private const val TAG = "LookbonRemote"

        val REMOTE_NAME_FILTERS = listOf("LOOKBON", "lookbon")

        /** Joystick auto-repeat interval ms */
        const val JOY_REPEAT_MS = 200L
    }

    // ── Callbacks ─────────────────────────────────────────────────────────────
    /** Single command callback — same type as existing remote for drop-in compatibility. */
    var onRemoteCommand: ((RemoteBleManager.RemoteCommand) -> Unit)? = null
    var onConnected:     (() -> Unit)?                               = null
    var onDisconnected:  (() -> Unit)?                               = null
    /** Called when manual override starts (L held) or ends (L released). */
    var onManualOverride: ((Boolean) -> Unit)?                       = null

    // ── State ─────────────────────────────────────────────────────────────────
    private var rHeld = false        // R (near trigger) held
    private var lHeld = false        // L (far trigger) held — manual override
    private var apEngaged = false    // track AP state so @ can toggle

    private val mainHandler = Handler(Looper.getMainLooper())

    // Joystick auto-repeat
    private var joyRepeatRunnable: Runnable? = null
    private var lastJoyDir = 0   // 0=none, 1=up, 2=down, 3=left, 4=right

    // ── GATT ─────────────────────────────────────────────────────────────────
    // We don't know the LOOKBON service/characteristic UUIDs, so we scan
    // every service and subscribe to every characteristic that supports NOTIFY.
    // All notifications are routed to handleRawPacket() regardless of UUID.

    private val notifyChars = mutableListOf<BluetoothGattCharacteristic>()

    override fun getGattCallback(): BleManagerGattCallback = LookbonGattCallback()

    private inner class LookbonGattCallback : BleManagerGattCallback() {

        override fun isRequiredServiceSupported(gatt: BluetoothGatt): Boolean {
            // Accept any device — collect all notify-capable characteristics
            notifyChars.clear()
            for (svc in gatt.services) {
                for (char in svc.characteristics) {
                    val props = char.properties
                    if (props and BluetoothGattCharacteristic.PROPERTY_NOTIFY != 0 ||
                        props and BluetoothGattCharacteristic.PROPERTY_INDICATE != 0) {
                        notifyChars.add(char)
                        Log.d(TAG, "Found notify char: ${char.uuid} in svc: ${svc.uuid}")
                    }
                }
            }
            // Connect as long as there is at least one notify characteristic
            return notifyChars.isNotEmpty()
        }

        override fun initialize() {
            // Subscribe to every notify characteristic found
            for (c in notifyChars) {
                setNotificationCallback(c).with { _, data ->
                    val raw = data.value ?: return@with
                    val str = raw.decodeToString().trim().uppercase()
                    Log.d(TAG, "Notify [${c.uuid}]: $str  hex=${raw.joinToString("") { "%02X".format(it) }}")
                    handleRawPacket(str)
                }
                enableNotifications(c).enqueue()
            }
            Log.i(TAG, "Subscribed to ${notifyChars.size} notify characteristics")
        }

        override fun onServicesInvalidated() { notifyChars.clear() }
    }

    // ── Packet parsing ────────────────────────────────────────────────────────

    private fun handleRawPacket(raw: String) {
        // Joystick: D0–D8
        if (raw.startsWith("D") && raw.length >= 2) {
            val dir = raw[1].digitToIntOrNull() ?: return
            handleJoystick(dir)
            return
        }
        // Button: [A|B|C][1-7]
        if (raw.length >= 2) {
            val event  = raw[0]   // A=click  B=long-press  C=long-press-release
            val btnNum = raw[1].digitToIntOrNull() ?: return
            handleButton(event, btnNum)
        }
    }

    private fun handleButton(event: Char, btn: Int) {
        when {
            // ── R modifier (near trigger) ─────────────────────────────────────
            event == 'B' && btn == 6 -> { rHeld = true;  return }
            event == 'C' && btn == 6 -> { rHeld = false; return }

            // ── L modifier (far trigger) — Manual Override ────────────────────
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
            event == 'A' && btn == 1 -> {
                when {
                    lHeld && rHeld -> emit(stopCmd())           // L+R+@ = Emergency STOP
                    else           -> emit(toggleApCmd())       // @ click = toggle AP
                }
            }
            event == 'B' && btn == 1 -> emit(stopCmd())        // @ long-press = STOP

            // ── Thumb buttons (mode-dependent) ───────────────────────────────
            event == 'A' && btn == 3 -> {   // B = thumb left
                if (lHeld) emit(manualTurn(-5))
                else       emit(courseCmd(if (rHeld) -10f else -1f))
            }
            event == 'A' && btn == 2 -> {   // A = thumb right
                if (lHeld) emit(manualTurn(+5))
                else       emit(courseCmd(if (rHeld) +10f else +1f))
            }
            event == 'A' && btn == 4 -> {   // C = thumb up
                if (lHeld) emit(speedBothCmd(+5))
                else       emit(speedBothCmd(if (rHeld) +5 else +1))
            }
            event == 'A' && btn == 5 -> {   // D = thumb down
                if (lHeld) emit(speedBothCmd(-5))
                else       emit(speedBothCmd(if (rHeld) -5 else -1))
            }
        }
    }

    // ── Joystick (Manual Override only while L held) ──────────────────────────

    private fun handleJoystick(dir: Int) {
        stopJoyRepeat()
        if (dir == 0) return          // D0 = released

        // Only active in manual override (L held) and autopilot course nudge otherwise
        val cmd = when (dir) {
            1    -> if (lHeld) speedBothCmd(+1)   else null   // up
            2    -> if (lHeld) speedBothCmd(-1)   else null   // down
            3    -> if (lHeld) manualTurn(-1)     else courseCmd(-1f)   // left
            4    -> if (lHeld) manualTurn(+1)     else courseCmd(+1f)   // right
            5    -> if (lHeld) manualTurn(-1)     else null   // up-left → turn left
            6    -> if (lHeld) manualTurn(-1)     else null   // down-left
            7    -> if (lHeld) manualTurn(+1)     else null   // up-right → turn right
            8    -> if (lHeld) manualTurn(+1)     else null   // down-right
            else -> null
        } ?: return

        lastJoyDir = dir
        emit(cmd)

        // Auto-repeat while held
        joyRepeatRunnable = object : Runnable {
            override fun run() {
                emit(cmd)
                mainHandler.postDelayed(this, JOY_REPEAT_MS)
            }
        }
        mainHandler.postDelayed(joyRepeatRunnable!!, JOY_REPEAT_MS)
    }

    private fun stopJoyRepeat() {
        joyRepeatRunnable?.let { mainHandler.removeCallbacks(it) }
        joyRepeatRunnable = null
        lastJoyDir = 0
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

    private fun courseCmd(delta: Float): RemoteBleManager.RemoteCommand {
        val value = when (delta) {
            -10f -> RemoteBleManager.CRS_LEFT_10
            +10f -> RemoteBleManager.CRS_RIGHT_10
            -1f  -> RemoteBleManager.CRS_LEFT_1
            else -> RemoteBleManager.CRS_RIGHT_1
        }
        return RemoteBleManager.RemoteCommand(RemoteBleManager.GRP_COURSE, value)
    }

    private fun speedBothCmd(step: Int): RemoteBleManager.RemoteCommand {
        val v = when {
            step >= 5  -> RemoteBleManager.SPD_UP
            step <= -5 -> RemoteBleManager.SPD_DOWN
            step > 0   -> RemoteBleManager.SPD_UP_1
            else       -> RemoteBleManager.SPD_DOWN_1
        }
        return RemoteBleManager.RemoteCommand(RemoteBleManager.GRP_SPEED, v)
    }

    /**
     * Manual turn: positive = turn starboard (PORT faster, STBD slower).
     * Emits PORT command; STBD is the mirror (handled in ControlActivity
     * by the existing per-motor handler when sync is off).
     */
    private fun manualTurn(step: Int): RemoteBleManager.RemoteCommand {
        val v = if (step > 0) RemoteBleManager.MOT_UP else RemoteBleManager.MOT_DOWN
        return RemoteBleManager.RemoteCommand(RemoteBleManager.GRP_PORT, v)
    }

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
                Log.i(TAG, "Lookbon connected")
                mainHandler.post { onConnected?.invoke() }
            }
            override fun onDeviceFailedToConnect(d: BluetoothDevice, reason: Int) {
                Log.w(TAG, "Lookbon failed: $reason")
                mainHandler.post { onDisconnected?.invoke() }
            }
            override fun onDeviceDisconnected(d: BluetoothDevice, reason: Int) {
                Log.i(TAG, "Lookbon disconnected")
                mainHandler.post { onDisconnected?.invoke() }
            }
        })
        connect(device).useAutoConnect(false).enqueue()
    }

    override fun close() {
        stopJoyRepeat()
        super.close()
    }
}