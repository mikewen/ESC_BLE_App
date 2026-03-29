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
 * BLE Remote Controller — 2-byte command packets via ae04 NOTIFY.
 *
 * ── Packet format: [GROUP, VALUE] ────────────────────────────────────────────
 *
 *  0x10  Speed (both motors)
 *        0x01 = UP +5%    0x02 = DOWN -5%   0x03 = STOP
 *        0x04 = UP +1%    0x05 = DOWN -1%
 *        0x13 nn = Set absolute speed 0–100%
 *
 *  0x11  Course adjust (autopilot target heading)
 *        0x01 = −1°   0x02 = +1°   0x0A = −10°   0x0B = +10°
 *
 *  0x12  Autopilot
 *        0x01 = ENGAGE   0x02 = DISENGAGE   0x03 = HOLD COURSE
 *
 *  0x16  Sync (PORT mirrors STBD in manual control)
 *        0x01 = SYNC ON    0x02 = SYNC OFF
 *
 *  0x14  PORT motor individual
 *        0x01 = UP +5%   0x02 = DOWN -5%
 *        0x04 = UP +1%   0x05 = DOWN -1%
 *        nn≥0x10 = absolute (nn-0x10) %
 *
 *  0x15  STBD motor individual
 *        same as PORT
 *
 * Remote firmware: expose service ae00 (or ae30), characteristic ae04 NOTIFY.
 * Advertise name containing "ESC_REMOTE", "REMOTE", "RC_CTRL", or "BLE_RC".
 */
class RemoteBleManager(context: Context) : BleManager(context) {

    companion object {
        private const val TAG = "RemoteBle"

        val SERVICE_AE00_UUID = UUID.fromString("0000ae00-0000-1000-8000-00805f9b34fb")
        val SERVICE_AE30_UUID = UUID.fromString("0000ae30-0000-1000-8000-00805f9b34fb")
        val CHAR_AE04_UUID    = UUID.fromString("0000ae04-0000-1000-8000-00805f9b34fb")

        const val GRP_SPEED     = 0x10.toByte()
        const val GRP_COURSE    = 0x11.toByte()
        const val GRP_AUTOPILOT = 0x12.toByte()
        const val GRP_SET_SPEED = 0x13.toByte()
        const val GRP_PORT      = 0x14.toByte()
        const val GRP_STBD      = 0x15.toByte()
        const val GRP_SYNC      = 0x16.toByte()   // sync: PORT mirrors STBD

        const val SPD_UP        = 0x01.toByte()   // both motors +5%
        const val SPD_DOWN      = 0x02.toByte()   // both motors -5%
        const val SPD_STOP      = 0x03.toByte()
        const val SPD_UP_1      = 0x04.toByte()   // both motors +1%
        const val SPD_DOWN_1    = 0x05.toByte()   // both motors -1%

        const val CRS_LEFT_1    = 0x01.toByte()
        const val CRS_RIGHT_1   = 0x02.toByte()
        const val CRS_LEFT_10   = 0x0A.toByte()
        const val CRS_RIGHT_10  = 0x0B.toByte()

        const val AP_ENGAGE     = 0x01.toByte()
        const val AP_DISENGAGE  = 0x02.toByte()
        const val AP_HOLD       = 0x03.toByte()

        const val SYNC_ON       = 0x01.toByte()
        const val SYNC_OFF      = 0x02.toByte()

        const val MOT_UP        = 0x01.toByte()   // +5%
        const val MOT_DOWN      = 0x02.toByte()   // -5%
        const val MOT_UP_1      = 0x04.toByte()   // +1%
        const val MOT_DOWN_1    = 0x05.toByte()   // -1%
        const val MOT_ABS_BASE  = 0x10            // value = MOT_ABS_BASE + pct (0–100)

        const val SPEED_STEP    = 5               // % per UP/DOWN press
        const val SPEED_STEP_1  = 1               // % per UP_1/DOWN_1 press

        val REMOTE_NAME_FILTERS = listOf("ESC_REMOTE", "REMOTE", "RC_CTRL", "BLE_RC")
    }

    // ── Command data class ────────────────────────────────────────────────────

    data class RemoteCommand(val group: Byte, val value: Byte) {
        private val v = value.toInt() and 0xFF

        // Both-motor speed
        val isSpeedUp:       Boolean get() = group == GRP_SPEED    && value == SPD_UP
        val isSpeedDown:     Boolean get() = group == GRP_SPEED    && value == SPD_DOWN
        val isSpeedUp1:      Boolean get() = group == GRP_SPEED    && value == SPD_UP_1
        val isSpeedDown1:    Boolean get() = group == GRP_SPEED    && value == SPD_DOWN_1
        val isStop:          Boolean get() = group == GRP_SPEED    && value == SPD_STOP
        val isSetBothSpeed:  Boolean get() = group == GRP_SET_SPEED
        val bothSpeedPct:    Int     get() = v.coerceIn(0, 100)

        val isCourse:        Boolean get() = group == GRP_COURSE
        val courseDelta:     Float   get() = when (value) {
            CRS_LEFT_1  -> -1f;  CRS_RIGHT_1  -> +1f
            CRS_LEFT_10 -> -10f; CRS_RIGHT_10 -> +10f
            else        ->  0f
        }

        val isEngage:        Boolean get() = group == GRP_AUTOPILOT && value == AP_ENGAGE
        val isDisengage:     Boolean get() = group == GRP_AUTOPILOT && value == AP_DISENGAGE
        val isHoldCourse:    Boolean get() = group == GRP_AUTOPILOT && value == AP_HOLD

        val isSyncOn:        Boolean get() = group == GRP_SYNC && value == SYNC_ON
        val isSyncOff:       Boolean get() = group == GRP_SYNC && value == SYNC_OFF

        val isPortCmd:       Boolean get() = group == GRP_PORT
        val isStbdCmd:       Boolean get() = group == GRP_STBD
        val isMotorUp:       Boolean get() = value == MOT_UP
        val isMotorDown:     Boolean get() = value == MOT_DOWN
        val isMotorUp1:      Boolean get() = value == MOT_UP_1
        val isMotorDown1:    Boolean get() = value == MOT_DOWN_1
        val isMotorAbsolute: Boolean get() = v >= MOT_ABS_BASE
        val motorAbsPct:     Int     get() = (v - MOT_ABS_BASE).coerceIn(0, 100)
    }

    // ── Callback ──────────────────────────────────────────────────────────────

    var onRemoteCommand:  ((RemoteCommand) -> Unit)? = null
    var onConnected:      (() -> Unit)?              = null
    var onDisconnected:   (() -> Unit)?              = null

    private val mainHandler = Handler(Looper.getMainLooper())

    // ── GATT ─────────────────────────────────────────────────────────────────

    private var charAe04: BluetoothGattCharacteristic? = null

    override fun getGattCallback(): BleManagerGattCallback = RemoteGattCallback()

    private inner class RemoteGattCallback : BleManagerGattCallback() {
        override fun isRequiredServiceSupported(gatt: BluetoothGatt): Boolean {
            val svc = gatt.getService(SERVICE_AE00_UUID)
                ?: gatt.getService(SERVICE_AE30_UUID)
                ?: return false
            charAe04 = svc.getCharacteristic(CHAR_AE04_UUID)
            if (charAe04 == null) Log.w(TAG, "ae04 not found on remote")
            return charAe04 != null
        }

        override fun initialize() {
            charAe04?.let { c ->
                setNotificationCallback(c).with { _, data ->
                    val bytes = data.value ?: return@with
                    if (bytes.size >= 2) {
                        val cmd = RemoteCommand(bytes[0], bytes[1])
                        Log.d(TAG, "Remote: ${"%02X %02X".format(cmd.group, cmd.value)}")
                        mainHandler.post { onRemoteCommand?.invoke(cmd) }
                    }
                }
                enableNotifications(c).enqueue()
            }
        }

        override fun onServicesInvalidated() { charAe04 = null }
    }

    fun connectToDevice(device: BluetoothDevice) {
        setConnectionObserver(object : ConnectionObserver {
            override fun onDeviceConnecting(d: BluetoothDevice)    = Unit
            override fun onDeviceDisconnecting(d: BluetoothDevice) = Unit
            override fun onDeviceReady(d: BluetoothDevice)         = Unit
            override fun onDeviceConnected(d: BluetoothDevice) {
                Log.i(TAG, "Remote connected: ${d.address}")
                mainHandler.post { onConnected?.invoke() }
            }
            override fun onDeviceFailedToConnect(d: BluetoothDevice, reason: Int) {
                Log.w(TAG, "Remote failed: $reason")
                mainHandler.post { onDisconnected?.invoke() }
            }
            override fun onDeviceDisconnected(d: BluetoothDevice, reason: Int) {
                Log.i(TAG, "Remote disconnected: $reason")
                mainHandler.post { onDisconnected?.invoke() }
            }
        })
        connect(device).useAutoConnect(false).enqueue()
    }
}