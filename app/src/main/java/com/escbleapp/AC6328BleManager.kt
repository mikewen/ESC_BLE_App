package com.escbleapp

import android.bluetooth.BluetoothDevice
import android.bluetooth.BluetoothGatt
import android.bluetooth.BluetoothGattCharacteristic
import android.content.Context
import android.util.Log
import no.nordicsemi.android.ble.BleManager
import java.util.UUID

/**
 * AC6328 / AC6329C BLE motor controller.
 *
 * Both ESC and BLDC modes use the same 50Hz PWM timer.
 * Duty units are timer counts, NOT microseconds:
 *
 *   ESC mode  (CMD_ESC_PWM  0x01): duty 500–1000
 *     500 = 1000µs (stop/arm)
 *     750 = 1500µs (mid)
 *    1000 = 2000µs (full)
 *
 *   BLDC mode (CMD_BLDC_DUTY 0x02): duty 0–10000
 *     0     = 0%  (stop)
 *    10000  = 100% (full)
 *
 * Service: ae00
 *   ae03 WRITE_WITHOUT_RESPONSE — 5-byte command packet
 *   ae02 NOTIFY                 — echo / CASIC GNSS stream
 *   ae10 READ|WRITE             — status read / mode switch write
 *
 * Packet format (ae03, 5 bytes):
 *   [CMD, portLo, portHi, stbdLo, stbdHi]  little-endian 16-bit values
 */
class AC6328BleManager(context: Context) : BleManager(context) {

    companion object {
        private const val TAG = "AC6328"

        val SERVICE_UUID   = UUID.fromString("0000ae00-0000-1000-8000-00805f9b34fb")
        val CHAR_AE03_UUID = UUID.fromString("0000ae03-0000-1000-8000-00805f9b34fb")
        val CHAR_AE02_UUID = UUID.fromString("0000ae02-0000-1000-8000-00805f9b34fb")
        val CHAR_AE10_UUID = UUID.fromString("0000ae10-0000-1000-8000-00805f9b34fb")

        const val CMD_ESC_PWM:   Byte = 0x01
        const val CMD_BLDC_DUTY: Byte = 0x02
        const val CMD_STOP:      Byte = 0xFF.toByte()

        // ESC duty range — 50Hz timer, same units as BLDC but narrower band
        // duty 500 = 1000µs (stop), duty 1000 = 2000µs (full throttle)
        const val ESC_MIN     = 500
        const val ESC_DEFAULT = 500   // stop / arm value
        const val ESC_MAX     = 1000

        // BLDC duty range — 0=stop, 10000=100%
        const val BLDC_MIN     = 0
        const val BLDC_DEFAULT = 0
        const val BLDC_MAX     = 10000
    }

    // ── Characteristics ───────────────────────────────────────────────────────

    private var charAe03: BluetoothGattCharacteristic? = null
    private var charAe02: BluetoothGattCharacteristic? = null
    private var charAe10: BluetoothGattCharacteristic? = null

    // ── Callbacks ─────────────────────────────────────────────────────────────

    var onFeedback: ((FeedbackData) -> Unit)? = null
    var onAe02Raw:  ((ByteArray)    -> Unit)? = null
    var onError:    ((String)       -> Unit)? = null

    // ── Nordic BleManager overrides ───────────────────────────────────────────

    override fun getGattCallback(): BleManagerGattCallback = AC6328GattCallback()

    private inner class AC6328GattCallback : BleManagerGattCallback() {

        override fun isRequiredServiceSupported(gatt: BluetoothGatt): Boolean {
            val svc = gatt.getService(SERVICE_UUID) ?: return false
            charAe03 = svc.getCharacteristic(CHAR_AE03_UUID)
            charAe02 = svc.getCharacteristic(CHAR_AE02_UUID)
            charAe10 = svc.getCharacteristic(CHAR_AE10_UUID)
            return charAe03 != null
        }

        override fun initialize() {
            charAe02?.let { c ->
                setNotificationCallback(c).with { _, data ->
                    data.value?.let { bytes ->
                        onAe02Raw?.invoke(bytes)
                        parseAe02Echo(bytes)
                    }
                }
                enableNotifications(c).enqueue()
            }
        }

        override fun onServicesInvalidated() {
            charAe03 = null; charAe02 = null; charAe10 = null
        }
    }

    // ── Commands ──────────────────────────────────────────────────────────────

    /**
     * Send ESC duty command.
     * duty: 500–1000  (500=stop/1000µs, 1000=full/2000µs)
     */
    fun sendEscPwm(portDuty: Int, stbdDuty: Int) {
        val p = portDuty.coerceIn(ESC_MIN, ESC_MAX)
        val s = stbdDuty.coerceIn(ESC_MIN, ESC_MAX)
        Log.d(TAG, "ESC duty port=$p stbd=$s  (${dutyToUs(p)}µs / ${dutyToUs(s)}µs)")
        writeCommand(buildPacket(CMD_ESC_PWM, p, s))
    }

    /**
     * Send BLDC duty command.
     * duty: 0–10000  (0=stop, 10000=100%)
     */
    fun sendBldc(portDuty: Int, stbdDuty: Int) {
        val p = portDuty.coerceIn(BLDC_MIN, BLDC_MAX)
        val s = stbdDuty.coerceIn(BLDC_MIN, BLDC_MAX)
        Log.d(TAG, "BLDC duty port=$p stbd=$s")
        writeCommand(buildPacket(CMD_BLDC_DUTY, p, s))
    }

    /** Stop both motors immediately — sends CMD_STOP (0xFF) */
    fun stopMotors() {
        Log.d(TAG, "STOP sent")
        writeCommand(buildPacket(CMD_STOP, 0, 0))
    }

    /** Switch firmware to ESC mode */
    fun setEscMode()  { charAe10?.let { writeCharacteristic(it, byteArrayOf(0x01), BluetoothGattCharacteristic.WRITE_TYPE_DEFAULT).enqueue() } }

    /** Switch firmware to BLDC mode */
    fun setBldcMode() { charAe10?.let { writeCharacteristic(it, byteArrayOf(0x02), BluetoothGattCharacteristic.WRITE_TYPE_DEFAULT).enqueue() } }

    /** Read ae10 status: "M<mode>A<vbat_mv>T<uptime_min>" */
    fun readStatus() {
        charAe10?.let {
            readCharacteristic(it).with { _, data ->
                val raw = data.value?.toString(Charsets.UTF_8) ?: return@with
                onFeedback?.invoke(parseAe10Status(raw))
            }.enqueue()
        }
    }

    /** Arm ESC: send stop duty so ESC completes its arm sequence */
    fun armEsc() = sendEscPwm(ESC_MIN, ESC_MIN)

    // ── Battery helper ────────────────────────────────────────────────────────

    fun battMvToPercent(mv: Int): Int = when {
        mv >= 4200 -> 100
        mv >= 3700 -> 60 + (mv - 3700) * 40 / 500
        mv >= 3500 -> 20 + (mv - 3500) * 40 / 200
        mv >= 3300 -> (mv - 3300) * 20 / 200
        else       -> 0
    }.coerceIn(0, 100)

    // ── Packet builder ────────────────────────────────────────────────────────

    private fun buildPacket(cmd: Byte, port: Int, stbd: Int): ByteArray = byteArrayOf(
        cmd,
        (port and 0xFF).toByte(), ((port shr 8) and 0xFF).toByte(),
        (stbd and 0xFF).toByte(), ((stbd shr 8) and 0xFF).toByte()
    )

    private fun writeCommand(bytes: ByteArray) {
        charAe03?.let {
            writeCharacteristic(it, bytes, BluetoothGattCharacteristic.WRITE_TYPE_NO_RESPONSE).enqueue()
        }
    }

    // ── ae02 echo parser ──────────────────────────────────────────────────────

    private fun parseAe02Echo(bytes: ByteArray) {
        if (bytes.size < 5) return
        val cmd      = bytes[0]
        val portVal  = (bytes[1].toInt() and 0xFF) or ((bytes[2].toInt() and 0xFF) shl 8)
        val stbdVal  = (bytes[3].toInt() and 0xFF) or ((bytes[4].toInt() and 0xFF) shl 8)
        onFeedback?.invoke(FeedbackData(
            source    = "ae02-echo",
            echoCmd   = cmd.toInt() and 0xFF,
            echoPort  = portVal,
            echoStbd  = stbdVal,
            rawAe02   = bytes
        ))
    }

    // ── ae10 status parser ────────────────────────────────────────────────────

    private fun parseAe10Status(raw: String): FeedbackData {
        // Format: "M<mode>A<vbat_mv>T<uptime_min>"
        val mode      = Regex("M(\\d+)").find(raw)?.groupValues?.get(1)?.toIntOrNull() ?: 0
        val battMv    = Regex("A(\\d+)").find(raw)?.groupValues?.get(1)?.toIntOrNull() ?: 0
        val uptimeMin = Regex("T(\\d+)").find(raw)?.groupValues?.get(1)?.toIntOrNull() ?: -1
        return FeedbackData(source = "ae10-read", batteryMv = battMv,
            uptimeMin = uptimeMin, rawAe10 = raw)
    }

    // ── Connect helper ────────────────────────────────────────────────────────

    fun connectToDevice(device: BluetoothDevice) {
        connect(device).useAutoConnect(false).retry(3, 200).enqueue()
    }

    // ── Feedback data ─────────────────────────────────────────────────────────

    data class FeedbackData(
        val source:    String   = "",
        val batteryMv: Int      = 0,
        val uptimeMin: Int      = -1,
        val rawAe10:   String   = "",
        val echoCmd:   Int      = -1,
        val echoPort:  Int      = -1,
        val echoStbd:  Int      = -1,
        val rawAe02:   ByteArray = ByteArray(0)
    )

    // ── Utility ───────────────────────────────────────────────────────────────

    /** Convert ESC duty unit to microseconds for display */
    fun dutyToUs(duty: Int): Int = duty * 2   // 50Hz: period=20000µs, 1 duty unit = 20000/10000 = 2µs
}
