package com.escbleapp

import android.bluetooth.BluetoothDevice
import android.bluetooth.BluetoothGatt
import android.bluetooth.BluetoothGattCharacteristic
import android.content.Context
import android.util.Log
import no.nordicsemi.android.ble.BleManager
import java.util.UUID

/**
 * AC6328 BLE Manager — ESC_BLE_App
 *
 * ── Service ae00 ─────────────────────────────────────────────────────────────
 *
 *   ae03  WRITE_WITHOUT_RESPONSE  Android → MCU  5-byte motor command
 *   ae02  NOTIFY                  MCU → Android  5-byte command echo
 *   ae10  READ | WRITE            MCU → Android  status string
 *                                 Android → MCU  1-byte mode switch
 *
 * ── ae03 / ae02 Packet (5 bytes) ─────────────────────────────────────────────
 *
 *   [0]   CMD byte
 *           CMD_ESC_PWM   0x01   ESC pulse-width
 *           CMD_BLDC_DUTY 0x02   BLDC duty cycle
 *           CMD_STOP      0xFF   Stop both motors
 *   [1-2] port_value      little-endian uint16
 *   [3-4] stbd_value      little-endian uint16
 *
 *   CMD_ESC_PWM   values: 1000–2000 µs
 *   CMD_BLDC_DUTY values: 0–10000   (0=stop, 10000=100%, default start=500)
 *   CMD_STOP      values: ignored
 *
 * ── ae10 WRITE (1 byte) — mode switch ────────────────────────────────────────
 *   0x01 → ESC mode     0x02 → BLDC mode
 *
 * ── ae10 READ — status response (ASCII) ──────────────────────────────────────
 *   "M<mode>A<vbat_mv>T<uptime_min>"
 *   e.g. "M1A3712T5"   M1=ESC  M2=BLDC
 */
class AC6328BleManager(context: Context) : BleManager(context) {

    companion object {
        private const val TAG = "AC6328BleManager"

        val SERVICE_UUID   = UUID.fromString("0000ae00-0000-1000-8000-00805f9b34fb")
        val CHAR_AE03_UUID = UUID.fromString("0000ae03-0000-1000-8000-00805f9b34fb")
        val CHAR_AE02_UUID = UUID.fromString("0000ae02-0000-1000-8000-00805f9b34fb")
        val CHAR_AE10_UUID = UUID.fromString("0000ae10-0000-1000-8000-00805f9b34fb")

        // CMD byte values — must match firmware
        const val CMD_ESC_PWM:   Byte = 0x01
        const val CMD_BLDC_DUTY: Byte = 0x02
        const val CMD_STOP:      Byte = 0xFF.toByte()

        // Mode bytes for ae10 WRITE
        const val MODE_ESC:  Byte = 0x01
        const val MODE_BLDC: Byte = 0x02

        // ESC range (µs)
        const val ESC_MIN     = 1000
        const val ESC_MAX     = 2000

        // BLDC duty range — firmware: 0=0%, 10000=100%  default start: 500 (5%)
        const val BLDC_MIN     = 0
        const val BLDC_DEFAULT = 500
        const val BLDC_MAX     = 10000
    }

    // ── Characteristics ───────────────────────────────────────────────────────
    private var charAe03: BluetoothGattCharacteristic? = null
    private var charAe02: BluetoothGattCharacteristic? = null
    private var charAe10: BluetoothGattCharacteristic? = null

    // ── Feedback data ─────────────────────────────────────────────────────────
    data class FeedbackData(
        // From ae10 READ
        val mode:       Int    = -1,   // 1=ESC 2=BLDC
        val batteryMv:  Int    = -1,
        val uptimeMin:  Int    = -1,
        val rawAe10:    String = "",
        // From ae02 NOTIFY echo
        val echoCmd:    Byte   = 0,
        val echoPort:   Int    = -1,
        val echoStbd:   Int    = -1,
        val rawAe02:    ByteArray = ByteArray(0),
        val source:     String = ""
    )

    // ── Callbacks ─────────────────────────────────────────────────────────────
    var onFeedback:   ((FeedbackData) -> Unit)? = null
    var onAe02Raw:    ((ByteArray) -> Unit)?    = null
    var onError:      ((String) -> Unit)?       = null

    // ── GATT ──────────────────────────────────────────────────────────────────
    override fun getGattCallback(): BleManagerGattCallback = GattCallback()

    private inner class GattCallback : BleManagerGattCallback() {

        override fun isRequiredServiceSupported(gatt: BluetoothGatt): Boolean {
            Log.i(TAG, "Services (${gatt.services.size}):")
            gatt.services.forEach { svc ->
                Log.i(TAG, "  ${svc.uuid}  chars=${svc.characteristics.size}")
                svc.characteristics.forEach { c -> Log.i(TAG, "    ${c.uuid}  props=${c.properties}") }
            }

            val svc = gatt.getService(SERVICE_UUID) ?: run {
                Log.e(TAG, "Service ae00 not found"); return false
            }
            charAe03 = svc.getCharacteristic(CHAR_AE03_UUID)
            charAe02 = svc.getCharacteristic(CHAR_AE02_UUID)
            charAe10 = svc.getCharacteristic(CHAR_AE10_UUID)

            Log.i(TAG, "ae03=${charAe03 != null} ae02=${charAe02 != null} ae10=${charAe10 != null}")
            if (charAe03 == null) { Log.e(TAG, "ae03 missing"); return false }
            return true
        }

        override fun initialize() {
            // ae02 NOTIFY — command echo from firmware
            charAe02?.also { char ->
                setNotificationCallback(char).with { _, data ->
                    val bytes = data.value ?: return@with
                    Log.d(TAG, "ae02 echo ${bytes.size}B: ${bytes.hex()}")
                    onFeedback?.invoke(parseEcho(bytes))
                    onAe02Raw?.invoke(bytes)
                }
                enableNotifications(char).enqueue()
                Log.i(TAG, "ae02 notifications enabled")
            }
        }

        override fun onServicesInvalidated() {
            charAe03 = null; charAe02 = null; charAe10 = null
        }
    }

    override fun log(priority: Int, message: String) { Log.println(priority, TAG, message) }

    // ── Motor commands ────────────────────────────────────────────────────────

    /**
     * Send ESC pulse-width command via CMD_ESC_PWM (0x01).
     * portUs / stbdUs: 1000–2000 µs
     */
    fun sendEscPwm(portUs: Int, stbdUs: Int) {
        val p = portUs.coerceIn(ESC_MIN, ESC_MAX)
        val s = stbdUs.coerceIn(ESC_MIN, ESC_MAX)
        writeCommand(buildPacket(CMD_ESC_PWM, p, s))
        Log.d(TAG, "ESC → port=${p}µs stbd=${s}µs")
    }

    /**
     * Send BLDC duty-cycle command via CMD_BLDC_DUTY (0x02).
     * portDuty / stbdDuty: 0–10000  (0=stop, 10000=100%, default start=500)
     */
    fun sendBldc(portDuty: Int, stbdDuty: Int) {
        val p = portDuty.coerceIn(BLDC_MIN, BLDC_MAX)
        val s = stbdDuty.coerceIn(BLDC_MIN, BLDC_MAX)
        writeCommand(buildPacket(CMD_BLDC_DUTY, p, s))
        Log.d(TAG, "BLDC → port=${p} stbd=${s}  (${p/100}% ${s/100}%)")
    }

    /**
     * Stop both motors immediately via CMD_STOP (0xFF).
     * Values are ignored by firmware but we send zeros for clarity.
     */
    fun stopMotors() {
        writeCommand(buildPacket(CMD_STOP, 0, 0))
        Log.d(TAG, "STOP sent")
    }

    /**
     * Arm ESC: send minimum throttle (1000µs) so ESC completes its arm sequence.
     */
    fun armEsc() = sendEscPwm(ESC_MIN, ESC_MIN)

    // ── Mode switch via ae10 WRITE ────────────────────────────────────────────

    /**
     * Switch firmware mode over BLE (ae10 WRITE, 1 byte).
     * Firmware will stop motors and update its internal motor_mode.
     */
    fun setMode(mode: Byte) {
        val char = charAe10 ?: run { Log.w(TAG, "ae10 not available"); return }
        writeCharacteristic(char, byteArrayOf(mode),
            BluetoothGattCharacteristic.WRITE_TYPE_DEFAULT)
            .done  { Log.i(TAG, "Mode set: ${if (mode == MODE_ESC) "ESC" else "BLDC"}") }
            .fail  { _, s -> Log.e(TAG, "Mode write failed: $s") }
            .enqueue()
    }

    fun setEscMode()  = setMode(MODE_ESC)
    fun setBldcMode() = setMode(MODE_BLDC)

    // ── Status READ from ae10 ─────────────────────────────────────────────────

    /**
     * Poll ae10 READ for device status (battery, mode, uptime).
     * Response format: "M<mode>A<vbat_mv>T<uptime_min>"
     * Result delivered via onFeedback with source="ae10-read".
     */
    fun readStatus() {
        val char = charAe10 ?: run { Log.w(TAG, "ae10 not available"); return }
        readCharacteristic(char)
            .with { _, data ->
                val bytes = data.value ?: return@with
                val ascii = String(bytes).trimEnd('\u0000', ' ')
                Log.d(TAG, "ae10 READ: '$ascii'")
                onFeedback?.invoke(FeedbackData(
                    mode      = parseField(ascii, 'M'),
                    batteryMv = parseField(ascii, 'A'),
                    uptimeMin = parseField(ascii, 'T'),
                    rawAe10   = ascii,
                    source    = "ae10-read"
                ))
            }
            .fail { _, s -> Log.w(TAG, "ae10 READ failed: $s") }
            .enqueue()
    }

    // ── Packet building ───────────────────────────────────────────────────────

    /** Build a 5-byte command packet */
    private fun buildPacket(cmd: Byte, portVal: Int, stbdVal: Int): ByteArray =
        byteArrayOf(
            cmd,
            (portVal        and 0xFF).toByte(),
            ((portVal shr 8) and 0xFF).toByte(),
            (stbdVal        and 0xFF).toByte(),
            ((stbdVal shr 8) and 0xFF).toByte()
        )

    private fun writeCommand(payload: ByteArray) {
        val char = charAe03 ?: run { onError?.invoke("Not connected"); return }
        writeCharacteristic(char, payload, BluetoothGattCharacteristic.WRITE_TYPE_NO_RESPONSE)
            .fail { _, s -> Log.e(TAG, "Write failed: $s"); onError?.invoke("Write failed ($s)") }
            .enqueue()
    }

    // ── Echo parsing ──────────────────────────────────────────────────────────

    /** Parse a 5-byte ae02 echo packet into FeedbackData */
    private fun parseEcho(bytes: ByteArray): FeedbackData {
        if (bytes.size < 5) return FeedbackData(rawAe02 = bytes, source = "ae02-short")
        val cmd  = bytes[0]
        val port = le16(bytes, 1)
        val stbd = le16(bytes, 3)
        return FeedbackData(echoCmd = cmd, echoPort = port, echoStbd = stbd,
            rawAe02 = bytes, source = "ae02-echo")
    }

    // ── Parse helpers ─────────────────────────────────────────────────────────

    /** Extract numeric value after a tag letter: parseField("M1A3712T5", 'A') → 3712 */
    fun parseField(raw: String, tag: Char): Int =
        Regex("$tag(\\d+)").find(raw)?.groupValues?.get(1)?.toIntOrNull() ?: -1

    /** Battery mV → 0–100% (assumes 1S LiPo: 3.0V=0%, 4.2V=100%) */
    fun battMvToPercent(mv: Int): Int =
        if (mv <= 0) -1
        else ((mv - 3000).toFloat() / 1200f * 100).toInt().coerceIn(0, 100)

    // ── Utils ─────────────────────────────────────────────────────────────────

    private fun le16(b: ByteArray, offset: Int) =
        (b[offset].toInt() and 0xFF) or ((b[offset + 1].toInt() and 0xFF) shl 8)

    private fun ByteArray.hex() = joinToString(" ") { "%02X".format(it) }

    // ── Connection ────────────────────────────────────────────────────────────

    fun connectToDevice(device: BluetoothDevice) {
        connect(device)
            .useAutoConnect(false)
            .retry(3, 500)
            .timeout(15_000)
            .done  { Log.i(TAG, "Connected OK") }
            .fail  { _, s ->
                Log.e(TAG, "Connection failed: $s")
                onError?.invoke("Connection failed ($s)")
            }
            .enqueue()
    }

    fun isDeviceConnected() = isConnected
}