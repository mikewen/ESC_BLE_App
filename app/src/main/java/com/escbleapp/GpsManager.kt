package com.escbleapp

import android.Manifest
import android.content.Context
import android.content.pm.PackageManager
import android.location.Location
import android.os.Environment
import android.os.Looper
import android.util.Log
import androidx.core.content.ContextCompat
import com.google.android.gms.location.*
import java.io.File
import java.io.FileWriter
import java.io.PrintWriter
import java.text.SimpleDateFormat
import java.util.Date
import java.util.Locale
import kotlin.math.*

/**
 * GpsManager — Android wrapper around [SensorFusion].
 *
 * Responsibilities:
 *   - Phone GPS via FusedLocationProviderClient → SensorFusion.processNmeaRmc/Gga
 *   - ae02 BLE bytes routing:
 *       0xA1 → SensorFusion.processA1  (IMU+Mag 50Hz)
 *       0xA2 → SensorFusion.processA2  (LC02H heading 1Hz)
 *       0xA3 → SensorFusion.processA3  (position 0.2Hz)
 *       0xBA 0xCE → CASIC NAV2-SOL parser → SensorFusion.processCasicNav2Sol
 *       '$' → NMEA text (ignored — AC6329C firmware handles LC02H NMEA)
 *   - Trip distance and max speed accumulation
 *   - CSV logging to Downloads/
 *   - Source preference (phone vs BLE)
 *
 * All sensor fusion math lives in [SensorFusion].
 */
class GpsManager(private val context: Context) {

    companion object {
        private const val TAG = "GpsManager"
    }

    // ── Public data types (mirrors SensorFusion.FusedState for API compatibility) ──

    enum class Source { NONE, PHONE, BLE }

    data class GpsData(
        val source:            Source  = Source.NONE,
        val speedKnots:        Float   = 0f,
        val headingDeg:        Float   = 0f,
        val hasHeading:        Boolean = false,
        val hasFix:            Boolean = false,
        val satellites:        Int     = 0,
        val altitudeM:         Float   = 0f,
        val latDeg:            Double  = 0.0,
        val lonDeg:            Double  = 0.0,
        val speedAccMs:        Float   = Float.MAX_VALUE,
        val headingConfidence: Float   = 0f,
    ) {
        val speedKmh: Float get() = speedKnots * 1.852f
        val headingCardinal: String get() = when {
            !hasHeading -> "—"
            headingDeg < 22.5 || headingDeg >= 337.5 -> "N"
            headingDeg < 67.5  -> "NE"
            headingDeg < 112.5 -> "E"
            headingDeg < 157.5 -> "SE"
            headingDeg < 202.5 -> "S"
            headingDeg < 247.5 -> "SW"
            headingDeg < 292.5 -> "W"
            else               -> "NW"
        }
    }

    // ── Fusion engine ─────────────────────────────────────────────────────────

    val fusion = SensorFusion()

    init {
        // Load saved calibration offsets (mag hard-iron, gyro bias)
        val calPrefs = context.getSharedPreferences("calibration_prefs", Context.MODE_PRIVATE)
        CalibrationActivity.loadCalibration(calPrefs, fusion)
    }

    private var currentData  = GpsData()
    private var currentSource = Source.NONE

    // ── Callbacks ─────────────────────────────────────────────────────────────

    var onUpdate:    ((GpsData) -> Unit)? = null
    var onNmeaDebug: ((String)  -> Unit)? = null
    var onLogStatus: ((String)  -> Unit)? = null

    /** Set by AutopilotActivity on engage — receives SENS and GPS log rows. */
    var autopilotLogger: AutopilotLogger? = null

    // ── Hardware configuration ────────────────────────────────────────────────
    /**
     * Set true if QMI8658C accelerometer X/Y axes are mounted 180° from MMC5603
     * on the PCB (i.e. they face opposite directions).
     * Effect: ax = -ax, ay = -ay before tilt compensation.
     */
    var accelRotated180: Boolean = false

    // ── Second BLE sensor ─────────────────────────────────────────────────────
    // Optional AC6329C whose ae02 feeds the same SensorFusion instance.
    // A1/A2/A3 from either sensor processed identically — just merge the streams.

    private var sensor2Ble: AC6328BleManager? = null
    var onSensor2Status: ((String) -> Unit)? = null

    fun connectSensor2(device: android.bluetooth.BluetoothDevice, name: String) {
        sensor2Ble?.disconnect()?.enqueue()
        sensor2Ble?.close()
        sensor2Ble = AC6328BleManager(context)
        sensor2Ble!!.setConnectionObserver(object : no.nordicsemi.android.ble.observer.ConnectionObserver {
            override fun onDeviceConnecting(d: android.bluetooth.BluetoothDevice)    = Unit
            override fun onDeviceDisconnecting(d: android.bluetooth.BluetoothDevice) = Unit
            override fun onDeviceReady(d: android.bluetooth.BluetoothDevice)         = Unit
            override fun onDeviceConnected(d: android.bluetooth.BluetoothDevice) {
                Log.i(TAG, "Sensor2 connected: $name")
                onSensor2Status?.invoke("📡 $name")
            }
            override fun onDeviceFailedToConnect(d: android.bluetooth.BluetoothDevice, reason: Int) {
                Log.w(TAG, "Sensor2 failed: $reason")
                onSensor2Status?.invoke("📡 $name failed ($reason)")
            }
            override fun onDeviceDisconnected(d: android.bluetooth.BluetoothDevice, reason: Int) {
                Log.i(TAG, "Sensor2 disconnected: $reason")
                onSensor2Status?.invoke("")
            }
        })
        sensor2Ble!!.onAe02Raw = { bytes -> feedAe02Bytes(bytes) }
        sensor2Ble!!.connectToDevice(device)
    }

    fun disconnectSensor2() {
        sensor2Ble?.disconnect()?.enqueue()
        sensor2Ble?.close()
        sensor2Ble = null
        onSensor2Status?.invoke("")
    }

    val isSensor2Connected: Boolean get() = sensor2Ble != null

    var tripDistanceNm: Double = 0.0
        private set
    var maxSpeedKnots: Float = 0f
        private set
    private var lastFixLat = Double.NaN
    private var lastFixLon = Double.NaN

    fun resetTrip() {
        tripDistanceNm = 0.0; maxSpeedKnots = 0f
        lastFixLat = Double.NaN; lastFixLon = Double.NaN
    }

    // ── Source preference ─────────────────────────────────────────────────────

    private var usePhoneGps = true

    fun setPreferBleGps()   { usePhoneGps = false }
    fun setPreferPhoneGps() { usePhoneGps = true  }
    fun getCurrentData()    = currentData

    // ── Fusion callback wiring ────────────────────────────────────────────────

    init {
        fusion.onFusedHeading = { fs ->
            // Only accept BLE update if BLE is preferred or phone has no fix
            //val isBleUpdate = fs.source != "nmea"
            val isPhoneUpdate = fs.source == "nmea"

            //val phoneBlocking = currentSource == Source.PHONE && usePhoneGps && currentData.hasFix
            //if (!phoneBlocking || !isBleUpdate) {
            // Block ONLY phone updates when BLE is preferred
            if (!(isPhoneUpdate && currentSource == Source.BLE && !usePhoneGps)) {
                currentData = GpsData(
                    source            = if (fs.source == "nmea") Source.PHONE else Source.BLE,
                    speedKnots        = fs.speedKnots,
                    headingDeg        = fs.headingDeg,
                    hasHeading        = fs.hasHeading,
                    hasFix            = fs.hasFix,
                    satellites        = fs.satellites,
                    altitudeM         = fs.altM,
                    latDeg            = fs.latDeg,
                    lonDeg            = fs.lonDeg,
                    headingConfidence = fs.headingConf,
                )
                currentSource = currentData.source
                accumulateTrip(currentData)
                logData(currentData)
                Log.d(TAG, fs.debugMsg)
                onUpdate?.invoke(currentData)
            }
        }
    }

    // ── Phone GPS ─────────────────────────────────────────────────────────────

    private val fusedClient = LocationServices.getFusedLocationProviderClient(context)

    private val locationRequest = LocationRequest.Builder(
        Priority.PRIORITY_HIGH_ACCURACY, 1_000L
    )
        .setMinUpdateIntervalMillis(500)
        .setWaitForAccurateLocation(false)
        .setMinUpdateDistanceMeters(0f)
        .build()

    private val locationCallback = object : LocationCallback() {
        override fun onLocationResult(result: LocationResult) {
            val loc = result.lastLocation ?: run {
                Log.w(TAG, "locationCallback: lastLocation null"); return
            }
            Log.d(TAG, "locationCallback: lat=${loc.latitude} spd=${loc.speed}m/s acc=${loc.accuracy}m")
            if (currentSource == Source.BLE && !usePhoneGps) return

            val speedKt = loc.speed * 1.94384f
            //val cogDeg  = if (loc.hasBearing() && speedKt >= 0.3f) loc.bearing else null
            val cogDeg = if (
                loc.hasBearing() &&
                speedKt >= 2.0f &&          // ≥ ~4 knots
                loc.accuracy < 10f          // strong fix only
            ) loc.bearing else null
            //val hasFix  = loc.accuracy < 200f
            val hasFix  = loc.accuracy < 15f
            val sAccMs  = if (loc.hasSpeedAccuracy()) loc.speedAccuracyMetersPerSecond else 0.5f

            /*
            fusion.processNmeaRmc(
                speedKt  = speedKt,
                cogDeg   = cogDeg,
                hasFix   = hasFix,
                latDeg   = loc.latitude,
                lonDeg   = loc.longitude
            ) */
            fusion.processNmeaRmc(
                speedKt  = speedKt,
                cogDeg   = cogDeg,   // now often null
                hasFix   = hasFix && loc.accuracy < 10f,
                latDeg   = loc.latitude,
                lonDeg   = loc.longitude
            )
            // Manually patch speedAccMs since processNmeaRmc doesn't carry it
            currentData = currentData.copy(speedAccMs = sAccMs)
        }
    }

    fun startPhoneGps() {
        val hasPerm = hasLocationPermission()
        Log.i(TAG, "startPhoneGps: perm=$hasPerm")
        if (!hasPerm) return
        try {
            fusedClient.requestLocationUpdates(locationRequest, locationCallback, Looper.getMainLooper())
            Log.i(TAG, "Phone GPS started")
            fusedClient.lastLocation.addOnSuccessListener { loc ->
                if (loc != null) {
                    Log.i(TAG, "Last known: lat=${loc.latitude} acc=${loc.accuracy}m")
                    locationCallback.onLocationResult(
                        LocationResult.create(listOf(loc))
                    )
                } else Log.w(TAG, "No last known location")
            }
        } catch (e: SecurityException) { Log.e(TAG, "GPS security: ${e.message}") }
        catch (e: Exception)           { Log.e(TAG, "GPS start error: ${e.message}") }
    }

    fun stopPhoneGps() {
        fusedClient.removeLocationUpdates(locationCallback)
        disconnectSensor2()
    }

    fun hasLocationPermission() =
        ContextCompat.checkSelfPermission(context, Manifest.permission.ACCESS_FINE_LOCATION) ==
                PackageManager.PERMISSION_GRANTED

    // ── ae02 BLE byte routing ─────────────────────────────────────────────────

    private val nmeaBuffer  = StringBuilder()
    private val casicBuffer = mutableListOf<Byte>()
    private var casicState  = CasicState.IDLE

    private enum class CasicState { IDLE, SYNC2, LEN1, LEN2, CLASS, ID, PAYLOAD, CK }
    private var casicLen   = 0
    private var casicClass = 0.toByte()
    private var casicId    = 0.toByte()

    private val CASIC_SYNC1: Byte = 0xBA.toByte()
    private val CASIC_SYNC2: Byte = 0xCE.toByte()
    private val CASIC_CLASS: Byte = 0x11.toByte()
    private val CASIC_ID:    Byte = 0x02.toByte()
    private val CASIC_PAYLOAD_LEN = 72

    fun feedAe02Bytes(bytes: ByteArray) {
        if (bytes.isEmpty()) return
        // AC6329C fusion packets
        val b0 = bytes[0]
        if (b0 == 0xA1.toByte() || b0 == 0xA2.toByte() || b0 == 0xA3.toByte()) {
            parseAcPacket(bytes); return
        }
        // CASIC binary
        for (b in bytes) {
            when {
                casicState != CasicState.IDLE -> processCasicByte(b)
                b == CASIC_SYNC1              -> processCasicByte(b)
                else                          -> nmeaBuffer.append(b.toInt().toChar())
            }
        }
        // NMEA text extraction
        while (nmeaBuffer.contains('$')) {
            val start = nmeaBuffer.indexOf('$')
            val end   = nmeaBuffer.indexOf('\n', start)
            if (start < 0 || end < 0) break
            val sentence = nmeaBuffer.substring(start, end).trim()
            nmeaBuffer.delete(0, end + 1)
            // All BLE NMEA handled by AC6329C firmware — forward as debug
            onNmeaDebug?.invoke(sentence)
        }
        if (!nmeaBuffer.contains('$') && nmeaBuffer.length > 256) {
            onNmeaDebug?.invoke(bytes.joinToString(" ") { "%02X".format(it) })
            nmeaBuffer.clear()
        }
    }

    // ── AC6329C packet parser ─────────────────────────────────────────────────
    //
    // 0xA1 — IMU+Mag raw (20 bytes, existing format)
    // 0xA2 — PQTMTAR orientation (17 bytes):
    //   [0]     0xA2
    //   [1-4]   timeMs     uint32 LE
    //   [5]     quality    uint8  (GNSS status: 0/4/6)
    //   [6-7]   reserved   (bytes 6,7 — skipped per firmware)
    //   [8-9]   pitch      int16 LE × 0.01°
    //   [10-11] roll       int16 LE × 0.01°
    //   [12-13] heading    uint16 LE × 0.01° (unsigned 0–36000)
    //   [14-15] acc_hdg    uint16 LE × 0.001°
    //   [16]    usedSV     uint8
    //
    // 0xA3 — GNRMC position (17 bytes):
    //   [0]     0xA3
    //   [1-4]   timeMs     uint32 LE
    //   [5-8]   rawLat     int32 LE — NMEA lat × 10000 (DDMM.MMMM format)
    //   [9-12]  rawLon     int32 LE — NMEA lon × 10000 (DDDMM.MMMM format)
    //   [13-14] speedKt    uint16 LE × 0.01 kt
    //   [15-16] course     uint16 LE × 0.01°

    private fun parseAcPacket(b: ByteArray) {
        val buf = java.nio.ByteBuffer.wrap(b).order(java.nio.ByteOrder.LITTLE_ENDIAN)
        when (b[0]) {
            0xA1.toByte() -> {
                if (b.size < 20) return
                val seq = b[1].toInt() and 0xFF
                val ax  = getI2(b, 2);  val ay = getI2(b, 4);  val az = getI2(b, 6)
                val gx  = getI2(b, 8);  val gy = getI2(b, 10); val gz = getI2(b, 12)
                val mx  = getI2(b, 14); val my = getI2(b, 16); val mz = getI2(b, 18)
                fusion.processA1(ax, ay, az, gx, gy, gz, mx, my, mz,
                    System.currentTimeMillis(), accelRotated180)
                // SENS log row — decimated to senseSampleHz (default 10 Hz)
                val fsA1 = fusion.getState()
                autopilotLogger?.logSens(
                    ax = ax, ay = ay, az = az,
                    gx = gx, gy = gy, gz = gz,
                    mx = mx, my = my, mz = mz,
                    rawMagHdg = fsA1.rawMagHeadingDeg,
                    fusedHdg  = fsA1.headingDeg,
                    gyroZDps  = fusion.lastGyroZDegS,
                    tiltDeg   = fsA1.tiltDeg,
                    seaState  = fsA1.seaState
                )
            }
            0xA2.toByte() -> {
                if (b.size < 17) return
                val quality    = b[5].toInt() and 0xFF
                val pitch      = buf.getShort(8)  / 100.0f
                val roll       = buf.getShort(10) / 100.0f
                val heading    = (buf.getShort(12).toInt() and 0xFFFF) / 100.0f
                val accHeading = (buf.getShort(14).toInt() and 0xFFFF) / 1000.0f
                val usedSV     = b[16].toInt() and 0xFF
                Log.d("GpsManager", "A2: hdg=$heading acc=$accHeading qual=$quality sv=$usedSV")
                fusion.processA2(
                    tarHeadingDeg = heading,
                    pitchDeg      = pitch,
                    rollDeg       = roll,
                    tarAccDeg     = accHeading,
                    gnssQuality   = quality,
                    satellites    = usedSV,
                    nowMs         = System.currentTimeMillis()
                )
                // GPS log row — every A2 (1 Hz)
                val fs = fusion.getState()
                autopilotLogger?.logGps(
                    tarHdg      = heading,
                    rmcCog      = fusion.cachedRmcHeading,
                    blendedHdg  = fs.headingDeg,
                    gnssQuality = quality,
                    tarAccDeg   = accHeading,
                    satellites  = usedSV,
                    speedKt     = fusion.cachedRmcSpeed,
                    lat         = fs.latDeg,
                    lon         = fs.lonDeg,
                    misalignDeg = fs.tarMisalignDeg,
                    misalignCal = fs.tarMisalignCalibrated
                )
            }
            0xA3.toByte() -> {
                if (b.size < 17) return
                val rawLat   = buf.getInt(5)  / 10000.0f
                val rawLon   = buf.getInt(9)  / 10000.0f
                val speedKt  = (buf.getShort(13).toInt() and 0xFFFF) / 100.0f
                val course   = (buf.getShort(15).toInt() and 0xFFFF) / 100.0f
                val lat      = convertNmeaToDecimal(rawLat)
                val lon      = convertNmeaToDecimal(rawLon)
                val hasFix   = rawLat != 0f || rawLon != 0f
                Log.d("GpsManager", "A3: lat=${"%.6f".format(lat)} lon=${"%.6f".format(lon)} spd=$speedKt cog=$course fix=$hasFix")
                // Auto-switch to BLE GPS when A3 has a valid fix — BLE position is more accurate
                if (hasFix && usePhoneGps) {
                    Log.i("GpsManager", "A3 valid fix received — switching to BLE GPS automatically")
                    usePhoneGps = false
                }
                fusion.processA3(lat, lon, speedKt, course, hasFix)
            }
        }
    }

    /**
     * Convert NMEA coordinate format (DDMM.MMMM) to decimal degrees.
     * Input is the NMEA value already divided by 10000 (i.e. rawLat = DDMM.MMMM as float).
     */
    private fun convertNmeaToDecimal(nmea: Float): Double {
        val degrees = (nmea / 100).toInt()
        val minutes = nmea - degrees * 100f
        return degrees + minutes / 60.0
    }

    // ── CASIC NAV2-SOL parser ─────────────────────────────────────────────────

    private fun processCasicByte(b: Byte) {
        when (casicState) {
            CasicState.IDLE    -> if (b == CASIC_SYNC1) casicState = CasicState.SYNC2
            CasicState.SYNC2   -> if (b == CASIC_SYNC2) { casicState = CasicState.LEN1; casicBuffer.clear() }
            else casicState = CasicState.IDLE
            CasicState.LEN1    -> { casicLen = b.toInt() and 0xFF; casicState = CasicState.LEN2 }
            CasicState.LEN2    -> { casicLen = casicLen or ((b.toInt() and 0xFF) shl 8); casicState = CasicState.CLASS }
            CasicState.CLASS   -> { casicClass = b; casicState = CasicState.ID }
            CasicState.ID      -> { casicId = b; casicState = CasicState.PAYLOAD }
            CasicState.PAYLOAD -> {
                casicBuffer.add(b)
                if (casicBuffer.size >= casicLen) casicState = CasicState.CK
            }
            CasicState.CK -> {
                casicBuffer.add(b)
                if (casicBuffer.size >= casicLen + 4) {
                    val payload  = casicBuffer.subList(0, casicLen).toByteArray()
                    val ckBytes  = casicBuffer.subList(casicLen, casicLen + 4).toByteArray()
                    casicState   = CasicState.IDLE
                    if (verifyCasicChecksum(casicLen, casicClass, casicId, payload, ckBytes) &&
                        casicClass == CASIC_CLASS && casicId == CASIC_ID) {
                        parseCasicNav2Sol(payload)
                    }
                }
            }
        }
    }

    private fun verifyCasicChecksum(len: Int, cls: Byte, id: Byte,
                                    payload: ByteArray, ckBytes: ByteArray): Boolean {
        var ck = ((id.toInt() and 0xFF) shl 24) + ((cls.toInt() and 0xFF) shl 16) + len
        var i  = 0
        while (i + 3 < payload.size) {
            val w = ((payload[i].toInt() and 0xFF)) or
                    ((payload[i+1].toInt() and 0xFF) shl 8) or
                    ((payload[i+2].toInt() and 0xFF) shl 16) or
                    ((payload[i+3].toInt() and 0xFF) shl 24)
            ck = (ck + w) and 0xFFFFFFFF.toInt()
            i += 4
        }
        val rx = ((ckBytes[0].toInt() and 0xFF)) or ((ckBytes[1].toInt() and 0xFF) shl 8) or
                ((ckBytes[2].toInt() and 0xFF) shl 16) or ((ckBytes[3].toInt() and 0xFF) shl 24)
        return ck == rx
    }

    private fun parseCasicNav2Sol(p: ByteArray) {
        if (p.size < CASIC_PAYLOAD_LEN) return
        val fixflags = p[8].toInt() and 0xFF
        val fixOk    = (fixflags and 0x01) != 0
        val fixType  = (fixflags shr 1) and 0x07
        val x  = getR8(p, 24); val y = getR8(p, 32); val z = getR8(p, 40)
        val vx = getR4(p, 52).toDouble(); val vy = getR4(p, 56).toDouble(); val vz = getR4(p, 60).toDouble()
        val sAcc = getR4(p, 64)
        fusion.processCasicNav2Sol(x, y, z, vx, vy, vz, sAcc, fixOk, fixType)
    }

    // ── Trip accumulation ─────────────────────────────────────────────────────

    private fun accumulateTrip(data: GpsData) {
        if (!data.hasFix) return
        if (data.speedKnots > maxSpeedKnots) maxSpeedKnots = data.speedKnots
        val lat = data.latDeg; val lon = data.lonDeg
        if (!lastFixLat.isNaN() && (lat != lastFixLat || lon != lastFixLon))
            tripDistanceNm += fusion.haversineNm(lastFixLat, lastFixLon, lat, lon)
        lastFixLat = lat; lastFixLon = lon
    }

    // ── CSV Logging ───────────────────────────────────────────────────────────

    private var logWriter: PrintWriter? = null
    private var logFile:   File?        = null
    var isLogging: Boolean = false
        private set
    private val logDateFmt = SimpleDateFormat("yyyy-MM-dd'T'HH:mm:ss.SSS", Locale.US)
    private val fileNameFmt = SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US)

    fun startLogging(): String? {
        if (isLogging) return logFile?.absolutePath
        return try {
            val dir = Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_DOWNLOADS)
            dir.mkdirs()
            logFile   = File(dir, "ESC_BLE_GPS_${fileNameFmt.format(Date())}.csv")
            logWriter = PrintWriter(FileWriter(logFile!!, false))
            logWriter!!.println("timestamp,source,lat,lon,speedKt,headingDeg,altM,satellites,hasFix,headingConf")
            logWriter!!.flush()
            isLogging = true
            logFile!!.absolutePath
        } catch (e: Exception) { Log.e(TAG, "Log start error: ${e.message}"); null }
    }

    fun stopLogging() {
        if (!isLogging) return
        logWriter?.flush(); logWriter?.close(); logWriter = null
        isLogging = false
        onLogStatus?.invoke("Saved: ${logFile?.name}")
    }

    private fun logData(data: GpsData) {
        if (!isLogging || logWriter == null) return
        try {
            logWriter!!.println(
                "${logDateFmt.format(Date())},${data.source}," +
                        "${"%.8f".format(data.latDeg)},${"%.8f".format(data.lonDeg)}," +
                        "${"%.2f".format(data.speedKnots)},${"%.1f".format(data.headingDeg)}," +
                        "${"%.1f".format(data.altitudeM)},${data.satellites},${data.hasFix}," +
                        "${"%.2f".format(data.headingConfidence)}"
            )
            logWriter!!.flush()
        } catch (e: Exception) { Log.e(TAG, "Log write: ${e.message}") }
    }

    // ── Binary helpers ────────────────────────────────────────────────────────

    private fun getU2(b: ByteArray, o: Int): Int =
        (b[o].toInt() and 0xFF) or ((b[o+1].toInt() and 0xFF) shl 8)

    private fun getI2(b: ByteArray, o: Int): Short =
        ((b[o].toInt() and 0xFF) or ((b[o+1].toInt() and 0xFF) shl 8)).toShort()

    private fun getI4(b: ByteArray, o: Int): Int =
        (b[o].toInt() and 0xFF) or ((b[o+1].toInt() and 0xFF) shl 8) or
                ((b[o+2].toInt() and 0xFF) shl 16) or ((b[o+3].toInt() and 0xFF) shl 24)

    private fun getR4(b: ByteArray, o: Int): Float =
        java.lang.Float.intBitsToFloat(getI4(b, o))

    private fun getR8(b: ByteArray, o: Int): Double {
        val lo = (b[o].toLong() and 0xFF) or ((b[o+1].toLong() and 0xFF) shl 8) or
                ((b[o+2].toLong() and 0xFF) shl 16) or ((b[o+3].toLong() and 0xFF) shl 24)
        val hi = (b[o+4].toLong() and 0xFF) or ((b[o+5].toLong() and 0xFF) shl 8) or
                ((b[o+6].toLong() and 0xFF) shl 16) or ((b[o+7].toLong() and 0xFF) shl 24)
        return java.lang.Double.longBitsToDouble(lo or (hi shl 32))
    }
}