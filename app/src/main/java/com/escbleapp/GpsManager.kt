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
 * GpsManager — dual-source GPS: phone FLP + BLE NMEA/UBX via ae02.
 *
 * Sources:
 *   Phone GPS  — FusedLocationProviderClient, 1 Hz
 *   BLE GPS    — NMEA sentences ($GPRMC/$GPGGA) or u-blox UBX binary
 *                streamed via ae02 NOTIFY from AC6329C
 *
 * UBX messages parsed:
 *   NAV-PVT  (0x01/0x07) — position, velocity, time, heading-of-motion
 *                           Most accurate heading from u-blox modules
 *   NAV2-SOL (0x29/0x03) — legacy solution (older receivers)
 *
 * NMEA sentences parsed:
 *   $GPRMC/$GNRMC — speed, heading, lat/lon
 *   $GPGGA/$GNGGA — fix quality, satellites, altitude
 *
 * GPS logging:
 *   Call startLogging() / stopLogging().
 *   Writes CSV to Downloads/ESC_BLE_GPS_<timestamp>.csv
 *   Columns: timestamp, source, lat, lon, speedKt, headingDeg, altM, sats, hasFix
 */
class GpsManager(private val context: Context) {

    companion object {
        private const val TAG = "GpsManager"
        private const val MIN_SPEED_FOR_HEADING_KT = 0.3f

    }

    // ── Data ──────────────────────────────────────────────────────────────────

    enum class Source { NONE, PHONE, BLE }

    data class GpsData(
        val source:     Source  = Source.NONE,
        val speedKnots: Float   = 0f,
        val headingDeg: Float   = 0f,
        val hasHeading: Boolean = false,
        val hasFix:     Boolean = false,
        val satellites: Int     = 0,
        val altitudeM:  Float   = 0f,
        val latDeg:     Double  = 0.0,
        val lonDeg:     Double  = 0.0,
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

    // ── Trip stats ────────────────────────────────────────────────────────────

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

    // ── Callbacks ─────────────────────────────────────────────────────────────

    var onUpdate:    ((GpsData) -> Unit)? = null
    var onNmeaDebug: ((String)  -> Unit)? = null   // non-GPS ae02 hex
    var onLogStatus: ((String)  -> Unit)? = null   // logging status messages

    // ── State ─────────────────────────────────────────────────────────────────

    private var currentData    = GpsData()
    private var usePhoneGps    = true
    private var phoneGpsActive = false

    // ── Logging ───────────────────────────────────────────────────────────────

    private var logWriter:     PrintWriter? = null
    private var logFile:       File?        = null
    var isLogging: Boolean = false
        private set
    private val logDateFmt = SimpleDateFormat("yyyy-MM-dd'T'HH:mm:ss.SSS", Locale.US)
    private val fileNameFmt = SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US)

    /**
     * Start CSV logging to Downloads/ESC_BLE_GPS_<timestamp>.csv
     * Returns the file path on success, null on failure.
     */
    fun startLogging(): String? {
        if (isLogging) return logFile?.absolutePath
        return try {
            val dir = Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_DOWNLOADS)
            dir.mkdirs()
            val fname = "ESC_BLE_GPS_${fileNameFmt.format(Date())}.csv"
            logFile = File(dir, fname)
            logWriter = PrintWriter(FileWriter(logFile!!, false))
            // CSV header
            logWriter!!.println("timestamp,source,lat,lon,speedKt,headingDeg,altM,satellites,hasFix")
            logWriter!!.flush()
            isLogging = true
            Log.i(TAG, "GPS logging started: ${logFile!!.absolutePath}")
            logFile!!.absolutePath
        } catch (e: Exception) {
            Log.e(TAG, "Failed to start logging: ${e.message}")
            null
        }
    }

    fun stopLogging() {
        if (!isLogging) return
        logWriter?.flush()
        logWriter?.close()
        logWriter = null
        isLogging = false
        Log.i(TAG, "GPS logging stopped: ${logFile?.absolutePath}")
        onLogStatus?.invoke("Saved: ${logFile?.name}")
    }

    private fun logData(data: GpsData) {
        if (!isLogging || logWriter == null) return
        try {
            logWriter!!.println(
                "${logDateFmt.format(Date())}," +
                        "${data.source}," +
                        "${"%.8f".format(data.latDeg)}," +
                        "${"%.8f".format(data.lonDeg)}," +
                        "${"%.2f".format(data.speedKnots)}," +
                        "${"%.1f".format(data.headingDeg)}," +
                        "${"%.1f".format(data.altitudeM)}," +
                        "${data.satellites}," +
                        "${data.hasFix}"
            )
            logWriter!!.flush()
        } catch (e: Exception) {
            Log.e(TAG, "Log write error: ${e.message}")
        }
    }

    // ── Phone GPS ─────────────────────────────────────────────────────────────

    private val fusedClient = LocationServices.getFusedLocationProviderClient(context)

    private val locationRequest = LocationRequest.Builder(
        Priority.PRIORITY_HIGH_ACCURACY, 1_000L
    ).setMinUpdateIntervalMillis(500).build()

    private val locationCallback = object : LocationCallback() {
        override fun onLocationResult(result: LocationResult) {
            val loc = result.lastLocation ?: return
            if (currentData.source != Source.BLE || usePhoneGps) fromPhoneLocation(loc)
        }
    }

    fun startPhoneGps() {
        if (!hasLocationPermission()) { Log.w(TAG, "No location permission"); return }
        try {
            fusedClient.requestLocationUpdates(locationRequest, locationCallback, Looper.getMainLooper())
            phoneGpsActive = true
            Log.i(TAG, "Phone GPS started")
        } catch (e: SecurityException) { Log.e(TAG, "GPS security: ${e.message}") }
    }

    fun stopPhoneGps() {
        fusedClient.removeLocationUpdates(locationCallback)
        phoneGpsActive = false
    }

    fun hasLocationPermission() =
        ContextCompat.checkSelfPermission(context, Manifest.permission.ACCESS_FINE_LOCATION) ==
                PackageManager.PERMISSION_GRANTED

    fun setPreferBleGps()   { usePhoneGps = false }
    fun setPreferPhoneGps() { usePhoneGps = true  }
    fun getCurrentData()    = currentData

    private fun fromPhoneLocation(loc: Location) {
        val speedKt   = loc.speed * 1.94384f
        val hasHdg    = loc.hasBearing() && speedKt >= MIN_SPEED_FOR_HEADING_KT
        currentData   = GpsData(
            source     = Source.PHONE,
            speedKnots = speedKt,
            headingDeg = if (hasHdg) loc.bearing else currentData.headingDeg,
            hasHeading = hasHdg,
            hasFix     = loc.accuracy < 50f,
            altitudeM  = loc.altitude.toFloat(),
            latDeg     = loc.latitude,
            lonDeg     = loc.longitude,
            satellites = currentData.satellites
        )
        accumulateTrip(currentData)
        logData(currentData)
        onUpdate?.invoke(currentData)
    }

    // ── ae02 byte feed — detects CASIC binary vs NMEA text ─────────────────────

    // CASIC NAV2-SOL constants
    // Sync: 0xBA 0xCE  |  Class: 0x11  |  ID: 0x02  |  Payload: 72 bytes
    private val CASIC_SYNC1:  Byte = 0xBA.toByte()
    private val CASIC_SYNC2:  Byte = 0xCE.toByte()
    private val CASIC_CLASS:  Byte = 0x11.toByte()
    private val CASIC_ID:     Byte = 0x02.toByte()
    private val CASIC_PAYLOAD_LEN = 72

    private val nmeaBuffer  = StringBuilder()
    private val casicBuffer = mutableListOf<Byte>()
    private var casicState  = CasicState.IDLE

    private enum class CasicState { IDLE, SYNC2, LEN1, LEN2, CLASS, ID, PAYLOAD, CK }
    private var casicLen  = 0
    private var casicClass = 0.toByte()
    private var casicId    = 0.toByte()

    /**
     * Feed raw bytes from ae02 NOTIFY.
     * Routes to CASIC binary parser (0xBA 0xCE) or NMEA text parser ('$').
     */
    fun feedAe02Bytes(bytes: ByteArray) {
        for (b in bytes) {
            when {
                // Active CASIC parse in progress
                casicState != CasicState.IDLE -> processCasicByte(b)
                // Start of CASIC frame
                b == CASIC_SYNC1 -> processCasicByte(b)
                // NMEA text
                else -> nmeaBuffer.append(b.toInt().toChar())
            }
        }
        // Extract complete NMEA sentences
        while (nmeaBuffer.contains('$')) {
            val start = nmeaBuffer.indexOf('$')
            val end   = nmeaBuffer.indexOf('\n', start)
            if (start < 0 || end < 0) break
            val sentence = nmeaBuffer.substring(start, end).trim()
            nmeaBuffer.delete(0, end + 1)
            if (sentence.startsWith('$')) parseNmea(sentence)
            else onNmeaDebug?.invoke(sentence)
        }
        if (!nmeaBuffer.contains('$') && nmeaBuffer.length > 256) {
            onNmeaDebug?.invoke(bytes.joinToString(" ") { "%02X".format(it) })
            nmeaBuffer.clear()
        }
    }

    // ── CASIC state machine ───────────────────────────────────────────────────

    private fun processCasicByte(b: Byte) {
        when (casicState) {
            CasicState.IDLE   -> if (b == CASIC_SYNC1) casicState = CasicState.SYNC2
            CasicState.SYNC2  -> if (b == CASIC_SYNC2) { casicState = CasicState.LEN1; casicBuffer.clear() }
            else casicState = CasicState.IDLE
            CasicState.LEN1   -> { casicLen = b.toInt() and 0xFF; casicState = CasicState.LEN2 }
            CasicState.LEN2   -> { casicLen = casicLen or ((b.toInt() and 0xFF) shl 8); casicState = CasicState.CLASS }
            CasicState.CLASS  -> { casicClass = b; casicState = CasicState.ID }
            CasicState.ID     -> { casicId = b; casicState = CasicState.PAYLOAD }
            CasicState.PAYLOAD -> {
                casicBuffer.add(b)
                if (casicBuffer.size >= casicLen) casicState = CasicState.CK
            }
            CasicState.CK -> {
                // Checksum is 4 bytes — collect then verify
                casicBuffer.add(b)  // reuse buffer for checksum bytes
                val totalExpected = casicLen + 4
                if (casicBuffer.size >= totalExpected) {
                    val payload  = casicBuffer.subList(0, casicLen).toByteArray()
                    val ckBytes  = casicBuffer.subList(casicLen, casicLen + 4).toByteArray()
                    casicState   = CasicState.IDLE
                    if (verifyCasicChecksum(casicLen, casicClass, casicId, payload, ckBytes)) {
                        if (casicClass == CASIC_CLASS && casicId == CASIC_ID) parseCasicNav2Sol(payload)
                        else Log.d(TAG, "CASIC unhandled: class=0x%02X id=0x%02X".format(casicClass, casicId))
                    } else {
                        Log.d(TAG, "CASIC checksum fail")
                    }
                }
            }
        }
    }

    /**
     * CASIC checksum algorithm (from Python parser):
     *   ckSum = (id << 24) + (class << 16) + length
     *   for each 4-byte word in payload: ckSum += word  (32-bit wrap)
     */
    private fun verifyCasicChecksum(len: Int, cls: Byte, id: Byte,
                                    payload: ByteArray, ckBytes: ByteArray): Boolean {
        var ck = ((id.toInt() and 0xFF) shl 24) +
                ((cls.toInt() and 0xFF) shl 16) +
                (len and 0xFFFF)
        var i = 0
        while (i + 3 < payload.size) {
            val word = ((payload[i].toInt() and 0xFF)) or
                    ((payload[i+1].toInt() and 0xFF) shl 8) or
                    ((payload[i+2].toInt() and 0xFF) shl 16) or
                    ((payload[i+3].toInt() and 0xFF) shl 24)
            ck = (ck + word) and 0xFFFFFFFF.toInt()
            i += 4
        }
        val received = ((ckBytes[0].toInt() and 0xFF)) or
                ((ckBytes[1].toInt() and 0xFF) shl 8) or
                ((ckBytes[2].toInt() and 0xFF) shl 16) or
                ((ckBytes[3].toInt() and 0xFF) shl 24)
        return ck == received
    }

    /**
     * Parse CASIC NAV2-SOL payload (72 bytes).
     *
     * Layout (from Python parser + spec):
     *   0  I4  tow (ms)
     *   4  U2  wn
     *   6  U2  reserved
     *   8  U1  fixflags   bit0=fixOk  bits1-3=fixType(0=no,2=2D,3=3D,4=DR)
     *   9  U1  velflags
     *  10  U1  reserved
     *  11  U1  fixGnssMask
     *  12  8×U1 satellite counts
     *  20  U4  reserved
     *  24  R8  x (m) ECEF
     *  32  R8  y (m) ECEF
     *  40  R8  z (m) ECEF
     *  48  R4  pAcc (m)
     *  52  R4  vx (m/s) ECEF
     *  56  R4  vy (m/s) ECEF
     *  60  R4  vz (m/s) ECEF
     *  64  R4  sAcc (m/s)
     *  68  4   padding
     */
    private fun parseCasicNav2Sol(p: ByteArray) {
        if (p.size < CASIC_PAYLOAD_LEN) { Log.w(TAG, "NAV2-SOL short: ${p.size}"); return }

        val fixflags = p[8].toInt() and 0xFF
        val fixOk    = (fixflags and 0x01) != 0
        val fixType  = (fixflags shr 1) and 0x07   // bits 1-3

        // Position ECEF (R8 doubles, little-endian)
        val x = getR8(p, 24)
        val y = getR8(p, 32)
        val z = getR8(p, 40)

        // Velocity ECEF (R4 floats, little-endian)
        val vx = getR4(p, 52).toDouble()
        val vy = getR4(p, 56).toDouble()
        val vz = getR4(p, 60).toDouble()

        val (lat, lon, alt) = ecefToLla(x, y, z)
        val speedMs  = sqrt(vx*vx + vy*vy + vz*vz)
        val speedKt  = (speedMs * 1.94384).toFloat()

        // Heading from horizontal velocity (ENU decomposition)
        val (ve, vn) = ecefVelToEnu(vx, vy, vz, lat, lon)
        val hasHdg   = speedKt >= MIN_SPEED_FOR_HEADING_KT
        val heading  = if (hasHdg) ((Math.toDegrees(atan2(ve, vn)) + 360) % 360).toFloat()
        else currentData.headingDeg

        val hasFix = fixOk && fixType >= 2

        if (usePhoneGps && currentData.source == Source.PHONE && currentData.hasFix) return

        currentData = currentData.copy(
            source     = Source.BLE,
            speedKnots = speedKt,
            headingDeg = heading,
            hasHeading = hasHdg,
            hasFix     = hasFix,
            altitudeM  = alt.toFloat(),
            latDeg     = lat,
            lonDeg     = lon,
        )
        accumulateTrip(currentData)
        logData(currentData)
        Log.d(TAG, "CASIC NAV2-SOL: fix=$fixType ok=$fixOk spd=${speedKt}kt hdg=$heading")
        onUpdate?.invoke(currentData)
    }

    // ── NMEA parser ───────────────────────────────────────────────────────────

    private fun parseNmea(sentence: String) {
        if (!validateChecksum(sentence)) { Log.d(TAG, "NMEA CRC fail: $sentence"); return }
        val fields = sentence.substringBefore('*').split(',')
        val type   = fields.getOrNull(0)?.uppercase() ?: return
        when {
            type.endsWith("RMC") -> parseRmc(fields)
            type.endsWith("GGA") -> parseGga(fields)
        }
    }

    private fun parseRmc(f: List<String>) {
        val hasFix  = f.getOrNull(2) == "A"
        val speedKt = f.getOrNull(7)?.toFloatOrNull() ?: 0f
        val heading = f.getOrNull(8)?.toFloatOrNull()
        val lat     = parseLatLon(f.getOrNull(3), f.getOrNull(4))
        val lon     = parseLatLon(f.getOrNull(5), f.getOrNull(6))

        if (usePhoneGps && currentData.source == Source.PHONE && currentData.hasFix) return

        val hasHdg  = heading != null && speedKt >= MIN_SPEED_FOR_HEADING_KT
        currentData = currentData.copy(
            source     = Source.BLE,
            speedKnots = speedKt,
            headingDeg = if (hasHdg) heading!! else currentData.headingDeg,
            hasHeading = hasHdg,
            hasFix     = hasFix,
            latDeg     = lat ?: currentData.latDeg,
            lonDeg     = lon ?: currentData.lonDeg,
        )
        accumulateTrip(currentData)
        logData(currentData)
        onUpdate?.invoke(currentData)
    }

    private fun parseGga(f: List<String>) {
        val numSv = f.getOrNull(7)?.toIntOrNull() ?: 0
        val alt   = f.getOrNull(9)?.toFloatOrNull() ?: 0f
        val lat   = parseLatLon(f.getOrNull(2), f.getOrNull(3))
        val lon   = parseLatLon(f.getOrNull(4), f.getOrNull(5))
        currentData = currentData.copy(
            satellites = numSv,
            altitudeM  = alt,
            hasFix     = (f.getOrNull(6)?.toIntOrNull() ?: 0) > 0,
            latDeg     = lat ?: currentData.latDeg,
            lonDeg     = lon ?: currentData.lonDeg,
        )
        onUpdate?.invoke(currentData)
    }

    private fun parseLatLon(value: String?, hemisphere: String?): Double? {
        if (value.isNullOrEmpty()) return null
        return try {
            val dot = value.indexOf('.')
            val deg = value.substring(0, if (dot >= 4) dot - 2 else 2).toDouble()
            val min = value.substring(if (dot >= 4) dot - 2 else 2).toDouble()
            val d   = deg + min / 60.0
            if (hemisphere == "S" || hemisphere == "W") -d else d
        } catch (e: Exception) { null }
    }

    private fun validateChecksum(s: String): Boolean {
        val star = s.lastIndexOf('*')
        if (star < 1 || star + 2 >= s.length) return true
        return try {
            val expected = s.substring(star + 1, star + 3).toInt(16)
            val actual   = s.substring(1, star).fold(0) { acc, c -> acc xor c.code }
            expected == actual
        } catch (e: Exception) { false }
    }

    // ── Trip accumulation ─────────────────────────────────────────────────────

    private fun accumulateTrip(data: GpsData) {
        if (!data.hasFix) return
        if (data.speedKnots > maxSpeedKnots) maxSpeedKnots = data.speedKnots
        val lat = data.latDeg; val lon = data.lonDeg
        if (!lastFixLat.isNaN() && (lat != lastFixLat || lon != lastFixLon))
            tripDistanceNm += haversineNm(lastFixLat, lastFixLon, lat, lon)
        lastFixLat = lat; lastFixLon = lon
    }

    private fun haversineNm(lat1: Double, lon1: Double, lat2: Double, lon2: Double): Double {
        val R    = 3440.065
        val dLat = Math.toRadians(lat2 - lat1)
        val dLon = Math.toRadians(lon2 - lon1)
        val a    = sin(dLat/2).pow(2) +
                cos(Math.toRadians(lat1)) * cos(Math.toRadians(lat2)) * sin(dLon/2).pow(2)
        return R * 2 * asin(sqrt(a))
    }

    // ── ECEF coordinate helpers ───────────────────────────────────────────────

    /** ECEF (metres) → geodetic (lat/lon degrees, alt metres) — WGS84 iterative */
    private fun ecefToLla(x: Double, y: Double, z: Double): Triple<Double, Double, Double> {
        val a  = 6378137.0
        val e2 = 6.6943799901414e-3
        val lon = Math.toDegrees(atan2(y, x))
        var lat = atan2(z, sqrt(x * x + y * y))
        repeat(5) {
            val N = a / sqrt(1 - e2 * sin(lat).pow(2))
            lat   = atan2(z + e2 * N * sin(lat), sqrt(x * x + y * y))
        }
        val N   = a / sqrt(1 - e2 * sin(lat).pow(2))
        val alt = sqrt(x * x + y * y) / cos(lat) - N
        return Triple(Math.toDegrees(lat), lon, alt)
    }

    /** ECEF velocity (m/s) → East/North components at given geodetic position */
    private fun ecefVelToEnu(vx: Double, vy: Double, vz: Double,
                             latDeg: Double, lonDeg: Double): Pair<Double, Double> {
        val lat = Math.toRadians(latDeg)
        val lon = Math.toRadians(lonDeg)
        val ve  = -sin(lon) * vx + cos(lon) * vy
        val vn  = -sin(lat) * cos(lon) * vx - sin(lat) * sin(lon) * vy + cos(lat) * vz
        return Pair(ve, vn)
    }

    // ── Binary read helpers (little-endian) ───────────────────────────────────

    /** Read R4 (IEEE 754 float, little-endian) */
    private fun getR4(b: ByteArray, offset: Int): Float {
        val bits = ((b[offset].toInt()   and 0xFF)) or
                ((b[offset+1].toInt() and 0xFF) shl 8) or
                ((b[offset+2].toInt() and 0xFF) shl 16) or
                ((b[offset+3].toInt() and 0xFF) shl 24)
        return java.lang.Float.intBitsToFloat(bits)
    }

    /** Read R8 (IEEE 754 double, little-endian) */
    private fun getR8(b: ByteArray, offset: Int): Double {
        val lo = ((b[offset].toInt()   and 0xFF).toLong()) or
                ((b[offset+1].toInt() and 0xFF).toLong() shl 8) or
                ((b[offset+2].toInt() and 0xFF).toLong() shl 16) or
                ((b[offset+3].toInt() and 0xFF).toLong() shl 24)
        val hi = ((b[offset+4].toInt() and 0xFF).toLong()) or
                ((b[offset+5].toInt() and 0xFF).toLong() shl 8) or
                ((b[offset+6].toInt() and 0xFF).toLong() shl 16) or
                ((b[offset+7].toInt() and 0xFF).toLong() shl 24)
        return java.lang.Double.longBitsToDouble(lo or (hi shl 32))
    }
}