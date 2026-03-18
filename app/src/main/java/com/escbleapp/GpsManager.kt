package com.escbleapp

import android.Manifest
import android.content.Context
import android.content.pm.PackageManager
import android.location.Location
import android.os.Looper
import android.util.Log
import androidx.core.content.ContextCompat
import com.google.android.gms.location.*
import kotlin.math.*

/**
 * GpsManager — dual-source GPS for speed and heading.
 *
 * Source 1: Phone GPS via FusedLocationProviderClient
 * Source 2: BLE GPS module via NMEA sentences on ae02 NOTIFY
 *           Feed raw bytes via [feedNmea]. If data looks like NMEA ($GP/$GN)
 *           it is parsed here instead of being shown as debug hex.
 *
 * Delivers [GpsData] updates via [onUpdate] callback (main thread).
 *
 * NMEA sentences parsed:
 *   $GPRMC / $GNRMC — speed (knots), heading (true), lat/lon, fix status
 *   $GPGGA / $GNGGA — altitude, fix quality, satellite count
 */
class GpsManager(private val context: Context) {

    companion object {
        private const val TAG = "GpsManager"

        // Minimum speed to trust heading (below this heading is meaningless)
        private const val MIN_SPEED_FOR_HEADING_KT = 0.3f
    }

    // ── Data class ────────────────────────────────────────────────────────────

    enum class Source { NONE, PHONE, BLE }

    data class GpsData(
        val source:        Source  = Source.NONE,
        val speedKnots:    Float   = 0f,
        val headingDeg:    Float   = 0f,
        val hasHeading:    Boolean = false,
        val hasFix:        Boolean = false,
        val satellites:    Int     = 0,
        val altitudeM:     Float   = 0f,
        val latDeg:        Double  = 0.0,
        val lonDeg:        Double  = 0.0,
    ) {
        val speedKmh: Float  get() = speedKnots * 1.852f
        val speedMph: Float  get() = speedKnots * 1.15078f
        val headingCardinal: String get() = when {
            !hasHeading -> "—"
            headingDeg < 22.5  || headingDeg >= 337.5 -> "N"
            headingDeg < 67.5  -> "NE"
            headingDeg < 112.5 -> "E"
            headingDeg < 157.5 -> "SE"
            headingDeg < 202.5 -> "S"
            headingDeg < 247.5 -> "SW"
            headingDeg < 292.5 -> "W"
            else               -> "NW"
        }
    }

    // Trip stats — accumulated since last reset
    var tripDistanceNm: Double = 0.0
        private set
    var maxSpeedKnots: Float = 0f
        private set

    private var lastFixLat = Double.NaN
    private var lastFixLon = Double.NaN

    fun resetTrip() {
        tripDistanceNm = 0.0
        maxSpeedKnots  = 0f
        lastFixLat     = Double.NaN
        lastFixLon     = Double.NaN
    }

    // ── State ─────────────────────────────────────────────────────────────────

    var onUpdate:   ((GpsData) -> Unit)? = null
    var onNmeaDebug: ((String) -> Unit)? = null  // non-GPS ae02 bytes → forward to UI

    private var currentData = GpsData()
    private var usePhoneGps = true   // true = phone preferred, false = BLE preferred
    private var phoneGpsActive = false

    private val fusedClient: FusedLocationProviderClient =
        LocationServices.getFusedLocationProviderClient(context)

    private val locationRequest = LocationRequest.Builder(
        Priority.PRIORITY_HIGH_ACCURACY, 1_000L
    ).setMinUpdateIntervalMillis(500).build()

    private val locationCallback = object : LocationCallback() {
        override fun onLocationResult(result: LocationResult) {
            val loc = result.lastLocation ?: return
            // Only use phone GPS if we have no BLE fix or phone is preferred
            if (currentData.source != Source.BLE || usePhoneGps) {
                fromPhoneLocation(loc)
            }
        }
    }

    // ── NMEA buffer for multi-byte ae02 notifications ─────────────────────────
    private val nmeaBuffer = StringBuilder()

    // ── Public API ────────────────────────────────────────────────────────────

    fun startPhoneGps() {
        if (!hasLocationPermission()) {
            Log.w(TAG, "Location permission not granted")
            return
        }
        try {
            fusedClient.requestLocationUpdates(
                locationRequest, locationCallback, Looper.getMainLooper()
            )
            phoneGpsActive = true
            Log.i(TAG, "Phone GPS started")
        } catch (e: SecurityException) {
            Log.e(TAG, "GPS permission error: ${e.message}")
        }
    }

    fun stopPhoneGps() {
        fusedClient.removeLocationUpdates(locationCallback)
        phoneGpsActive = false
        Log.i(TAG, "Phone GPS stopped")
    }

    fun hasLocationPermission(): Boolean =
        ContextCompat.checkSelfPermission(context, Manifest.permission.ACCESS_FINE_LOCATION) ==
                PackageManager.PERMISSION_GRANTED

    /**
     * Feed raw bytes from ae02 NOTIFY.
     * If bytes look like NMEA → parse for GPS.
     * Otherwise → forward to onNmeaDebug as hex string.
     */
    fun feedAe02Bytes(bytes: ByteArray) {
        val str = String(bytes, Charsets.US_ASCII)

        // Quick NMEA prefix check
        if (!str.contains('$')) {
            // Not NMEA — pass through as debug
            onNmeaDebug?.invoke(bytes.joinToString(" ") { "%02X".format(it) })
            return
        }

        // Accumulate into buffer (ae02 may fragment sentences across notifications)
        nmeaBuffer.append(str)

        // Extract complete sentences
        while (true) {
            val start = nmeaBuffer.indexOf('$')
            val end   = nmeaBuffer.indexOf('\n', start)
            if (start < 0 || end < 0) break
            val sentence = nmeaBuffer.substring(start, end).trim()
            nmeaBuffer.delete(0, end + 1)
            parseNmea(sentence)
        }

        // Prevent unbounded growth if no newlines
        if (nmeaBuffer.length > 512) nmeaBuffer.clear()
    }

    fun setPreferBleGps()   { usePhoneGps = false }
    fun setPreferPhoneGps() { usePhoneGps = true }

    fun getCurrentData() = currentData

    // ── Phone GPS → GpsData ───────────────────────────────────────────────────

    private fun fromPhoneLocation(loc: Location) {
        val speedKt = (loc.speed * 1.94384f)  // m/s → knots
        val heading = if (loc.hasBearing() && speedKt >= MIN_SPEED_FOR_HEADING_KT)
            loc.bearing else currentData.headingDeg
        val hasHeading = loc.hasBearing() && speedKt >= MIN_SPEED_FOR_HEADING_KT

        currentData = GpsData(
            source      = Source.PHONE,
            speedKnots  = speedKt,
            headingDeg  = heading,
            hasHeading  = hasHeading,
            hasFix      = loc.accuracy < 50f,
            altitudeM   = loc.altitude.toFloat(),
            latDeg      = loc.latitude,
            lonDeg      = loc.longitude,
            satellites  = currentData.satellites
        )
        accumulateTrip(currentData)
        onUpdate?.invoke(currentData)
    }

    // ── NMEA parser ───────────────────────────────────────────────────────────

    private fun parseNmea(sentence: String) {
        if (!validateChecksum(sentence)) {
            Log.d(TAG, "NMEA checksum fail: $sentence")
            return
        }
        val fields = sentence.substringBefore('*').split(',')
        val type   = fields.getOrNull(0)?.uppercase() ?: return

        when {
            type.endsWith("RMC") -> parseRmc(fields)  // $GPRMC / $GNRMC
            type.endsWith("GGA") -> parseGga(fields)  // $GPGGA / $GNGGA
        }
    }

    /**
     * $GPRMC,hhmmss.ss,A,lat,N,lon,E,speed,heading,date,,,*checksum
     * Field indices:
     *   0=type 1=time 2=status(A=active) 3=lat 4=N/S 5=lon 6=E/W
     *   7=speed(knots) 8=heading(true) 9=date
     */
    private fun parseRmc(f: List<String>) {
        val status = f.getOrNull(2) ?: return
        val hasFix = status == "A"
        val speedKt = f.getOrNull(7)?.toFloatOrNull() ?: 0f
        val heading = f.getOrNull(8)?.toFloatOrNull()
        val lat = parseLatLon(f.getOrNull(3), f.getOrNull(4))
        val lon = parseLatLon(f.getOrNull(5), f.getOrNull(6))

        // Only override phone GPS if BLE is preferred or phone has no fix
        if (usePhoneGps && currentData.source == Source.PHONE && currentData.hasFix) return

        currentData = currentData.copy(
            source     = Source.BLE,
            speedKnots = speedKt,
            headingDeg = if (heading != null && speedKt >= MIN_SPEED_FOR_HEADING_KT) heading
            else currentData.headingDeg,
            hasHeading = heading != null && speedKt >= MIN_SPEED_FOR_HEADING_KT,
            hasFix     = hasFix,
            latDeg     = lat ?: currentData.latDeg,
            lonDeg     = lon ?: currentData.lonDeg,
        )
        accumulateTrip(currentData)
        Log.d(TAG, "BLE GPS RMC: ${speedKt}kt hdg=${heading} fix=$hasFix")
        onUpdate?.invoke(currentData)
    }

    // ── Trip accumulation ─────────────────────────────────────────────────────

    /**
     * Add distance from last fix to current fix (Haversine formula).
     * Update max speed.
     */
    private fun accumulateTrip(data: GpsData) {
        if (!data.hasFix) return

        // Max speed
        if (data.speedKnots > maxSpeedKnots) maxSpeedKnots = data.speedKnots

        // Distance — skip first fix or if coords unchanged
        val lat = data.latDeg; val lon = data.lonDeg
        if (!lastFixLat.isNaN() && (lat != lastFixLat || lon != lastFixLon)) {
            tripDistanceNm += haversineNm(lastFixLat, lastFixLon, lat, lon)
        }
        lastFixLat = lat; lastFixLon = lon
    }

    /** Haversine distance between two lat/lon points in nautical miles */
    private fun haversineNm(lat1: Double, lon1: Double, lat2: Double, lon2: Double): Double {
        val R = 3440.065  // Earth radius in nautical miles
        val dLat = Math.toRadians(lat2 - lat1)
        val dLon = Math.toRadians(lon2 - lon1)
        val a = sin(dLat / 2).pow(2) +
                cos(Math.toRadians(lat1)) * cos(Math.toRadians(lat2)) * sin(dLon / 2).pow(2)
        return R * 2 * asin(sqrt(a))
    }

    /**
     * $GPGGA,time,lat,N,lon,E,quality,numSV,hdop,alt,M,...
     * quality: 0=no fix 1=GPS 2=DGPS
     */
    private fun parseGga(f: List<String>) {
        val quality = f.getOrNull(6)?.toIntOrNull() ?: 0
        val numSv   = f.getOrNull(7)?.toIntOrNull() ?: 0
        val alt     = f.getOrNull(9)?.toFloatOrNull() ?: 0f
        val lat     = parseLatLon(f.getOrNull(2), f.getOrNull(3))
        val lon     = parseLatLon(f.getOrNull(4), f.getOrNull(5))

        currentData = currentData.copy(
            satellites = numSv,
            altitudeM  = alt,
            hasFix     = quality > 0,
            latDeg     = lat ?: currentData.latDeg,
            lonDeg     = lon ?: currentData.lonDeg,
        )
        onUpdate?.invoke(currentData)
    }

    /** Parse NMEA lat/lon: "ddmm.mmmm", "N"/"S"/"E"/"W" → decimal degrees */
    private fun parseLatLon(value: String?, hemisphere: String?): Double? {
        if (value.isNullOrEmpty()) return null
        return try {
            val dotIdx = value.indexOf('.')
            val degDigits = if (dotIdx >= 4) dotIdx - 2 else 2
            val degrees = value.substring(0, degDigits).toDouble()
            val minutes = value.substring(degDigits).toDouble()
            val decimal = degrees + minutes / 60.0
            if (hemisphere == "S" || hemisphere == "W") -decimal else decimal
        } catch (e: Exception) { null }
    }

    /** Validate NMEA XOR checksum: $...*XX */
    private fun validateChecksum(sentence: String): Boolean {
        val star = sentence.lastIndexOf('*')
        if (star < 1 || star + 2 >= sentence.length) return true  // no checksum → accept
        return try {
            val expected = sentence.substring(star + 1, star + 3).toInt(16)
            val actual   = sentence.substring(1, star).fold(0) { acc, c -> acc xor c.code }
            expected == actual
        } catch (e: Exception) { false }
    }
}