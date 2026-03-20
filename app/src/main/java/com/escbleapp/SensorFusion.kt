package com.escbleapp

import kotlin.math.*

/**
 * SensorFusion — portable heading/position fusion engine.
 *
 * NO Android imports. Pure Kotlin math only.
 * Can be copied to any project (autopilot firmware companion, desktop tool, etc.)
 *
 * Inputs  (via processXxx() methods):
 *   processA1()  — IMU + Mag raw at 50Hz  (QMI8658C + MMC5603)
 *   processA2()  — LC02H GNSS heading at 1Hz  (PQTMTAR + RMC)
 *   processA3()  — Position at 0.2Hz  (RMC/GGA lat/lon)
 *   processCasicNav2Sol() — raw ECEF position+velocity from CASIC receiver
 *   processNmeaRmc()      — standard NMEA speed/heading (phone GPS)
 *   processNmeaGga()      — standard NMEA fix quality/sats (phone GPS)
 *
 * Output (via [onFusedHeading] callback, called after every update):
 *   FusedState — heading, speed, position, confidence, source
 *
 * Tuning constants (adjust to match your hardware):
 *   GYRO_SCALE_DEG_S   — QMI8658C gyro LSB → °/s
 *   MAX_HEADING_RATE   — rate limiter in °/s (rejects spikes)
 */
class SensorFusion {

    // ── Tuning ────────────────────────────────────────────────────────────────

    /** QMI8658C gyro scale. ±256°/s range → 256/32768 = 0.0078. Adjust to firmware config. */
    var gyroScaleDegS: Float = 1f / 128f

    /** Maximum believable yaw rate in °/s — rejects vibration spikes */
    var maxHeadingRateDegS: Float = 60f

    // ── State ─────────────────────────────────────────────────────────────────

    data class FusedState(
        val headingDeg:       Float   = 0f,
        val speedKnots:       Float   = 0f,
        val latDeg:           Double  = 0.0,
        val lonDeg:           Double  = 0.0,
        val altM:             Float   = 0f,
        val hasHeading:       Boolean = false,
        val hasFix:           Boolean = false,
        val satellites:       Int     = 0,
        val headingConf:      Float   = 0f,   // 0.0 = unreliable … 1.0 = fully trusted
        val source:           String  = "none",
        val debugMsg:         String  = "",
    )

    private var state = FusedState()
    fun getState(): FusedState = state

    /** Called after every state update. Implementations should be non-blocking. */
    var onFusedHeading: ((FusedState) -> Unit)? = null

    // ── Complementary filter ──────────────────────────────────────────────────

    private var filteredHeading   = 0f
    private var filterInitialised = false
    private var lastImuTimeMs     = 0L
    private var lastGnssTimeMs    = 0L

    /**
     * Apply complementary filter for one step.
     *
     * @param sensorHeading  Absolute heading from a sensor (mag, GNSS, etc.) in degrees
     * @param gyroZDegS      Yaw rate from gyro in °/s (positive = clockwise)
     * @param sensorWeight   0.0 = pure gyro integration … 1.0 = pure sensor
     * @param dtS            Time step in seconds
     * @return               Filtered heading 0–360°
     */
    fun applyFilter(
        sensorHeading: Float,
        gyroZDegS:     Float,
        sensorWeight:  Float,
        dtS:           Float
    ): Float {
        if (!filterInitialised) {
            filteredHeading   = sensorHeading
            filterInitialised = true
            return sensorHeading
        }
        val gyroHeading = ((filteredHeading + gyroZDegS * dtS) + 360f) % 360f
        var diff = sensorHeading - gyroHeading
        while (diff >  180f) diff -= 360f
        while (diff < -180f) diff += 360f
        val maxStep    = maxHeadingRateDegS * dtS
        val correction = diff.coerceIn(-maxStep, maxStep)
        filteredHeading = ((gyroHeading + sensorWeight * correction) + 360f) % 360f
        return filteredHeading
    }

    /** Reset filter — call when source changes or after long gap */
    fun resetFilter() { filterInitialised = false }

    // ── 0xA1 — IMU + Mag (50 Hz) ─────────────────────────────────────────────

    /**
     * Process raw IMU + magnetometer packet.
     *
     * @param ax,ay,az  Accelerometer raw LSB (QMI8658C)
     * @param gx,gy,gz  Gyroscope raw LSB
     * @param mx,my,mz  Magnetometer raw LSB (MMC5603)
     * @param nowMs     Current time in milliseconds
     */
    fun processA1(
        ax: Short, ay: Short, az: Short,
        gx: Short, gy: Short, gz: Short,
        mx: Short, my: Short, mz: Short,
        nowMs: Long
    ) {
        val dtS = if (lastImuTimeMs > 0L)
            ((nowMs - lastImuTimeMs) / 1000f).coerceIn(0f, 0.1f)
        else 0.02f
        lastImuTimeMs = nowMs

        val gyroZDegS = gz * gyroScaleDegS

        // ── Tilt-compensated magnetometer heading ────────────────────────────
        // Compute roll and pitch from accelerometer
        val roll  = atan2(ay.toFloat(), az.toFloat())
        val pitch = atan2(-ax.toFloat(), ay * sin(roll) + az * cos(roll))

        // Project magnetometer onto horizontal plane
        val mxH = mx * cos(pitch) + mz * sin(pitch)
        val myH = mx * sin(roll) * sin(pitch) + my * cos(roll) - mz * sin(roll) * cos(pitch)
        val magHeading = ((Math.toDegrees(atan2(myH.toDouble(), mxH.toDouble()))
            .toFloat() + 360f) % 360f)

        // ── Tilt quality factor ───────────────────────────────────────────────
        // Large tilt = larger mag projection error → reduce mag influence
        val accelNorm  = sqrt((ax * ax + ay * ay + az * az).toFloat())
        val tiltDeg    = if (accelNorm > 0f)
            Math.toDegrees(acos((az / accelNorm).toDouble().coerceIn(-1.0, 1.0))).toFloat()
        else 90f
        // 0° → 1.0, 15° → 0.5, ≥30° → 0
        val tiltFactor = (1f - tiltDeg / 30f).coerceIn(0f, 1f)

        // ── Reduce mag weight when fresh GNSS heading is available ────────────
        val gnssRecent = (nowMs - lastGnssTimeMs) < 3_000L
        val gnssFactor = if (gnssRecent) (1f - state.headingConf * 0.8f) else 1f

        // Base mag weight 0.02 (gyro-dominant), scaled by quality
        val magWeight = (0.02f * tiltFactor * gnssFactor).coerceIn(0.002f, 0.04f)

        val fused = applyFilter(magHeading, gyroZDegS, magWeight, dtS)

        state = state.copy(
            headingDeg  = fused,
            hasHeading  = true,
            source      = "imu+mag",
            debugMsg    = "A1: gz=${"%.2f".format(gyroZDegS)} mag=${"%.1f".format(magHeading)} tilt=${"%.1f".format(tiltDeg)} w=${"%.4f".format(magWeight)} → ${"%.1f".format(fused)}"
        )
        onFusedHeading?.invoke(state)
    }

    // ── 0xA2 — LC02H GNSS heading (1 Hz) ─────────────────────────────────────

    /**
     * Process LC02H heading packet.
     *
     * @param tarHeadingDeg   PQTMTAR dual-antenna heading (degrees)
     * @param rmcHeadingDeg   RMC course-over-ground (degrees)
     * @param speedKt         RMC speed (knots)
     * @param tarAccDeg       PQTMTAR heading accuracy (degrees, lower = better)
     * @param gnssQuality     0=none, 4=RTK fixed, 6=DR
     * @param rmcValid        true if RMC heading is available and fresh
     * @param satellites      number of satellites used
     * @param nowMs           current time ms
     */
    fun processA2(
        tarHeadingDeg: Float,
        rmcHeadingDeg: Float,
        speedKt:       Float,
        tarAccDeg:     Int,
        gnssQuality:   Int,
        rmcValid:      Boolean,
        satellites:    Int,
        nowMs:         Long
    ) {
        // ── PQTMTAR weight ────────────────────────────────────────────────────
        val qualFactor = when (gnssQuality) { 4 -> 1.0f; 6 -> 0.5f; else -> 0.0f }
        // Accuracy: 0°=1.0, 20°=0.0
        val accFactor  = (1f - tarAccDeg / 20f).coerceIn(0f, 1f)
        // Satellites: <4=0, ≥8=1.0
        val satFactor  = ((satellites - 4f) / 4f).coerceIn(0f, 1f)
        // Speed: PQTMTAR drifts when nearly stationary
        // 40% weight retained below 0.3kt, full weight above 0.5kt
        val tarSpeedFactor = ((speedKt - 0.3f) / 0.2f).coerceIn(0.4f, 1f)

        var wTar = qualFactor * accFactor * satFactor * tarSpeedFactor

        // ── RMC COG weight ────────────────────────────────────────────────────
        // Zero below 0.5kt (COG is noise), full above 2kt, linear ramp between
        var wRmc = if (rmcValid) ((speedKt - 0.5f) / 1.5f).coerceIn(0f, 1f) else 0f

        val totalW = wTar + wRmc
        if (totalW <= 0f) return   // no valid heading source

        // Normalise
        wTar /= totalW
        wRmc /= totalW

        // Wrap-safe blend PQTMTAR + RMC
        var diff = rmcHeadingDeg - tarHeadingDeg
        while (diff >  180f) diff -= 360f
        while (diff < -180f) diff += 360f
        val blended = ((tarHeadingDeg + wRmc * diff) + 360f) % 360f

        // Overall confidence
        val conf = (qualFactor * accFactor * satFactor * 0.8f +
                    wRmc * (speedKt / 2f).coerceIn(0f, 1f) * 0.2f).coerceIn(0f, 1f)

        // Filter weight — how aggressively this 1Hz GNSS corrects the running estimate
        // High confidence → stronger correction; low → let gyro hold
        val filterW = (0.05f + conf * 0.15f).coerceIn(0.05f, 0.20f)

        lastGnssTimeMs = nowMs
        val fused = applyFilter(blended, 0f, filterW, 0.1f)  // ~1Hz = dt≈1s handled by filter

        state = state.copy(
            headingDeg  = fused,
            speedKnots  = speedKt,
            hasHeading  = true,
            hasFix      = gnssQuality >= 4,
            satellites  = satellites,
            headingConf = conf,
            source      = "gnss+imu",
            debugMsg    = "A2: tar=${"%.1f".format(tarHeadingDeg)}(w=${"%.2f".format(wTar)}) rmc=${"%.1f".format(rmcHeadingDeg)}(w=${"%.2f".format(wRmc)}) blend=${"%.1f".format(blended)} → ${"%.1f".format(fused)} conf=${"%.2f".format(conf)} sats=$satellites acc=$tarAccDeg spd=$speedKt"
        )
        onFusedHeading?.invoke(state)
    }

    // ── 0xA3 — Position (0.2 Hz) ─────────────────────────────────────────────

    /**
     * Process position-only packet.
     * Updates lat/lon/alt for waypoint bearing and trip distance.
     */
    fun processA3(
        latDeg:   Double,
        lonDeg:   Double,
        speedKt:  Float,
        satellites: Int,
        fixQuality: Int
    ) {
        state = state.copy(
            latDeg     = latDeg,
            lonDeg     = lonDeg,
            speedKnots = speedKt,
            hasFix     = fixQuality >= 1,
            satellites = satellites,
            source     = state.source,
            debugMsg   = "A3: lat=${"%.6f".format(latDeg)} lon=${"%.6f".format(lonDeg)} spd=$speedKt fix=$fixQuality"
        )
        onFusedHeading?.invoke(state)
    }

    // ── CASIC NAV2-SOL — ECEF position + velocity ─────────────────────────────

    /**
     * Process CASIC NAV2-SOL ECEF data.
     * Derives lat/lon, speed, and velocity-based heading.
     *
     * @param ecefX,Y,Z  Position in metres (ECEF)
     * @param ecefVX,VY,VZ  Velocity in m/s (ECEF)
     * @param sAccMs     Speed accuracy estimate in m/s
     * @param fixOk      Fix valid flag
     * @param fixType    0=none, 2=2D, 3=3D, 4=DR
     */
    fun processCasicNav2Sol(
        ecefX: Double, ecefY: Double, ecefZ: Double,
        ecefVX: Double, ecefVY: Double, ecefVZ: Double,
        sAccMs: Float,
        fixOk: Boolean,
        fixType: Int
    ) {
        val (lat, lon, alt) = ecefToLla(ecefX, ecefY, ecefZ)
        val speedMs  = sqrt(ecefVX * ecefVX + ecefVY * ecefVY + ecefVZ * ecefVZ)
        val speedKt  = (speedMs * 1.94384).toFloat()
        val (ve, vn) = ecefVelToEnu(ecefVX, ecefVY, ecefVZ, lat, lon)
        val conf     = if (sAccMs > 0f) (speedMs / sAccMs).toFloat().coerceIn(0f, 1f) else 0f
        val hasHdg   = speedKt >= 0.3f && conf > 0.1f
        val heading  = if (hasHdg)
            ((Math.toDegrees(atan2(ve, vn)) + 360) % 360).toFloat()
        else state.headingDeg

        val hasFix = fixOk && fixType >= 2
        if (hasHdg) {
            val fused = applyFilter(heading, 0f, 0.15f, 0.1f)
            state = state.copy(
                headingDeg  = fused,
                speedKnots  = speedKt,
                hasHeading  = true,
                hasFix      = hasFix,
                latDeg      = lat,
                lonDeg      = lon,
                altM        = alt.toFloat(),
                headingConf = conf,
                source      = "casic",
                debugMsg    = "CASIC: hdg=${"%.1f".format(fused)} spd=$speedKt conf=${"%.2f".format(conf)}"
            )
        } else {
            state = state.copy(
                hasFix = hasFix, latDeg = lat, lonDeg = lon,
                altM = alt.toFloat(), speedKnots = speedKt
            )
        }
        onFusedHeading?.invoke(state)
    }

    // ── NMEA RMC ──────────────────────────────────────────────────────────────

    /**
     * Process NMEA RMC data (from phone GPS or any NMEA source).
     */
    fun processNmeaRmc(
        speedKt:   Float,
        cogDeg:    Float?,   // null if no COG available
        hasFix:    Boolean,
        latDeg:    Double?,
        lonDeg:    Double?
    ) {
        val hasHdg = cogDeg != null && speedKt >= 0.3f
        val heading = if (hasHdg) {
            applyFilter(cogDeg!!, 0f, 0.15f, 0.1f)
        } else state.headingDeg

        state = state.copy(
            headingDeg  = heading,
            speedKnots  = speedKt,
            hasHeading  = hasHdg,
            hasFix      = hasFix,
            latDeg      = latDeg ?: state.latDeg,
            lonDeg      = lonDeg ?: state.lonDeg,
            source      = "nmea",
        )
        onFusedHeading?.invoke(state)
    }

    // ── NMEA GGA ──────────────────────────────────────────────────────────────

    fun processNmeaGga(
        satellites: Int,
        altM:       Float,
        fixQuality: Int,
        latDeg:     Double?,
        lonDeg:     Double?
    ) {
        state = state.copy(
            satellites = satellites,
            altM       = altM,
            hasFix     = fixQuality > 0,
            latDeg     = latDeg ?: state.latDeg,
            lonDeg     = lonDeg ?: state.lonDeg,
        )
        onFusedHeading?.invoke(state)
    }

    // ── Geo helpers ───────────────────────────────────────────────────────────

    /** ECEF (m) → geodetic (lat°, lon°, alt m) WGS84 iterative */
    fun ecefToLla(x: Double, y: Double, z: Double): Triple<Double, Double, Double> {
        val a  = 6378137.0; val e2 = 6.6943799901414e-3
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

    /** ECEF velocity → East/North at given geodetic position */
    fun ecefVelToEnu(vx: Double, vy: Double, vz: Double,
                     latDeg: Double, lonDeg: Double): Pair<Double, Double> {
        val lat = Math.toRadians(latDeg); val lon = Math.toRadians(lonDeg)
        val ve  = -sin(lon) * vx + cos(lon) * vy
        val vn  = -sin(lat) * cos(lon) * vx - sin(lat) * sin(lon) * vy + cos(lat) * vz
        return Pair(ve, vn)
    }

    /** Haversine distance in nautical miles */
    fun haversineNm(lat1: Double, lon1: Double, lat2: Double, lon2: Double): Double {
        val R = 3440.065
        val dLat = Math.toRadians(lat2 - lat1); val dLon = Math.toRadians(lon2 - lon1)
        val a = sin(dLat / 2).pow(2) +
                cos(Math.toRadians(lat1)) * cos(Math.toRadians(lat2)) * sin(dLon / 2).pow(2)
        return R * 2 * asin(sqrt(a))
    }

    /** Forward bearing from point 1 to point 2, degrees 0–360 */
    fun bearingTo(lat1: Double, lon1: Double, lat2: Double, lon2: Double): Float {
        val φ1 = Math.toRadians(lat1); val φ2 = Math.toRadians(lat2)
        val Δλ = Math.toRadians(lon2 - lon1)
        val y  = sin(Δλ) * cos(φ2)
        val x  = cos(φ1) * sin(φ2) - sin(φ1) * cos(φ2) * cos(Δλ)
        return ((Math.toDegrees(atan2(y, x)) + 360) % 360).toFloat()
    }

    /** Heading error wrapped to ±180° */
    fun headingError(target: Float, actual: Float): Float {
        var err = target - actual
        while (err >  180f) err -= 360f
        while (err < -180f) err += 360f
        return err
    }
}
