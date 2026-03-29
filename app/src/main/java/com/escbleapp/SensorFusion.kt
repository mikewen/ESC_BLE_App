package com.escbleapp

import android.util.Log
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

    /** QMI8658C gyro scale. ± 128°/s range → 128/32768 . Adjust to firmware config. */
    var gyroScaleDegS: Float = 1f / 256f

    /** Set true if turning right decreases heading (gz axis inverted on your PCB). */
    var gyroZFlipped: Boolean = true   // flipped by default for this hardware setup

    /** Maximum believable yaw rate in °/s — rejects vibration spikes */
    var maxHeadingRateDegS: Float = 60f

    /** Base deadband in degrees (calm sea) */
    var baseDeadbandDeg: Float = 3f

    /** Additional deadband per unit of sea state (0–1) */
    var seaDeadbandScale: Float = 12f

    private var lastGyroZDegS: Float = 0f

    // ── State ─────────────────────────────────────────────────────────────────

    data class FusedState(
        val headingDeg:           Float   = 0f,
        val speedKnots:           Float   = 0f,
        val latDeg:               Double  = 0.0,
        val lonDeg:               Double  = 0.0,
        val altM:                 Float   = 0f,
        val hasHeading:           Boolean = false,
        val hasFix:               Boolean = false,
        val satellites:           Int     = 0,
        val headingConf:          Float   = 0f,    // 0.0 = unreliable … 1.0 = fully trusted
        val seaState:             Float   = 0f,    // 0 = calm … 1 = rough
        val autoDeadbandDeg:      Float   = 2f,    // deadband adjusted for sea state
        val magCalibrated:        Boolean = false, // GPS auto-cal of MMC5603 complete
        val rawMagHeadingDeg:     Float   = 0f,    // Raw tilt-compensated magnetometer heading
        val tarMisalignDeg:       Float   = 0f,    // LC02H mounting offset vs COG (degrees)
        val tarMisalignCalibrated: Boolean = false, // mounting offset auto-detected
        val source:               String  = "none",
        val debugMsg:             String  = "",
    )

    private var state = FusedState()
    fun getState(): FusedState = state

    /** Called after every state update. Implementations should be non-blocking. */
    var onFusedHeading: ((FusedState) -> Unit)? = null

    // ── #1 Sea state estimation ────────────────────────────────────────────────
    // Combines vertical accel variance (heave) + gyro pitch/roll rate variance (angular motion).
    // More sensitive than accel alone — gyro responds immediately to wave-induced rotation.

    private val SEA_STATE_WINDOW = 100       // samples at 50Hz = 2 seconds

    /** Accel Z stddev (LSB) for calm sea. Raise to reduce sensitivity. Default 200. */
    var seaAzCalm:   Float = 200f
    /** Accel Z stddev (LSB) for rough sea. Default 2000. */
    var seaAzRough:  Float = 2000f
    /** Gyro XY RMS stddev (LSB) for calm sea. Raise to reduce sensitivity. Default 100. */
    var seaGxyCalm:  Float = 100f
    /** Gyro XY RMS stddev (LSB) for rough sea. Default 800. */
    var seaGxyRough: Float = 800f

    private val azWindow    = FloatArray(SEA_STATE_WINDOW)
    private val gxyWindow   = FloatArray(SEA_STATE_WINDOW)  // combined gx+gy energy
    private var seaWinIdx   = 0
    private var seaWinFull  = false
    private var azLpf       = 0f
    private var gxLpf       = 0f
    private var gyLpf       = 0f

    /**
     * Update sea state from accel Z (heave) and gyro X/Y (pitch/roll rate).
     * Returns 0=calm … 1=rough.
     */
    private fun updateSeaState(az: Short, gx: Short, gy: Short): Float {
        // High-pass filter each axis — remove DC (gravity / steady rotation)
        val azF = az.toFloat(); azLpf = azLpf * 0.99f + azF * 0.01f
        val gxF = gx.toFloat(); gxLpf = gxLpf * 0.99f + gxF * 0.01f
        val gyF = gy.toFloat(); gyLpf = gyLpf * 0.99f + gyF * 0.01f

        azWindow[seaWinIdx]  = azF - azLpf
        // Combine gx and gy as RMS energy — either axis rolling/pitching counts
        val gxyHp = sqrt((gxF - gxLpf) * (gxF - gxLpf) + (gyF - gyLpf) * (gyF - gyLpf))
        gxyWindow[seaWinIdx] = gxyHp

        seaWinIdx = (seaWinIdx + 1) % SEA_STATE_WINDOW
        if (seaWinIdx == 0) seaWinFull = true

        val n = if (seaWinFull) SEA_STATE_WINDOW else seaWinIdx
        if (n < 10) return state.seaState

        // Stddev of each channel
        fun stddev(arr: FloatArray): Float {
            val mean = arr.take(n).sum() / n
            val v = arr.take(n).sumOf { ((it - mean) * (it - mean)).toDouble() }.toFloat() / n
            return sqrt(v)
        }
        val azStd  = stddev(azWindow)
        val gxyStd = stddev(gxyWindow)

        // Normalise each to 0–1 then take max — either accel or gyro can detect rough sea
        val azSea  = ((azStd  - seaAzCalm)  / (seaAzRough  - seaAzCalm )).coerceIn(0f, 1f)
        val gxySea = ((gxyStd - seaGxyCalm) / (seaGxyRough - seaGxyCalm)).coerceIn(0f, 1f)
        return maxOf(azSea, gxySea)
    }

    // ── #3 Automatic deadband ─────────────────────────────────────────────────

    /** Compute deadband from current sea state. Call after updateSeaState(). */
    fun computeAutoDeadband(seaState: Float): Float =
        baseDeadbandDeg + seaState * seaDeadbandScale

    // ── #2 GPS auto-calibration of MMC5603 ────────────────────────────────────
    // When speed>2kt + calm sea + high GPS conf: accumulate (GPS_hdg - mag_hdg) bias.
    // Once variance is stable → mark calibrated, increase mag weight.

    private val MAG_CAL_SPEED_KT   = 2.0f
    private val MAG_CAL_SEA_STATE  = 0.15f   // only calibrate in calm conditions
    private val MAG_CAL_GPS_CONF   = 0.75f   // require reliable GPS
    private val MAG_CAL_WINDOW     = 30       // samples (~30 GNSS seconds)
    private val MAG_CAL_STABLE_DEG = 5f       // bias stddev must be < this to accept

    private val magBiasWindow = FloatArray(MAG_CAL_WINDOW)
    private var magBiasIdx    = 0
    private var magBiasFull   = false
    var magBiasEstimate: Float = 0f   // persistent bias offset (degrees), apply in processA1
        private set
    var magCalibrated: Boolean = false
        private set

    /**
     * Feed one mag-vs-GPS comparison sample.
     * Called from processA2 when conditions are met.
     * Returns updated bias if newly calibrated, or previous bias.
     */
    private fun updateMagCalibration(
        gpsHeading: Float,
        rawMagHeading: Float,
        speedKt: Float,
        seaState: Float,
        gpsConf: Float
    ) {
        if (speedKt < MAG_CAL_SPEED_KT || seaState > MAG_CAL_SEA_STATE || gpsConf < MAG_CAL_GPS_CONF)
            return

        // Wrap-safe bias
        var bias = gpsHeading - rawMagHeading
        while (bias >  180f) bias -= 360f
        while (bias < -180f) bias += 360f

        magBiasWindow[magBiasIdx] = bias
        magBiasIdx = (magBiasIdx + 1) % MAG_CAL_WINDOW
        if (magBiasIdx == 0) magBiasFull = true

        val n = if (magBiasFull) MAG_CAL_WINDOW else magBiasIdx
        if (n < MAG_CAL_WINDOW) return   // not enough samples

        val mean = magBiasWindow.sum() / n
        val variance = magBiasWindow.sumOf { ((it - mean) * (it - mean)).toDouble() }.toFloat() / n
        val stddev = sqrt(variance)

        if (stddev < MAG_CAL_STABLE_DEG) {
            magBiasEstimate = mean
            magCalibrated   = true
        }
    }

    // ── Manual calibration support ─────────────────────────────────────────────
    // Collect raw mx/my samples during 360° rotation for hard-iron calibration.

    data class MagCalPoint(val mx: Float, val my: Float)
    private val manualCalPoints = mutableListOf<MagCalPoint>()
    var isManualCalActive = false
        private set
    var manualCalHardIronX = 0f; var manualCalHardIronY = 0f   // offsets to apply

    fun startManualMagCal() { manualCalPoints.clear(); isManualCalActive = true }

    fun feedManualMagSample(mx: Short, my: Short) {
        if (isManualCalActive) manualCalPoints.add(MagCalPoint(mx.toFloat(), my.toFloat()))
    }

    /** Compute hard-iron offsets from collected samples. Returns true if enough data. */
    fun finishManualMagCal(): Boolean {
        isManualCalActive = false
        if (manualCalPoints.size < 36) return false   // need at least 36 points (~10° spacing)
        manualCalHardIronX = (manualCalPoints.maxOf { it.mx } + manualCalPoints.minOf { it.mx }) / 2f
        manualCalHardIronY = (manualCalPoints.maxOf { it.my } + manualCalPoints.minOf { it.my }) / 2f
        return true
    }

    // Gyro bias calibration (measured on level surface, stationary)
    var gyroBiasX = 0f; var gyroBiasY = 0f; var gyroBiasZ = 0f
    private val gyroBiasSamples = mutableListOf<Triple<Float,Float,Float>>()
    var isGyroBiasCalActive = false
        private set

    fun startGyroBiasCal() { gyroBiasSamples.clear(); isGyroBiasCalActive = true }

    fun feedGyroBiasSample(gx: Short, gy: Short, gz: Short) {
        if (isGyroBiasCalActive)
            gyroBiasSamples.add(Triple(gx.toFloat(), gy.toFloat(), gz.toFloat()))
    }

    fun finishGyroBiasCal(): Boolean {
        isGyroBiasCalActive = false
        if (gyroBiasSamples.size < 100) return false
        gyroBiasX = gyroBiasSamples.map { it.first  }.average().toFloat()
        gyroBiasY = gyroBiasSamples.map { it.second }.average().toFloat()
        gyroBiasZ = gyroBiasSamples.map { it.third  }.average().toFloat()
        return true
    }

    // ── LC02H Install Misalignment Auto-Calibration ───────────────────────────
    // When speed > 4kt AND calm sea: vessel tracks straight, so RMC COG is the
    // true vessel heading. Any stable offset between PQTMTAR and COG = mounting error.
    // Guard: only accept if |mean| < 15° (gross misinstall needs physical correction).

    private val TAR_MISALIGN_SPEED_KT  = 4.0f
    private val TAR_MISALIGN_SEA_STATE = 0.15f   // calm only
    private val TAR_MISALIGN_WINDOW    = 30       // samples to accumulate
    private val TAR_MISALIGN_STABLE    = 3.0f     // stddev threshold in degrees
    private val TAR_MISALIGN_MAX       = 15.0f    // reject large offsets (gross misinstall)

    private val misalignWindow = FloatArray(TAR_MISALIGN_WINDOW)
    private var misalignIdx    = 0
    private var misalignFull   = false

    var tarMisalignEstimate:   Float   = 0f    // mounting offset (degrees), applied in processA2
        private set
    var tarMisalignCalibrated: Boolean = false
        private set

    private fun updateTarMisalignment(cogDeg: Float, tarDeg: Float,
                                      speedKt: Float, seaState: Float) {
        if (speedKt < TAR_MISALIGN_SPEED_KT || seaState > TAR_MISALIGN_SEA_STATE) return

        // Wrap-safe bias: COG − PQTMTAR
        var bias = cogDeg - tarDeg
        while (bias >  180f) bias -= 360f
        while (bias < -180f) bias += 360f

        misalignWindow[misalignIdx] = bias
        misalignIdx = (misalignIdx + 1) % TAR_MISALIGN_WINDOW
        if (misalignIdx == 0) misalignFull = true

        val n = if (misalignFull) TAR_MISALIGN_WINDOW else misalignIdx
        if (n < TAR_MISALIGN_WINDOW) return   // not enough samples yet

        val mean     = misalignWindow.take(n).sum() / n
        val variance = misalignWindow.take(n).sumOf { ((it - mean) * (it - mean)).toDouble() }.toFloat() / n
        val stddev   = sqrt(variance)

        // Accept only if stable AND small — large = wrong physical orientation
        if (stddev < TAR_MISALIGN_STABLE && abs(mean) < TAR_MISALIGN_MAX) {
            tarMisalignEstimate   = mean
            tarMisalignCalibrated = true
            Log.i("SensorFusion", "LC02H misalign calibrated: ${"%.2f".format(mean)}° stddev=${"%.2f".format(stddev)}°")
        }
    }

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
        //val correction = diff.coerceIn(-maxStep, maxStep)
        val damping = 0.3f * gyroZDegS  // add derivative damping
        val correction = (diff - damping).coerceIn(-maxStep, maxStep)
        filteredHeading = ((gyroHeading + sensorWeight * correction) + 360f) % 360f
        return filteredHeading
    }

    /** Reset filter — call when source changes or after long gap */
    fun resetFilter() { filterInitialised = false }

    // ── 0xA1 — IMU + Mag (50 Hz) ─────────────────────────────────────────────

    /**
     * Process raw IMU + magnetometer packet (0xA1, 50Hz).
     *
     * QMI8658C accel (ax,ay,az) + gyro (gx,gy,gz):
     *   - gz → yaw rate → complementary filter gyro integration
     *   - gx,gy + az → sea state estimation (wave heave + angular motion)
     *   - ax,ay,az → tilt angles (roll/pitch) for magnetometer compensation
     *
     * MMC5603 mag (mx,my,mz):
     *   - Tilt-compensated using accel roll/pitch → magnetic heading
     *   - Used as absolute heading reference in complementary filter
     *   - Prevents gyro drift over time
     *   - Weight is small (0.02–0.05) — gyro dominates short-term, mag corrects long-term
     *   - Hard-iron offsets applied from dock calibration
     *   - GPS bias applied when GPS auto-cal has run at sea
     *
     * @param accelRotated180  true if QMI8658C is mounted 180° from MMC5603 on your PCB
     */
    fun processA1(
        ax: Short, ay: Short, az: Short,
        gx: Short, gy: Short, gz: Short,
        mx: Short, my: Short, mz: Short,
        nowMs: Long,
        accelRotated180: Boolean = false
    ) {
        val dtS = if (lastImuTimeMs > 0L)
            ((nowMs - lastImuTimeMs) / 1000f).coerceIn(0f, 0.1f)
        else 0.02f
        lastImuTimeMs = nowMs

        // Apply gyro bias correction (from dock calibration)
        val gzCorrected = gz - gyroBiasZ
        // Negate gz if gyro Z axis is mounted inverted (turning right should increase heading)
        val gyroZDegS   = gzCorrected * gyroScaleDegS * (if (gyroZFlipped) -1f else 1f)
        lastGyroZDegS = gyroZDegS

        // ── #1 Sea state: accel Z heave + gyro X/Y angular rate ──────────────
        val seaState     = updateSeaState(az, gx, gy)
        val autoDeadband = computeAutoDeadband(seaState)

        // ── Apply accel 180° rotation if sensors are misaligned on PCB ───────
        // If QMI8658C ax/ay are 180° from MMC5603 frame: flip ax and ay
        // so tilt compensation uses the same coordinate frame as the mag.
        val axEff = if (accelRotated180) (-ax.toInt()).toShort() else ax
        val ayEff = if (accelRotated180) (-ay.toInt()).toShort() else ay

        // ── MMC5603 tilt-compensated magnetic heading ─────────────────────────
        // Hard-iron offset from dock calibration
        val mxCal = (mx - manualCalHardIronX).toInt().toShort()
        val myCal = (my - manualCalHardIronY).toInt().toShort()

        // Compute roll and pitch from (possibly rotated) accelerometer
        val roll  = atan2(ayEff.toFloat(), az.toFloat())
        val pitch = atan2(-axEff.toFloat(), ayEff * sin(roll) + az * cos(roll))

        // Project magnetometer onto horizontal plane (tilt compensation)
        val mxH = mxCal * cos(pitch) + mz * sin(pitch)
        val myH = mxCal * sin(roll) * sin(pitch) + myCal * cos(roll) - mz * sin(roll) * cos(pitch)
        val rawMagHeading = ((Math.toDegrees(atan2(myH.toDouble(), mxH.toDouble()))
            .toFloat() + 360f) % 360f)

        // Apply GPS auto-calibration bias (learned at sea, speed>2kt, calm)
        val magHeading = if (magCalibrated)
            ((rawMagHeading + magBiasEstimate) + 360f) % 360f
        else rawMagHeading

        // ── Tilt quality factor ───────────────────────────────────────────────
        val accelNorm  = sqrt((axEff * axEff + ayEff * ayEff + az * az).toFloat())
        val tiltDeg    = if (accelNorm > 0f)
            Math.toDegrees(acos((az / accelNorm).toDouble().coerceIn(-1.0, 1.0))).toFloat()
        else 90f
        val tiltFactor = (1f - tiltDeg / 30f).coerceIn(0f, 1f)

        // ── Mag weight ────────────────────────────────────────────────────────
        val gnssRecent = (nowMs - lastGnssTimeMs) < 3_000L
        val gnssFactor = if (gnssRecent) (1f - state.headingConf * 0.8f) else 1f
        // Calibrated mag gets higher base weight (0.05 vs 0.02)
        val magBase   = if (magCalibrated) 0.12f else 0.02f

        //val turnRate = abs(gyroZDegS)   // reduce mag correction during rotation
        //val turnFactor = (1f - turnRate / 30f).coerceIn(0.2f, 1f)
        //val magWeight = (magBase * tiltFactor * gnssFactor * turnFactor)
        // ── NEW: Speed Factor to reduce Mag weight when GPS COG is reliable ──
        val speedKt = state.speedKnots
        val speedFactor = when {
            speedKt < 3.0f -> 1.0f                  // Below 3kt: Normal Mag weight
            speedKt > 5.0f -> 0.1f                  // Above 5kt: Minimal Mag weight (Trust Gyro+GPS)
            else -> 1.0f - ((speedKt - 3.0f) / 2.0f) * 0.9f // Ramp down between 3kt and 5kt
        }
        //val magWeight = (magBase * tiltFactor * gnssFactor).coerceIn(0.02f, 0.35f)
        // Apply speed factor to final weight
        val magWeight = (magBase * tiltFactor * gnssFactor * speedFactor)
            .coerceIn(0.01f, 0.35f) // Lower min bound to allow very low mag influence

        val fused = applyFilter(magHeading, gyroZDegS, magWeight, dtS)

        state = state.copy(
            headingDeg      = fused,
            hasHeading      = true,
            seaState        = seaState,
            autoDeadbandDeg = autoDeadband,
            magCalibrated   = magCalibrated,
            rawMagHeadingDeg = rawMagHeading,
            source          = "imu+mag",
            debugMsg        = "A1: gz=${"%.2f".format(gyroZDegS)} mag=${"%.1f".format(magHeading)} tilt=${"%.1f".format(tiltDeg)} sea=${"%.2f".format(seaState)} db=${"%.1f".format(autoDeadband)}° w=${"%.4f".format(magWeight)} → ${"%.1f".format(fused)}"
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
    // ── 0xA2 — LC02H GNSS heading (1 Hz) ─────────────────────────────────────

    // Cache latest RMC values from A3 packet for blending with A2
    private var cachedRmcHeading = 0f
    private var cachedRmcSpeed   = 0f
    private var cachedRmcValid   = false

    /**
     * Process PQTMTAR packet (0xA2).
     * RMC course and speed come from the cached A3 packet.
     *
     * @param tarHeadingDeg  PQTMTAR dual-antenna heading (degrees, 0–360)
     * @param pitchDeg       PQTMTAR pitch angle
     * @param rollDeg        PQTMTAR roll angle
     * @param tarAccDeg      Heading accuracy in degrees (float, e.g. 0.5°)
     * @param gnssQuality    0=none, 4=RTK fixed, 6=DR
     * @param satellites     Number of SVs used
     * @param nowMs          Current time ms
     */
    fun processA2(
        tarHeadingDeg: Float,
        pitchDeg:      Float,
        rollDeg:       Float,
        tarAccDeg:     Float,
        gnssQuality:   Int,
        satellites:    Int,
        nowMs:         Long
    ) {
        val speedKt = cachedRmcSpeed

        // Apply LC02H mounting misalignment correction if calibrated
        // e.g. antenna mounted 3° left → tarMisalignEstimate = +3° → correct right
        val correctedTarHdg = if (tarMisalignCalibrated)
            ((tarHeadingDeg + tarMisalignEstimate) + 360f) % 360f
        else tarHeadingDeg

        // ── PQTMTAR weight ────────────────────────────────────────────────────
        val qualFactor = when (gnssQuality) { 4 -> 1.0f; 6 -> 0.5f; else -> 0.0f }
        // Accuracy: 0°=1.0, 20°=0.0  (tarAccDeg is float, e.g. 0.5°)
        val accFactor  = (1f - tarAccDeg / 20f).coerceIn(0f, 1f)
        // Satellites: <4=0, ≥8=1.0
        val satFactor  = ((satellites - 4f) / 4f).coerceIn(0f, 1f)
        // Speed: PQTMTAR drifts when nearly stationary — retain 40% weight below 0.3kt
        val tarSpeedFactor = ((speedKt - 0.3f) / 0.2f).coerceIn(0.4f, 1f)

        var wTar = qualFactor * accFactor * satFactor * tarSpeedFactor

        // ── RMC COG weight (from cached A3) ─────────────────────────────────
        // Zero below 0.5kt, full above 2kt, linear ramp between
        var wRmc = if (cachedRmcValid) ((speedKt - 0.5f) / 1.5f).coerceIn(0f, 1f) else 0f

        val totalW = wTar + wRmc
        if (totalW <= 0f) {
            Log.d("SensorFusion", "A2: no valid heading (qual=$gnssQuality acc=$tarAccDeg sats=$satellites)")
            return
        }

        wTar /= totalW
        wRmc /= totalW

        // Wrap-safe blend correctedTarHdg + RMC COG
        var diff = cachedRmcHeading - correctedTarHdg
        while (diff >  180f) diff -= 360f
        while (diff < -180f) diff += 360f
        val blended = ((correctedTarHdg + wRmc * diff) + 360f) % 360f

        val conf = (qualFactor * accFactor * satFactor * 0.8f +
                wRmc * (speedKt / 2f).coerceIn(0f, 1f) * 0.2f).coerceIn(0f, 1f)
        val filterW = (0.05f + conf * 0.15f).coerceIn(0.05f, 0.20f)

        lastGnssTimeMs = nowMs
        // Estimate turn rate from last IMU update (deg/s)
        //val turnRate = abs(state.debugMsg.substringAfter("gz=").substringBefore(" ").toFloatOrNull() ?: 0f)
        //val fused = applyFilter(blended, 0f, filterW, 0.1f)
        val turnRate = abs(lastGyroZDegS)

        // Reduce GNSS influence when turning
        val turnFactor = (1f - turnRate / 20f).coerceIn(0.2f, 1f)
        val dynamicW = filterW * turnFactor
        val fused = applyFilter(blended, 0f, dynamicW, 0.1f)

        updateMagCalibration(
            gpsHeading    = blended,
            rawMagHeading = state.headingDeg,
            speedKt       = speedKt,
            seaState      = state.seaState,
            gpsConf       = conf
        )

        state = state.copy(
            headingDeg            = fused,
            speedKnots            = speedKt,
            hasHeading            = true,
            hasFix                = gnssQuality >= 4,
            satellites            = satellites,
            headingConf           = conf,
            magCalibrated         = magCalibrated,
            tarMisalignDeg        = tarMisalignEstimate,
            tarMisalignCalibrated = tarMisalignCalibrated,
            source                = "gnss+imu",
            debugMsg              = "A2: tar=${"%.1f".format(tarHeadingDeg)}${if (tarMisalignCalibrated) "(+${"%.1f".format(tarMisalignEstimate)}°)" else ""}(w=${"%.2f".format(wTar)}) rmc=${"%.1f".format(cachedRmcHeading)}(w=${"%.2f".format(wRmc)}) → ${"%.1f".format(fused)} conf=${"%.2f".format(conf)} spd=$speedKt"
        )
        onFusedHeading?.invoke(state)
    }

    // ── 0xA3 — Position + RMC COG (0.2 Hz) ──────────────────────────────────

    /**
     * Process GNRMC position packet (0xA3).
     * Caches speed and COG for blending with next A2 packet.
     * Also drives LC02H install misalignment auto-calibration at >4kt calm sea.
     *
     * @param latDeg    Decimal degrees latitude
     * @param lonDeg    Decimal degrees longitude
     * @param speedKt   Speed over ground in knots
     * @param cogDeg    Course over ground (RMC heading at speed) in degrees
     * @param hasFix    True if RMC status is 'A' (active)
     */
    fun processA3(
        latDeg:  Double,
        lonDeg:  Double,
        speedKt: Float,
        cogDeg:  Float,
        hasFix:  Boolean
    ) {
        // Cache RMC COG + speed for blending in processA2
        cachedRmcSpeed   = speedKt
        cachedRmcHeading = cogDeg
        cachedRmcValid   = hasFix && speedKt >= 0.5f

        // Auto-calibrate LC02H mounting misalignment vs COG at high speed + calm sea
        if (speedKt >= TAR_MISALIGN_SPEED_KT) {
            updateTarMisalignment(cogDeg, state.headingDeg, speedKt, state.seaState)
        }

        state = state.copy(
            latDeg                = latDeg,
            lonDeg                = lonDeg,
            speedKnots            = speedKt,
            hasFix                = hasFix,
            tarMisalignDeg        = tarMisalignEstimate,
            tarMisalignCalibrated = tarMisalignCalibrated,
            source                = state.source,
            debugMsg              = "A3: lat=${"%.6f".format(latDeg)} lon=${"%.6f".format(lonDeg)} spd=${"%.2f".format(speedKt)}kt cog=${"%.1f".format(cogDeg)}${if (tarMisalignCalibrated) " misalign=${"%.1f".format(tarMisalignEstimate)}°" else ""}"
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
        cogDeg:    Float?,
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
            // Only update hasHeading if phone actually has a heading.
            // Don't overwrite hasHeading=true that was set by IMU/mag (processA1).
            hasHeading  = if (hasHdg) true else state.hasHeading,
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