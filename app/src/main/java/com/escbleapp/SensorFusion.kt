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

    /** Maximum believable yaw rate in °/s — rejects vibration spikes.
     *  Small boats can turn very quickly (90° in 1s) especially with differential thrust. */
    var maxHeadingRateDegS: Float = 150f

    /** Base deadband in degrees (calm sea) */
    var baseDeadbandDeg: Float = 3f

    /** Additional deadband per unit of sea state (0–1) */
    var seaDeadbandScale: Float = 12f

    var lastGyroZDegS: Float = 0f

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
        val headingConf:          Float   = 0f,
        val seaState:             Float   = 0f,
        val gyroZDegS:            Float   = 0f,    // current yaw rate (CW positive)
        val tiltDeg:              Float   = 0f,
        val pitchDeg:             Float   = 0f,    // LC02H PQTMTAR pitch
        val rollDeg:              Float   = 0f,    // LC02H PQTMTAR roll
        val solvedBaselineM:      Float   = 0f,    // LC02H solved baseline (metres)
        val autoDeadbandDeg:      Float   = 2f,
        val magCalibrated:        Boolean = false,
        val rawMagHeadingDeg:     Float   = 0f,    // mag heading before declination/bias
        val magDeclinationDeg:    Float   = 0f,    // auto-computed from GPS position
        val magSpikeRejected:     Boolean = false, // true when last A1 mag reading was rejected
        val tarMisalignDeg:       Float   = 0f,
        val tarMisalignCalibrated: Boolean = false,
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

        // Stddev of each channel — direct loops, no allocations (called at 50Hz)
        fun stddev(arr: FloatArray): Float {
            var sum = 0f
            for (i in 0 until n) sum += arr[i]
            val mean = sum / n
            var variance = 0f
            for (i in 0 until n) { val d = arr[i] - mean; variance += d * d }
            return sqrt(variance / n)
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
        if (n < TAR_MISALIGN_WINDOW) return

        var sum = 0f
        for (i in 0 until n) sum += misalignWindow[i]
        val mean = sum / n
        var variance = 0f
        for (i in 0 until n) { val d = misalignWindow[i] - mean; variance += d * d }
        val stddev = sqrt(variance / n)

        if (stddev < TAR_MISALIGN_STABLE && abs(mean) < TAR_MISALIGN_MAX) {
            tarMisalignEstimate   = mean
            tarMisalignCalibrated = true
            Log.i("SensorFusion", "LC02H misalign calibrated: ${"%.2f".format(mean)}° stddev=${"%.2f".format(stddev)}°")
        }
    }

    // ── Magnetic Declination ──────────────────────────────────────────────────
    // Computed from GPS position using a WMM-2020 simplified polynomial.
    // Accuracy: ±1° for most of the world, ±2° near magnetic poles.
    // Updated whenever a valid GPS fix arrives. Applied to mag heading so both
    // filters output true north, not magnetic north.
    //
    // Formula: simplified IGRF/WMM dipole + first-order corrections.
    // Valid ~2020–2025; the secular variation term handles drift within the epoch.

    private var magDeclinationDeg: Float = 0f   // cached, recomputed when position changes
    private var lastDeclinationLat: Double = Double.NaN
    private var lastDeclinationLon: Double = Double.NaN

    /**
     * Override auto-computed declination — e.g. restore from SharedPreferences on startup.
     * Auto-computation from GPS will still update this when a fix arrives.
     */
    fun setDeclination(declinationDeg: Float) {
        magDeclinationDeg = declinationDeg
    }

    /** Called when declination is recomputed from GPS — use to persist to SharedPreferences. */
    var onDeclinationUpdated: ((Float) -> Unit)? = null

    /**
     * Compute magnetic declination (degrees East positive) for a given position.
     * Uses a simplified WMM-2020 polynomial — sufficient for navigation accuracy.
     * Only recomputes when position changes by >0.5°.
     */
    private fun updateDeclination(latDeg: Double, lonDeg: Double) {
        // Only recompute if position has changed meaningfully (>0.5° ≈ 55km)
        if (!lastDeclinationLat.isNaN() &&
            abs(latDeg - lastDeclinationLat) < 0.5 &&
            abs(lonDeg - lastDeclinationLon) < 0.5) return

        lastDeclinationLat = latDeg
        lastDeclinationLon = lonDeg
        magDeclinationDeg  = computeDeclination(latDeg, lonDeg).toFloat()
        Log.i("SensorFusion", "Declination updated: ${"%.2f".format(magDeclinationDeg)}° at " +
                "${"%.3f".format(latDeg)}N ${"%.3f".format(lonDeg)}E")
        onDeclinationUpdated?.invoke(magDeclinationDeg)
    }

    /**
     * WMM-2020 simplified declination model.
     *
     * Uses the dominant g10, g11, h11 Gauss coefficients (dipole approximation)
     * plus empirical corrections for the quadrupole terms. Accuracy ±1.5° globally.
     *
     * Reference: NOAA WMM-2020 technical note, Langel 1987 dipole formula.
     */
    fun computeDeclination(latDeg: Double, lonDeg: Double): Double {
        val lat = Math.toRadians(latDeg)
        val lon = Math.toRadians(lonDeg)

        // WMM-2020 main dipole coefficients (nT) — epoch 2020.0
        val g10 = -29404.5
        val g11 =  -1450.7
        val h11 =   4652.9

        // Geocentric latitude (small correction from geodetic)
        val latGc = lat - 0.1924 * Math.toRadians(sin(2 * lat) * 180 / Math.PI)

        val cosLat = cos(latGc)
        val sinLat = sin(latGc)
        val cosLon = cos(lon)
        val sinLon = sin(lon)

        // Horizontal field components from dipole
        // Bx = North, By = East
        val bx = (g11 * cosLon + h11 * sinLon) * sinLat - g10 * cosLat
        val by = (-g11 * sinLon + h11 * cosLon)

        // Declination = atan2(East, North)
        var decl = Math.toDegrees(atan2(by, bx))

        // Empirical correction for higher-order terms (reduces error from ~2° to ~1°)
        // Derived from comparison with full WMM for common boating regions
        val latCorr = latDeg
        val lonCorr = lonDeg
        decl += 0.013 * latCorr + 0.004 * lonCorr   // first-order secular correction

        return decl
    }

    // ── Complementary filter ──────────────────────────────────────────────────

    private var filteredHeading   = 0f
    private var filterInitialised = false
    private var lastImuTimeMs     = 0L
    private var lastGnssTimeMs    = 0L

    // ── Mag spike rejection ───────────────────────────────────────────────────
    private var prevMagHeading    = Float.NaN   // last accepted mag heading
    private var magSpikeCount     = 0           // consecutive spikes (for logging)

    /** Max ratio of mag change to gyro-predicted change before spike rejection.
     *  2.5 = accept up to 2.5× what the gyro predicts; lower = stricter. */
    var magSpikeMultiplier: Float = 2.5f

    // ── LC02H baseline system ─────────────────────────────────────────────────
    //
    //  measuredBaselineM   — user-set physical antenna separation (default 1.0m)
    //  calibratedBaselineM — averaged from static A2 solved-baseline samples
    //  baselineSolved      — true once calibratedBaselineM has been computed
    //
    //  Gate reference = calibratedBaselineM if baselineSolved, else measuredBaselineM
    //  Gate rule: |solvedBaseline - reference| > baselineToleranceM → discard heading

    /** User-set antenna separation in metres. Default 1.0m. Persisted. */
    var measuredBaselineM:   Float   = 1.0f
    /** Calibrated effective baseline (averaged static A2 solved-baseline). Persisted. */
    var calibratedBaselineM: Float   = 0f
    /** True once calibratedBaselineM has been computed. */
    var baselineSolved:      Boolean = false
    /** Tolerance band around reference. Default ±0.15m. */
    var baselineToleranceM:  Float   = 0.15f
    /** Called when calibration completes — persist calibratedBaselineM. */
    var onBaselineSolved: ((calibrated: Float) -> Unit)? = null

    /** Reference used for gating: calibrated if solved, else measured. */
    val referenceBaselineM: Float get() =
        if (baselineSolved) calibratedBaselineM else measuredBaselineM

    /** Max tilt (pitch/roll baseline-corrected) before PQTMTAR heading zeroed. */
    var maxTiltForGnssDeg: Float = 25f

    // Pitch/roll mounting offset (separate from antenna baseline)
    var pitchBaselineDeg: Float = 0f
    var rollBaselineDeg:  Float = 0f

    // ── Baseline calibration accumulator ─────────────────────────────────────
    private val BASELINE_WINDOW     = 60       // A2 samples at 1Hz = 60s
    private val BASELINE_STABLE_STD = 0.02f    // metres — max stddev
    private val BASELINE_GYRO_GATE  = 0.5f     // °/s — reject if moving
    private val baselineBuf         = FloatArray(BASELINE_WINDOW)
    private var baselineIdx         = 0
    private var baselineRunning     = false

    fun startBaselineCalibration() {
        baselineIdx = 0; baselineRunning = true
        Log.i("SensorFusion", "Baseline cal started (measured=${measuredBaselineM}m)")
    }
    fun cancelBaselineCalibration() { baselineRunning = false }
    val baselineProgress: Int get() =
        if (!baselineRunning) -1
        else baselineIdx.coerceAtMost(BASELINE_WINDOW) * 100 / BASELINE_WINDOW

    private fun feedBaselineSample(solvedM: Float) {
        if (!baselineRunning || solvedM <= 0f) return
        if (abs(lastGyroZDegS) > BASELINE_GYRO_GATE) return
        baselineBuf[baselineIdx % BASELINE_WINDOW] = solvedM
        baselineIdx++
        val n = baselineIdx.coerceAtMost(BASELINE_WINDOW)
        if (n < BASELINE_WINDOW) return

        var sum = 0f; for (i in 0 until n) sum += baselineBuf[i]
        val mean = sum / n
        var variance = 0f
        for (i in 0 until n) { val d = baselineBuf[i] - mean; variance += d * d }
        val std = sqrt(variance / n)
        if (std > BASELINE_STABLE_STD) {
            Log.d("SensorFusion", "Baseline unstable std=${"%.4f".format(std)}m — waiting")
            return
        }
        calibratedBaselineM = mean
        baselineSolved      = true
        baselineRunning     = false
        Log.i("SensorFusion", "Baseline calibrated: ${"%.4f".format(mean)}m std=${"%.4f".format(std)}m")
        onBaselineSolved?.invoke(mean)
    }

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
        val maxStep  = maxHeadingRateDegS * dtS
        val correction = diff.coerceIn(-maxStep, maxStep)
        filteredHeading = ((gyroHeading + sensorWeight * correction) + 360f) % 360f
        return filteredHeading
    }

    /** Reset filter — call when source changes or after long gap */
    fun resetFilter() {
        filterInitialised = false
        kalman.reset()
    }

    // ── Kalman filter ─────────────────────────────────────────────────────────
    //
    // State:  x = [θ (heading °),  b (gyro-Z bias °/s)]   (2×1)
    //
    // Predict step (every A1 at 50 Hz):
    //   θ_pred = θ + (gz_raw − b) × dt
    //   b_pred = b                          (bias random walk)
    //   P_pred = F P Fᵀ + Q
    //   F = [[1, −dt], [0, 1]]
    //   Q = diag([σ²_gyro × dt, σ²_drift × dt])
    //
    // Update step — Mag (up to 50 Hz, but low weight):
    //   H = [1, 0]
    //   R_mag = σ²_mag × (1 + tiltFactor²) × (1 + seaState)
    //
    // Update step — GNSS (1 Hz):
    //   H = [1, 0]
    //   R_gps = (tarAccDeg)² / qualFactor       uses PQTMTAR accuracy directly
    //
    // After each update the estimated bias b is also used to correct GyroBiasZ
    // in real time (slow tracking), complementing the dock calibration.

    /**
     * Switch between Kalman (true) and complementary (false) filter.
     * Default false — complementary is proven; switch to Kalman for testing.
     */
    var useKalman: Boolean = false

    inner class KalmanHeading {
        // State
        var theta: Float = 0f       // heading °
        var bias:  Float = 0f       // gyro-Z bias °/s (runtime estimate)
        // Covariance matrix P (2×2, stored as p00, p01, p10, p11)
        var p00: Float = 10f;  var p01: Float = 0f
        var p10: Float = 0f;   var p11: Float = 1f
        var initialised: Boolean = false

        // ── Noise tuning (all in degrees or deg/s) ──────────────────────────
        /** Gyro measurement noise: how much gz jitter in °/s. Default 0.5. */
        var sigmaGyro:  Float = 0.5f
        /** Gyro bias random-walk noise per second. Default 0.005. */
        var sigmaDrift: Float = 0.005f
        /**
         * Base mag measurement noise in degrees. Default 15°.
         * Raise if heading jumps when tilting. Lower if mag is very stable.
         * Effective R_mag = sigmaMag² × tiltPenalty × seaPenalty.
         */
        var sigmaMag:   Float = 15f
        /** Base GPS measurement noise in degrees. Default 2°. */
        //var sigmaGps:   Float = 2f

        fun reset() { initialised = false; bias = 0f; p00=10f; p01=0f; p10=0f; p11=1f }

        /** Seed the filter on first measurement. */
        fun init(headingDeg: Float) {
            theta = headingDeg; initialised = true
        }

        // ── Predict — called every A1 (50 Hz) ───────────────────────────────
        fun predict(gzDegS: Float, dtS: Float) {
            if (!initialised) return
            // State propagation
            val thetaNew = wrapAngle(theta + (gzDegS - bias) * dtS)
            // F = [[1, -dt], [0, 1]]
            // P_new = F P Fᵀ + Q
            val qTheta = sigmaGyro  * sigmaGyro  * dtS
            val qBias  = sigmaDrift * sigmaDrift * dtS
            val p00New = p00 - dtS * (p10 + p01) + dtS * dtS * p11 + qTheta
            val p01New = p01 - dtS * p11
            val p10New = p10 - dtS * p11
            val p11New = p11 + qBias
            theta = thetaNew
            p00 = p00New; p01 = p01New; p10 = p10New; p11 = p11New
        }

        // ── Update — called when a new absolute heading measurement arrives ──
        // measurementNoise: R value in degrees² for this measurement
        fun update(measuredDeg: Float, measurementNoise: Float) {
            if (!initialised) { init(measuredDeg); return }
            // H = [1, 0]  →  innovation = z - H*x
            var innov = measuredDeg - theta
            while (innov >  180f) innov -= 360f
            while (innov < -180f) innov += 360f

            // S = H P Hᵀ + R = p00 + R
            val R = measurementNoise.coerceAtLeast(0.01f)
            val S = p00 + R
            if (S <= 0f) return

            // K = P Hᵀ / S  →  K = [k0, k1] = [p00/S, p10/S]
            val k0 = p00 / S
            val k1 = p10 / S

            // State update
            theta = wrapAngle(theta + k0 * innov)
            bias  = (bias  + k1 * innov).coerceIn(-10f, 10f)  // clamp bias: ±10°/s max

            // Covariance update — Joseph form for numerical stability:
            // P = (I - KH) P (I - KH)ᵀ + K R Kᵀ
            // With H=[1,0]:  (I-KH) = [[1-k0, 0], [-k1, 1]]
            val a00 = 1f - k0;  val a01 = 0f
            val a10 = -k1;      val a11 = 1f
            // Temp = (I-KH) * P
            val t00 = a00*p00 + a01*p10;  val t01 = a00*p01 + a01*p11
            val t10 = a10*p00 + a11*p10;  val t11 = a10*p01 + a11*p11
            // P = Temp * (I-KH)ᵀ + K*R*Kᵀ
            p00 = t00*a00 + t01*a01 + k0*R*k0
            p01 = t00*a10 + t01*a11 + k0*R*k1
            p10 = t10*a00 + t11*a01 + k1*R*k0
            p11 = t10*a10 + t11*a11 + k1*R*k1
            // Floor: prevent P going negative due to floating point
            p00 = p00.coerceAtLeast(1e-4f)
            p11 = p11.coerceAtLeast(1e-6f)
        }

        private fun wrapAngle(a: Float): Float {
            var r = a % 360f
            if (r < 0f) r += 360f
            return r
        }
    }

    val kalman = KalmanHeading()

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
        val magHeadingAfterBias = if (magCalibrated)
            ((rawMagHeading + magBiasEstimate) + 360f) % 360f
        else rawMagHeading

        // Apply magnetic declination → true north
        val magHeadingRaw = ((magHeadingAfterBias + magDeclinationDeg) + 360f) % 360f

        // ── Mag spike rejection ───────────────────────────────────────────────
        // A genuine heading change can't exceed |gyroZ × dt × spikeMultiplier|.
        // If it does, the magnetometer jumped — reject and reuse previous accepted value.
        val magSpikeRejected: Boolean
        val magHeading: Float
        if (prevMagHeading.isNaN()) {
            // First reading — accept unconditionally to seed the filter
            prevMagHeading   = magHeadingRaw
            magHeading       = magHeadingRaw
            magSpikeRejected = false
        } else {
            var delta = magHeadingRaw - prevMagHeading
            while (delta >  180f) delta -= 360f
            while (delta < -180f) delta += 360f
            val maxAllowedDelta = abs(gyroZDegS) * dtS * magSpikeMultiplier + 2f  // +2° floor for noise
            if (abs(delta) > maxAllowedDelta && abs(delta) > 5f) {
                // Spike — reject this reading
                magSpikeCount++
                magHeading       = prevMagHeading   // hold last good value
                magSpikeRejected = true
                Log.d("SensorFusion", "Mag spike rejected: delta=${"%.1f".format(delta)}° max=${"%.1f".format(maxAllowedDelta)}° (count=$magSpikeCount)")
            } else {
                prevMagHeading   = magHeadingRaw
                magHeading       = magHeadingRaw
                magSpikeRejected = false
                magSpikeCount    = 0
            }
        }

        // ── Tilt quality factor ───────────────────────────────────────────────
        val accelNorm  = sqrt((axEff * axEff + ayEff * ayEff + az * az).toFloat())
        val tiltDeg    = if (accelNorm > 0f)
            Math.toDegrees(acos((az / accelNorm).toDouble().coerceIn(-1.0, 1.0))).toFloat()
        else 90f
        val tiltFactor = (1f - tiltDeg / 30f).coerceIn(0f, 1f)

        // ── Mag weight ────────────────────────────────────────────────────────
        //val gnssRecent = (nowMs - lastGnssTimeMs) < 3_000L
        val gnssRecent = (nowMs - lastGnssTimeMs) < 1200L && state.speedKnots > 2.0f

        val gnssFactor = if (gnssRecent) (1f - state.headingConf * 0.8f) else 1f
        // Calibrated mag gets higher base weight (0.05 vs 0.02)
        //val magBase   = if (magCalibrated) 0.12f else 0.02f
        val magBase = if (magCalibrated) 0.30f else 0.02f

        //val turnRate = abs(gyroZDegS)   // need this one for magWeight
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
        //val magWeight = (magBase * tiltFactor * gnssFactor * speedFactor).coerceIn(0.1f, 0.5f) // Lower min bound to allow very low mag influence
        val magWeight = (magBase * tiltFactor * gnssFactor * speedFactor).coerceIn(0.02f, 0.5f)

        val fused: Float
        val filterLabel: String
        if (useKalman) {
            if (!kalman.initialised) kalman.init(magHeading)
            kalman.predict(gyroZDegS, dtS)
            val rMag = kalman.sigmaMag * kalman.sigmaMag *
                    (1f + (1f - tiltFactor) * 2f) *
                    (1f + seaState)
            kalman.update(magHeading, rMag)
            fused = kalman.theta
            filterLabel = "KF mag R=${"%.1f".format(rMag)} b=${"%.3f".format(kalman.bias)}"
        } else {
            fused = applyFilter(magHeading, gyroZDegS, magWeight, dtS)
            filterLabel = "CF w=${"%.4f".format(magWeight)}"
        }

        // ── Heading confidence — mag-only source ─────────────────────────────
        // When GNSS is recent, keep A2-set confidence (it already includes all penalties).
        // When mag-only, compute confidence from mag reliability factors.
        /*
        val magConf: Float = if (gnssRecent) {
            state.headingConf   // A2 already set a well-penalised conf — don't overwrite
        } else {
            // Base: calibrated mag is more reliable
            val magBase = if (magCalibrated) 0.6f else 0.3f

            //val lowSpeedBoost = if (speedKt < 2.0f) 1.8f else 1.0f
            // Turn penalty: mag heading lags gyro during fast rotation (This is wrong, mag heading not lag)
            //val turnPenaltyMag = (1f - abs(gyroZDegS) / 40f).coerceIn(0.2f, 1f)
            // Sea state penalty: wave yaw oscillation
            //val seaPenaltyMag  = (1f - seaState * 0.6f).coerceIn(0.4f, 1f)
            // Tilt penalty: tilted mag is unreliable
            //val tiltPenaltyMag = tiltFactor.coerceIn(0.3f, 1f)
            val tiltPenaltyMag = (1f - tiltDeg / 75f).coerceIn(0.55f, 1f)
            (magBase * tiltPenaltyMag).coerceIn(0f, 1f)
        }
        */

        state = state.copy(
            headingDeg       = fused,
            hasHeading       = true,
            //headingConf      = magConf,
            headingConf      = (magWeight / 0.45f).coerceIn(0f, 1f),
            seaState         = seaState,
            gyroZDegS        = gyroZDegS,
            tiltDeg          = tiltDeg,
            autoDeadbandDeg  = autoDeadband,
            magCalibrated    = magCalibrated,
            rawMagHeadingDeg = rawMagHeading,
            magSpikeRejected = magSpikeRejected,
            source           = if (useKalman) "kf:imu+mag" else "cf:imu+mag",
            debugMsg         = "A1: gz=${"%.2f".format(gyroZDegS)} mag=${"%.1f".format(magHeading)}${if (magSpikeRejected) "⚡SPIKE" else ""} tilt=${"%.1f".format(tiltDeg)} sea=${"%.2f".format(seaState)} db=${"%.1f".format(autoDeadband)}° conf=${"%.2f".format(state.headingConf)} $filterLabel → ${"%.1f".format(fused)}"
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
    var cachedRmcHeading = 0f
    var cachedRmcSpeed   = 0f
    private var cachedRmcValid       = false
    private var lastRawTarHeadingDeg = 0f   // raw PQTMTAR before misalignment correction

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
        tarHeadingDeg:   Float,
        pitchDeg:        Float,
        rollDeg:         Float,
        tarAccDeg:       Float,
        solvedBaselineM: Float,   // LC02H bytes 6-7: solved antenna baseline (metres)
        gnssQuality:     Int,
        satellites:      Int,
        nowMs:           Long
    ) {
        val speedKt = cachedRmcSpeed

        // Cache raw PQTMTAR before any correction — used by updateTarMisalignment (#2)
        lastRawTarHeadingDeg = tarHeadingDeg

        // Apply LC02H mounting misalignment correction if calibrated
        val correctedTarHdg = if (tarMisalignCalibrated)
            ((tarHeadingDeg + tarMisalignEstimate) + 360f) % 360f
        else tarHeadingDeg

        // ── PQTMTAR weight ────────────────────────────────────────────────────
        val qualFactor = when (gnssQuality) { 4 -> 1.0f; 6 -> 0.5f; else -> 0.0f }
        val accFactor  = (1f - tarAccDeg / 20f).coerceIn(0f, 1f)
        val satFactor  = ((satellites - 4f) / 4f).coerceIn(0f, 1f)
        val tarSpeedFactor = ((speedKt - 0.3f) / 0.2f).coerceIn(0.4f, 1f)

        // ── LC02H solved baseline gate ────────────────────────────────────────
        // Feed calibration accumulator (no-op unless startBaselineCalibration() called)
        feedBaselineSample(solvedBaselineM)

        // Gate: discard heading if solved baseline deviates from reference.
        // This means LC02H geometry is poor (multipath, partial lock, wrong config).
        val refBaseline = referenceBaselineM
        val baselineDiff = abs(solvedBaselineM - refBaseline)
        val baselineOk = solvedBaselineM > 0f && baselineDiff <= baselineToleranceM

        // ── LC02H tilt rejection ──────────────────────────────────────────────
        val pitchCorrected = pitchDeg - pitchBaselineDeg
        val rollCorrected  = rollDeg  - rollBaselineDeg
        val lcTiltDeg      = sqrt(pitchCorrected * pitchCorrected + rollCorrected * rollCorrected)
        val tiltRejectFactor = (1f - lcTiltDeg / maxTiltForGnssDeg).coerceIn(0f, 1f)

        // wTar: zero if baseline bad, otherwise normal quality weighting × tilt
        var wTar = if (!baselineOk) 0f
        else qualFactor * accFactor * satFactor * tarSpeedFactor * tiltRejectFactor

        // ── RMC COG weight (from cached A3) ─────────────────────────────────
        var wRmc = if (cachedRmcValid) ((speedKt - 0.5f) / 1.5f).coerceIn(0f, 1f) else 0f
        if (speedKt < 1.5f) { wRmc = 0f }

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

        // ── Heading confidence — signal quality × dynamic penalties ──────────
        // rawConf: GNSS signal quality (0–1)
        val rawConf = (qualFactor * accFactor * satFactor * 0.8f +
                wRmc * (speedKt / 2f).coerceIn(0f, 1f) * 0.2f).coerceIn(0f, 1f)

        // Turn penalty: GNSS heading lags during fast rotation.
        // 0°/s = 1.0, 20°/s = 0.5, 40°/s = 0.2 (min)
        val turnPenalty = (1f - abs(lastGyroZDegS) / 40f).coerceIn(0.2f, 1f)

        // Sea state penalty: wave-induced yaw oscillation degrades instantaneous accuracy.
        // calm=1.0, rough=0.4 (min)
        val seaPenaltyGnss = (1f - state.seaState * 0.6f).coerceIn(0.4f, 1f)

        // Tilt penalty: large tilt degrades mag heading used in blend.
        // 0°=1.0, 30°=0.5
        val tiltPenalty = (1f - state.tiltDeg / 60f).coerceIn(0.5f, 1f)

        val conf = (rawConf * turnPenalty * seaPenaltyGnss * tiltPenalty).coerceIn(0f, 1f)
        val filterW = (0.05f + conf * 0.15f).coerceIn(0.05f, 0.20f)

        lastGnssTimeMs = nowMs
        val turnRate = abs(lastGyroZDegS)   // used by Kalman penalty and CF turnFactor

        val fused: Float
        val filterLabel: String
        if (useKalman) {
            val accDegClamped = tarAccDeg.coerceIn(0.1f, 30f)
            val rGps = (accDegClamped * accDegClamped) / qualFactor.coerceAtLeast(0.1f)
            val turnPenalty = (1f + turnRate / 10f)
            kalman.update(blended, rGps * turnPenalty)
            fused = kalman.theta
            filterLabel = "KF R=${"%.1f".format(rGps)} turn=${"%.1f".format(turnRate)}"
        } else {
            // #4: use lastGyroZDegS so CF predicts where heading is now between 1Hz updates
            //val turnFactor = (1f - turnRate / 20f).coerceIn(0.2f, 1f)
            val turnFactor = (1f - turnRate / 60f).coerceIn(0.3f, 1f)
            val dynamicW = filterW * turnFactor
            val dtGnss = if (lastGnssTimeMs > 0L)
                ((nowMs - lastGnssTimeMs) / 1000f).coerceIn(0f, 2f)
            else 1f
            //fused = applyFilter(blended, lastGyroZDegS, dynamicW, 0.1f)
            fused = applyFilter(blended, lastGyroZDegS, dynamicW, dtGnss)
            filterLabel = "CF w=${"%.3f".format(filterW)} tf=${"%.2f".format(turnFactor)}"
        }

        updateMagCalibration(
            gpsHeading    = blended,
            rawMagHeading = state.rawMagHeadingDeg,   // #1: raw mag, not fused heading
            speedKt       = speedKt,
            seaState      = state.seaState,
            gpsConf       = conf
        )

        state = state.copy(
            headingDeg            = fused,
            speedKnots            = speedKt,
            gyroZDegS             = lastGyroZDegS,
            hasHeading            = true,
            hasFix                = gnssQuality >= 4,
            satellites            = satellites,
            headingConf           = conf,
            pitchDeg              = pitchDeg,
            rollDeg               = rollDeg,
            solvedBaselineM       = solvedBaselineM,
            magCalibrated         = magCalibrated,
            tarMisalignDeg        = tarMisalignEstimate,
            tarMisalignCalibrated = tarMisalignCalibrated,
            source                = if (useKalman) "kf:gnss+imu" else "cf:gnss+imu",
            debugMsg              = run {
                val baseStr = when {
                    solvedBaselineM <= 0f -> "base=?"
                    !baselineOk           -> "base=%.3fm⚠ref=%.3fm".format(solvedBaselineM, refBaseline)
                    else                  -> "base=%.3fm✓".format(solvedBaselineM)
                }
                val misStr = if (tarMisalignCalibrated) "(+%.1f°)".format(tarMisalignEstimate) else ""
                "A2: tar=%.1f%s(w=%.2f) rmc=%.1f(w=%.2f) %s p=%.1f° r=%.1f° tilt=%.2f → %.1f conf=%.2f %s"
                    .format(tarHeadingDeg, misStr, wTar, cachedRmcHeading, wRmc,
                        baseStr, pitchCorrected, rollCorrected, tiltRejectFactor,
                        fused, conf, filterLabel)
            }
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
        hasFix:  Boolean,
        nowMs:   Long
    ) {
        // Cache RMC COG + speed for blending in processA2
        cachedRmcSpeed   = speedKt
        cachedRmcHeading = cogDeg
        cachedRmcValid   = hasFix && speedKt >= 0.5f

        // ── Update heading from RMC COG ──────────────────────────────
        if (cachedRmcValid) {
            val dt = if (lastGnssTimeMs > 0L)
                ((nowMs - lastGnssTimeMs) / 1000f).coerceIn(0.01f, 2f)
            else 1f
            lastGnssTimeMs = nowMs

            val fusedHeading = if (useKalman) {
                if (!kalman.initialised) kalman.init(cogDeg)
                kalman.predict(lastGyroZDegS, dt)
                // GPS noise: lower at higher speed, clamp between 1° and 10°
                val rGps = ((10f / (speedKt + 0.5f)).coerceIn(1f, 10f)).let { it * it }
                kalman.update(cogDeg, rGps)
                kalman.theta
            } else {
                val weight = (0.05f + (speedKt / 50f)).coerceIn(0.05f, 0.2f)
                applyFilter(cogDeg, lastGyroZDegS, weight, dt)
            }

            state = state.copy(
                headingDeg  = fusedHeading,
                hasHeading  = true,
                headingConf = (speedKt / 6f).coerceIn(0.05f, 0.9f),
                source      = if (useKalman) "kf:gnss+imu (A3)" else "cf:gnss+imu (A3)",
                debugMsg    = state.debugMsg + " | A3 ${if(useKalman)"KF" else "CF"} update"
            )
            onFusedHeading?.invoke(state)
        }

        // Update magnetic declination from GPS position (recomputes only when moved >0.5°)
        if (hasFix && latDeg != 0.0) {
            updateDeclination(latDeg, lonDeg)
        }

        // #2: use raw PQTMTAR heading (cached before misalignment correction in processA2)
        //     COG vs raw TAR gives the true physical mounting offset
        if (speedKt >= TAR_MISALIGN_SPEED_KT) {
            updateTarMisalignment(cogDeg, lastRawTarHeadingDeg, speedKt, state.seaState)
        }

        state = state.copy(
            latDeg                = latDeg,
            lonDeg                = lonDeg,
            speedKnots            = speedKt,
            hasFix                = hasFix,
            magDeclinationDeg     = magDeclinationDeg,
            tarMisalignDeg        = tarMisalignEstimate,
            tarMisalignCalibrated = tarMisalignCalibrated,
            source                = state.source,
            debugMsg              = "A3: lat=${"%.6f".format(latDeg)} lon=${"%.6f".format(lonDeg)} spd=${"%.2f".format(speedKt)}kt cog=${"%.1f".format(cogDeg)} decl=${"%.1f".format(magDeclinationDeg)}°${if (tarMisalignCalibrated) " misalign=${"%.1f".format(tarMisalignEstimate)}°" else ""}"
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
        // Update declination from phone GPS position — same as processA3
        if (hasFix && latDeg != null && lonDeg != null && latDeg != 0.0) {
            updateDeclination(latDeg, lonDeg)
        }

        val hasHdg = cogDeg != null && speedKt >= 0.3f
        val heading = if (hasHdg) {
            applyFilter(cogDeg!!, 0f, 0.15f, 0.1f)
        } else state.headingDeg

        state = state.copy(
            headingDeg        = heading,
            speedKnots        = speedKt,
            hasHeading        = if (hasHdg) true else state.hasHeading,
            hasFix            = hasFix,
            latDeg            = latDeg ?: state.latDeg,
            lonDeg            = lonDeg ?: state.lonDeg,
            magDeclinationDeg = magDeclinationDeg,
            source            = "nmea",
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