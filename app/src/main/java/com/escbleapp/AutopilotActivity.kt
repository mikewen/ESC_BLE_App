package com.escbleapp

import android.annotation.SuppressLint
import android.bluetooth.BluetoothDevice
import android.content.Context
import android.content.Intent
import android.content.SharedPreferences
import android.os.Bundle
import android.os.Handler
import android.os.Looper
import android.os.VibrationEffect
import android.os.VibratorManager
import android.os.Vibrator
import android.view.MotionEvent
import android.widget.SeekBar
import android.widget.Toast
import androidx.activity.addCallback
import androidx.activity.result.contract.ActivityResultContracts
import androidx.appcompat.app.AppCompatActivity
import com.escbleapp.databinding.ActivityAutopilotBinding
import no.nordicsemi.android.ble.observer.ConnectionObserver
import kotlin.math.*

/**
 * AutopilotActivity — Hold Course with differential thrust.
 *
 * Works with AC6329C which provides heading from GPS/GNSS or 9DoF IMU via ae02.
 * Falls back to phone GPS heading if BLE heading unavailable.
 *
 * Algorithm: Simple proportional controller
 *   heading_error = target - actual   (wrapped to ±180°)
 *   differential  = error × Kp        (clamped to ±MAX_DIFF)
 *   port_pct      = base_speed + differential   (if error > 0, turn right → port faster)
 *   stbd_pct      = base_speed - differential
 *
 * User controls:
 *   Course ±1°, ±10° buttons
 *   Speed ▲▼ hold buttons + slider (0–100%)
 *   ENGAGE / DISENGAGE
 *   SET CURRENT HEADING AS TARGET
 */
class AutopilotActivity : AppCompatActivity() {
    enum class BoatType { TRIMARAN, MONOHULL }
    private var boatType = BoatType.TRIMARAN

    private lateinit var binding: ActivityAutopilotBinding
    private lateinit var bleManager: AC6328BleManager
    private lateinit var gpsManager: GpsManager
    private var remoteBle: RemoteBleManager? = null
    private val apLogger = AutopilotLogger()
    private val handler = Handler(Looper.getMainLooper())

    // ── Mode ──────────────────────────────────────────────────────────────────
    private var escMode = true

    // ── Autopilot state ───────────────────────────────────────────────────────
    private var engaged       = false
    private var targetHeading = 0f    // degrees 0–360
    private var baseSpeedPct  = 0     // 0–100 (both motors at this when on course)
    private var actualHeading = 0f
    private var hasHeading    = false

    // ── PI controller ─────────────────────────────────────────────────────────
    // All gains and deadband loaded from SharedPreferences — adjustable live
    private val KP_DEFAULT       = 0.8f
    private val KI_DEFAULT       = 0.05f
    private val KP_STEP          = 0.05f
    private val KI_STEP          = 0.005f
    private val DEADBAND_DEFAULT = 3.0f   // degrees — no correction within this band
    private val DEADBAND_STEP    = 0.5f
    private val MAX_DIFF_PCT     = 30
    private val MAX_INTEGRAL     = 20f

    private var Kp              = KP_DEFAULT
    private var Ki              = KI_DEFAULT
    private var baseDeadbandDeg = DEADBAND_DEFAULT  // user-set base, saved to prefs
    private var deadbandDeg     = DEADBAND_DEFAULT  // effective = base + sea state addition
    private var autoDeadbandEnabled = true          // sea state adds to DB when true
    private var headingIntegral = 0f
    private var headingConfidence = 1f

    private lateinit var prefs: SharedPreferences

    // ── Timing ────────────────────────────────────────────────────────────────
    private val CONTROL_INTERVAL_MS = 200L   // 5 Hz control loop
    private val HOLD_INTERVAL_MS    = 100L   // speed ramp tick
    private var isConnected         = false

    companion object {
        const val EXTRA_DEVICE           = "extra_device"
        const val EXTRA_DEVICE_NAME      = "extra_device_name"
        const val EXTRA_ESC_MODE         = "extra_esc_mode"
        const val EXTRA_INIT_SPEED_PCT   = "extra_init_speed_pct"
        const val EXTRA_REMOTE_DEVICE    = "extra_remote_device"
        const val EXTRA_SENSOR2_DEVICE   = "extra_sensor2_device"
        const val EXTRA_SENSOR2_NAME     = "extra_sensor2_name"
    }

    // Map picker result launcher
    private val mapPickerLauncher = registerForActivityResult(
        ActivityResultContracts.StartActivityForResult()
    ) { result ->
        if (result.resultCode == RESULT_OK) {
            val data = result.data ?: return@registerForActivityResult
            val bearing = data.getFloatExtra(MapPickerActivity.RESULT_TARGET_BEARING, 0f)
            targetLat   = data.getDoubleExtra(MapPickerActivity.RESULT_TARGET_LAT, 0.0)
            targetLon   = data.getDoubleExtra(MapPickerActivity.RESULT_TARGET_LON, 0.0)
            hasWaypoint = true
            targetHeading = bearing
            updateCourseDisplay()
            showToast("Target set → %.0f".format(bearing))
        }
    }

    // Waypoint tracking
    private var targetLat   = 0.0
    private var targetLon   = 0.0
    private var hasWaypoint = false

    private val controlRunnable = object : Runnable {
        override fun run() {
            if (engaged && isConnected) {
                runControlStep()
                handler.postDelayed(this, CONTROL_INTERVAL_MS)
            }
        }
    }

    private fun loadBoatTuning() {
        when (boatType) {
            BoatType.TRIMARAN -> {
                Kp = 0.5f; Ki = 0.02f; baseDeadbandDeg = 2f
            }
            BoatType.MONOHULL -> {
                Kp = 1.2f; Ki = 0.08f; baseDeadbandDeg = 4f
            }
        }
    }

    // ── Lifecycle ─────────────────────────────────────────────────────────────

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        binding = ActivityAutopilotBinding.inflate(layoutInflater)
        setContentView(binding.root)

        val device: BluetoothDevice = if (android.os.Build.VERSION.SDK_INT >= 33) {
            intent.getParcelableExtra(EXTRA_DEVICE, BluetoothDevice::class.java)!!
        } else {
            @Suppress("DEPRECATION") intent.getParcelableExtra(EXTRA_DEVICE)!!
        }
        val deviceName  = intent.getStringExtra(EXTRA_DEVICE_NAME) ?: "AC6329C"
        escMode         = intent.getBooleanExtra(EXTRA_ESC_MODE, true)
        baseSpeedPct    = intent.getIntExtra(EXTRA_INIT_SPEED_PCT, 0)

        loadBoatTuning()
        // Load saved Kp/Ki
        prefs       = getSharedPreferences("autopilot_prefs", Context.MODE_PRIVATE)
        Kp          = prefs.getFloat("kp", KP_DEFAULT)
        Ki          = prefs.getFloat("ki", KI_DEFAULT)
        baseDeadbandDeg = prefs.getFloat("deadband", DEADBAND_DEFAULT)
        deadbandDeg = baseDeadbandDeg

        binding.tvApDeviceName.text = "⚡ $deviceName"

        setupBleManager(device)
        setupGps()
        setupCourseButtons()
        setupSpeedControls()
        setupEngageButtons()
        setupTuning()
        setupBackPress()

        updateSpeedDisplay()
        updateCourseDisplay()

        // Connect remote if passed from ControlActivity / MainActivity
        val remoteDevice: BluetoothDevice? = if (android.os.Build.VERSION.SDK_INT >= 33)
            intent.getParcelableExtra(EXTRA_REMOTE_DEVICE, BluetoothDevice::class.java)
        else @Suppress("DEPRECATION") intent.getParcelableExtra(EXTRA_REMOTE_DEVICE)
        remoteDevice?.let { connectRemote(it) }

        // IMU/mag sensor2 — GpsManager keeps connection; re-attach status callback
        val sensor2Name = intent.getStringExtra(EXTRA_SENSOR2_NAME)
        if (sensor2Name != null) {
            gpsManager.onSensor2Status = { msg -> runOnUiThread {
                binding.tvSensor2Status.text = msg
            }}
            if (!gpsManager.isSensor2Connected) {
                val s2Dev: BluetoothDevice? = if (android.os.Build.VERSION.SDK_INT >= 33)
                    intent.getParcelableExtra(EXTRA_SENSOR2_DEVICE, BluetoothDevice::class.java)
                else @Suppress("DEPRECATION") intent.getParcelableExtra(EXTRA_SENSOR2_DEVICE)
                s2Dev?.let { gpsManager.connectSensor2(it, sensor2Name) }
            }
        }
    }

    override fun onDestroy() {
        super.onDestroy()
        handler.removeCallbacksAndMessages(null)
        disengage()
        if (isConnected) { bleManager.stopMotors(); bleManager.disconnect().enqueue() }
        gpsManager.stopPhoneGps()
        bleManager.close()
        remoteBle?.disconnect()?.enqueue()
        remoteBle?.close()
    }

    // ── BLE ───────────────────────────────────────────────────────────────────

    private fun setupBleManager(device: BluetoothDevice) {
        bleManager = AC6328BleManager(this)

        bleManager.setConnectionObserver(object : ConnectionObserver {
            override fun onDeviceConnecting(d: BluetoothDevice)    = runOnUiThread { binding.tvApStatus.text = "✈ Connecting…" }
            override fun onDeviceDisconnecting(d: BluetoothDevice) = runOnUiThread { binding.tvApStatus.text = "✈ Disconnecting…" }
            override fun onDeviceReady(d: BluetoothDevice) {}
            override fun onDeviceConnected(d: BluetoothDevice) = runOnUiThread {
                isConnected = true
                binding.tvApStatus.text = "✈ AUTOPILOT · STANDBY"
                if (escMode) bleManager.setEscMode() else bleManager.setBldcMode()
                bleManager.stopMotors()
            }
            override fun onDeviceFailedToConnect(d: BluetoothDevice, reason: Int) = runOnUiThread {
                isConnected = false; binding.tvApStatus.text = "✈ Failed ($reason)"
            }
            override fun onDeviceDisconnected(d: BluetoothDevice, reason: Int) = runOnUiThread {
                isConnected = false
                disengage()
                binding.tvApStatus.text = "✈ Disconnected"
            }
        })

        // ae02 may carry NMEA from AC6329C GPS/9DoF
        bleManager.onAe02Raw = { bytes -> gpsManager.feedAe02Bytes(bytes) }
        bleManager.onError   = { msg  -> runOnUiThread { showToast(msg) } }
        bleManager.connectToDevice(device)
    }

    // ── GPS ───────────────────────────────────────────────────────────────────

    // Location permission launcher — needed if user arrives at autopilot before granting
    private val locationPermLauncher = registerForActivityResult(
        androidx.activity.result.contract.ActivityResultContracts.RequestPermission()
    ) { granted ->
        if (granted) gpsManager.startPhoneGps()
        else showToast("Location permission denied — GPS heading unavailable")
    }

    private fun setupGps() {
        gpsManager = GpsManager(this)
        gpsManager.accelRotated180 = true   // QMI8658C ax/ay flipped 180° vs MMC5603 on this PCB
        gpsManager.onUpdate = { data -> runOnUiThread { onGpsUpdate(data) } }

        if (gpsManager.hasLocationPermission()) gpsManager.startPhoneGps()
        else locationPermLauncher.launch(android.Manifest.permission.ACCESS_FINE_LOCATION)
    }

    @SuppressLint("SetTextI18n")
    private fun onGpsUpdate(data: GpsManager.GpsData) {
        actualHeading     = data.headingDeg
        hasHeading        = data.hasHeading
        headingConfidence = data.headingConfidence

        // Auto deadband: base + sea state addition (only when toggle is on)
        val fusionState  = gpsManager.fusion.getState()
        val seaState     = fusionState.seaState
        deadbandDeg = if (autoDeadbandEnabled)
            (baseDeadbandDeg + seaState * 8f).coerceIn(0f, 15f)
        else baseDeadbandDeg
        binding.tvDeadbandValue.text = "%.1f".format(baseDeadbandDeg) + "°" +
                if (autoDeadbandEnabled && seaState > 0.05f) "(+${"%.1f".format(deadbandDeg - baseDeadbandDeg)}°)" else ""

        // Speed — update both the small label and the new big display
        val speedStr = "%.1f kt".format(data.speedKnots)
        binding.tvApSpeed.text    = speedStr   // small status line (conf/sea label added below)
        //binding.tvApSpeedBig.text = speedStr   // large display right of TARGET
        val spannable = android.text.SpannableString(speedStr)
        // make "kn" smaller
        val unitStart = speedStr.indexOf("kt")
        if (unitStart >= 0) {
            spannable.setSpan(
                android.text.style.RelativeSizeSpan(0.2f), // 20% size
                unitStart,
                speedStr.length,
                android.text.Spannable.SPAN_EXCLUSIVE_EXCLUSIVE
            )
        }
        binding.tvApSpeedBig.text  = spannable

        // tvActualHeading = fused heading (GNSS+IMU+Mag complementary filter output)
        // same as data.headingDeg — used by autopilot controller
        binding.tvActualHeading.text = if (data.hasHeading) "%.0f".format(actualHeading) + "°" else "—°"

        // tvMagHeading = raw tilt-compensated MMC5603 magnetometer heading, BEFORE
        // gyro integration and GNSS correction. Shows true compass heading.
        // Useful to compare with ACTUAL to see how much the filter has drifted.
        binding.tvMagHeading.text = "%.0f°".format(fusionState.rawMagHeadingDeg)

        binding.apCompassView.headingDeg = actualHeading
        binding.apCompassView.hasFix     = data.hasFix

        val confPct   = (headingConfidence * 100).toInt()
        val fixLabel  = if (data.hasFix) "" else " ⚠"
        val calLabel     = if (fusionState.magCalibrated) " 🧭cal" else ""
        val misalLabel   = if (fusionState.tarMisalignCalibrated)
            " ⊾${"%.1f".format(fusionState.tarMisalignDeg)}°" else ""
        val seaLabel  = "sea:${"%.0f".format(seaState * 100)}%"
        binding.tvApSpeed.text = "%.1f kt  c:%d%%%s%s%s  %s".format(
            data.speedKnots, confPct, fixLabel, calLabel, misalLabel, seaLabel)

        if (hasWaypoint && data.latDeg != 0.0) {
            targetHeading = bearingTo(data.latDeg, data.lonDeg, targetLat, targetLon)
            val distNm    = haversineNm(data.latDeg, data.lonDeg, targetLat, targetLon)
            binding.tvCourseDisplay.text  = "%.0f".format(targetHeading) + "°"
            binding.tvCourseCardinal.text = "%.2f nm".format(distNm)
            binding.tvTargetHeading.text  = "%.0f".format(targetHeading) + "°"
            if (engaged) binding.tvApStatus.text =
                "✈ WAYPOINT → %.0f°  (%.2f nm)".format(targetHeading, distNm)
        }

        if (engaged && data.hasHeading) {
            val err = headingError(targetHeading, actualHeading)
            binding.tvHeadingError.text = "%+.0f".format(err) + "°"
        }
    }

    // ── Geo helpers (same as MapPickerActivity) ───────────────────────────────

    private fun bearingTo(lat1: Double, lon1: Double, lat2: Double, lon2: Double): Float {
        val φ1 = Math.toRadians(lat1); val φ2 = Math.toRadians(lat2)
        val Δλ = Math.toRadians(lon2 - lon1)
        val y  = sin(Δλ) * cos(φ2)
        val x  = cos(φ1) * sin(φ2) - sin(φ1) * cos(φ2) * cos(Δλ)
        return ((Math.toDegrees(atan2(y, x)) + 360) % 360).toFloat()
    }

    private fun haversineNm(lat1: Double, lon1: Double, lat2: Double, lon2: Double): Double {
        val R = 3440.065
        val dLat = Math.toRadians(lat2 - lat1)
        val dLon = Math.toRadians(lon2 - lon1)
        val a = sin(dLat/2).pow(2) +
                cos(Math.toRadians(lat1)) * cos(Math.toRadians(lat2)) * sin(dLon/2).pow(2)
        return R * 2 * asin(sqrt(a))
    }

    // ── Control loop ──────────────────────────────────────────────────────────

    /**
     * PI heading controller with deadband and confidence scaling.
     *
     * Deadband: if |error| < deadbandDeg → no correction (both motors equal).
     *   Prevents constant micro-corrections from GPS noise on a slow boat.
     *
     * Confidence scaling: multiply correction by headingConfidence (0–1).
     *   headingConfidence = clamp(speedMs / sAcc, 0, 1) from NAV2-SOL.
     *   When GPS heading is noisy (low speed or high sAcc), correction is reduced.
     */
    private fun runControlStep() {
        if (!hasHeading || !isConnected) return

        val dt    = CONTROL_INTERVAL_MS / 1000f
        val error = headingError(targetHeading, actualHeading)
        val fusionState = gpsManager.fusion.getState()

        if (Math.abs(error) < deadbandDeg) {
            headingIntegral *= 0.8f
            sendMotors(baseSpeedPct, baseSpeedPct)
            updateMotorDisplay(baseSpeedPct, baseSpeedPct, error)
            // Log: in deadband — zero correction
            apLogger.logCtrl(
                targetHdg = targetHeading, actualHdg = actualHeading,
                magHdg = fusionState.rawMagHeadingDeg, error = error,
                deadband = deadbandDeg, inDeadband = true,
                pTerm = 0f, iTerm = 0f, integral = headingIntegral,
                rawDiff = 0f, finalDiff = 0f,
                baseSpeed = baseSpeedPct, portPct = baseSpeedPct, stbdPct = baseSpeedPct,
                conf = headingConfidence, speedKt = gpsManager.getCurrentData().speedKnots,
                lat = fusionState.latDeg, lon = fusionState.lonDeg,
                seaState = fusionState.seaState, autoDeadband = fusionState.autoDeadbandDeg,
                useKalman = gpsManager.fusion.useKalman, kalmanBias = gpsManager.fusion.kalman.bias,
                source = fusionState.source
            )
            return
        }

        val newIntegral = (headingIntegral + error * dt).coerceIn(-MAX_INTEGRAL, MAX_INTEGRAL)
        val pTerm   = error * Kp
        val iTerm   = newIntegral * Ki
        val rawDiff = (pTerm + iTerm) * headingConfidence
        val speedFactor = (50f / max(baseSpeedPct, 10)).coerceIn(0.5f, 2.0f)
        val scaledDiff  = rawDiff * speedFactor
        val diff        = scaledDiff.coerceIn(-MAX_DIFF_PCT.toFloat(), MAX_DIFF_PCT.toFloat())

        if (abs(diff) < MAX_DIFF_PCT) headingIntegral = newIntegral

        val maxDiffAllowed = min(baseSpeedPct, 100 - baseSpeedPct)
        val finalDiff = diff.coerceIn(-maxDiffAllowed.toFloat(), maxDiffAllowed.toFloat())

        val portPct = (baseSpeedPct + finalDiff).roundToInt()
        val stbdPct = (baseSpeedPct - finalDiff).roundToInt()

        sendMotors(portPct, stbdPct)
        updateMotorDisplay(portPct, stbdPct, error)

        // Log CTRL row with full PI internals
        apLogger.logCtrl(
            targetHdg = targetHeading, actualHdg = actualHeading,
            magHdg = fusionState.rawMagHeadingDeg, error = error,
            deadband = deadbandDeg, inDeadband = false,
            pTerm = pTerm, iTerm = iTerm, integral = headingIntegral,
            rawDiff = rawDiff, finalDiff = finalDiff,
            baseSpeed = baseSpeedPct, portPct = portPct, stbdPct = stbdPct,
            conf = headingConfidence, speedKt = gpsManager.getCurrentData().speedKnots,
            lat = fusionState.latDeg, lon = fusionState.lonDeg,
            seaState = fusionState.seaState, autoDeadband = fusionState.autoDeadbandDeg,
            useKalman = gpsManager.fusion.useKalman, kalmanBias = gpsManager.fusion.kalman.bias,
            source = fusionState.source
        )
    }

    /** Heading error wrapped to ±180° */
    private fun headingError(target: Float, actual: Float): Float {
        var err = target - actual
        while (err >  180f) err -= 360f
        while (err < -180f) err += 360f
        return err
    }

    private fun sendMotors(portPct: Int, stbdPct: Int) {
        val portDuty: Int
        val stbdDuty: Int
        if (escMode) {
            portDuty = 500 + portPct * 5
            stbdDuty = 500 + stbdPct * 5
            bleManager.sendEscPwm(portDuty, stbdDuty)
        } else {
            portDuty = portPct * 100
            stbdDuty = stbdPct * 100
            bleManager.sendBldc(portDuty, stbdDuty)
        }
        apLogger.logCmd(portPct, stbdPct, portDuty, stbdDuty, escMode)
    }

    // ── Engage / Disengage ────────────────────────────────────────────────────

    private fun engage() {
        if (!isConnected) { showToast("Not connected"); return }

        // Allow engage with any GPS data — don't require movement.
        // hasHeading is false when stationary (speed < 0.3kt) but we can still
        // use the last known heading or the target heading as-is.
        val gpsData = gpsManager.getCurrentData()
        if (gpsData.source == GpsManager.Source.NONE) {
            showToast("No GPS data — check location permission")
            return
        }
        if (!hasHeading) {
            // Accept engagement with a warning — heading will update once moving
            showToast("⚠ Low-confidence heading — engage anyway, update when moving")
            // Use target heading as actual until real heading arrives
            actualHeading = targetHeading
        }

        engaged = true
        binding.tvApStatus.text = "✈ ENGAGED  →  %.0f".format(targetHeading)
        binding.btnEngage.backgroundTintList =
            android.content.res.ColorStateList.valueOf(android.graphics.Color.parseColor("#14FFEC"))
        binding.btnEngage.setTextColor(android.graphics.Color.parseColor("#0A1628"))
        vibrate(80)
        handler.post(controlRunnable)

        // Start autopilot log — logs CTRL+CMD rows each tick, SENS from GpsManager
        val path = apLogger.start()
        gpsManager.autopilotLogger = apLogger   // wire sensor rows
        if (path != null) showToast("📝 Logging: ${path.substringAfterLast('/')}")
    }

    private fun disengage() {
        engaged = false
        headingIntegral = 0f
        handler.removeCallbacks(controlRunnable)
        binding.tvApStatus.text = "✈ AUTOPILOT · STANDBY"
        binding.btnEngage.backgroundTintList =
            android.content.res.ColorStateList.valueOf(android.graphics.Color.parseColor("#0D4A3A"))
        binding.btnEngage.setTextColor(android.graphics.Color.parseColor("#14FFEC"))
        if (isConnected) bleManager.stopMotors()
        updateMotorDisplay(0, 0, 0f)

        // Stop log
        gpsManager.autopilotLogger = null
        apLogger.stop()
        apLogger.logFilePath?.let { showToast("📝 Saved: ${it.substringAfterLast('/')}") }
    }

    private fun setupEngageButtons() {
        binding.btnEngage.setOnClickListener    { engage() }
        binding.btnDisengage.setOnClickListener { disengage(); vibrate(80) }

        // HOLD COURSE — lock current GPS bearing as target
        binding.btnSetCurrentCourse.setOnClickListener {
            if (hasHeading) {
                hasWaypoint   = false   // cancel waypoint mode
                targetHeading = actualHeading
                updateCourseDisplay()
                showToast("Holding course %.0f".format(targetHeading))
            } else {
                showToast("No GPS heading yet")
            }
        }

        // PICK TARGET — open map picker
        binding.btnPickTarget.setOnClickListener {
            val gpsData = gpsManager.getCurrentData()
            val intent  = Intent(this, MapPickerActivity::class.java).apply {
                putExtra(MapPickerActivity.EXTRA_CURRENT_LAT,     gpsData.latDeg)
                putExtra(MapPickerActivity.EXTRA_CURRENT_LON,     gpsData.lonDeg)
                putExtra(MapPickerActivity.EXTRA_CURRENT_HEADING, actualHeading)
            }
            mapPickerLauncher.launch(intent)
        }
    }

    // ── Course buttons ────────────────────────────────────────────────────────

    private fun setupCourseButtons() {
        binding.btnCourseM10.setOnClickListener { adjustCourse(-10f) }
        binding.btnCourseM1.setOnClickListener  { adjustCourse(-1f)  }
        binding.btnCourseP1.setOnClickListener  { adjustCourse(+1f)  }
        binding.btnCourseP10.setOnClickListener { adjustCourse(+10f) }
    }

    private fun adjustCourse(delta: Float) {
        targetHeading = (targetHeading + delta + 360f) % 360f
        updateCourseDisplay()
        if (engaged) binding.tvApStatus.text = "✈ ENGAGED  →  %.0f".format(targetHeading)
        vibrate(30)
    }

    private fun headingToCardinal(deg: Float): String = when {
        deg < 22.5 || deg >= 337.5 -> "N"
        deg < 67.5  -> "NE"
        deg < 112.5 -> "E"
        deg < 157.5 -> "SE"
        deg < 202.5 -> "S"
        deg < 247.5 -> "SW"
        deg < 292.5 -> "W"
        else        -> "NW"
    }

    @SuppressLint("SetTextI18n")
    private fun updateCourseDisplay() {
        binding.tvTargetHeading.text  = "%.0f".format(targetHeading) + "°"
        binding.tvCourseDisplay.text  = "%.0f".format(targetHeading) + "°"
        binding.tvCourseCardinal.text = headingToCardinal(targetHeading)
        binding.apCompassView.invalidate()
    }

    // ── Speed controls ────────────────────────────────────────────────────────

    @SuppressLint("ClickableViewAccessibility")
    private fun setupSpeedControls() {
        binding.seekApSpeed.max      = 100
        binding.seekApSpeed.progress = baseSpeedPct

        binding.seekApSpeed.setOnSeekBarChangeListener(object : SeekBar.OnSeekBarChangeListener {
            override fun onProgressChanged(sb: SeekBar, p: Int, fromUser: Boolean) {
                if (!fromUser) return
                baseSpeedPct = p
                updateSpeedDisplay()
            }
            override fun onStartTrackingTouch(sb: SeekBar) {}
            override fun onStopTrackingTouch(sb: SeekBar) {}
        })

        // Hold ▲▼ to ramp speed
        fun holdSpeed(btn: android.widget.Button, delta: Int) {
            val ramp = object : Runnable {
                override fun run() {
                    baseSpeedPct = (baseSpeedPct + delta).coerceIn(0, 100)
                    binding.seekApSpeed.progress = baseSpeedPct
                    updateSpeedDisplay()
                    handler.postDelayed(this, HOLD_INTERVAL_MS)
                }
            }
            btn.setOnTouchListener { _, ev ->
                when (ev.action) {
                    MotionEvent.ACTION_DOWN -> { handler.post(ramp); true }
                    MotionEvent.ACTION_UP, MotionEvent.ACTION_CANCEL -> {
                        handler.removeCallbacks(ramp); true
                    }
                    else -> false
                }
            }
        }
        holdSpeed(binding.btnSpeedUp,   +1)
        holdSpeed(binding.btnSpeedDown, -1)
    }

    @SuppressLint("SetTextI18n")
    private fun updateSpeedDisplay() {
        binding.tvSpeedPct.text = "$baseSpeedPct%"
    }

    // ── Motor output display ──────────────────────────────────────────────────

    @SuppressLint("SetTextI18n")
    private fun updateMotorDisplay(portPct: Int, stbdPct: Int, headingErr: Float) {
        binding.tvApPortPct.text        = "$portPct%"
        binding.tvApStbdPct.text        = "$stbdPct%"
        binding.apProgressPort.progress = portPct
        binding.apProgressStbd.progress = stbdPct
        binding.tvDifferential.text     = "⟷" + "%+.0f".format(headingErr) + "°"
        binding.tvApTx.text             = "PORT:$portPct%  STBD:$stbdPct%  Δ=" + "%+.1f".format(headingErr) + "°"
    }

    // ── Utils ─────────────────────────────────────────────────────────────────

    private fun showToast(msg: String) = Toast.makeText(this, msg, Toast.LENGTH_SHORT).show()

    private fun vibrate(ms: Long) {
        if (android.os.Build.VERSION.SDK_INT >= android.os.Build.VERSION_CODES.S) {
            (getSystemService(Context.VIBRATOR_MANAGER_SERVICE) as VibratorManager)
                .defaultVibrator.vibrate(VibrationEffect.createOneShot(ms, VibrationEffect.DEFAULT_AMPLITUDE))
        } else {
            @Suppress("DEPRECATION")
            val vib = getSystemService(Context.VIBRATOR_SERVICE) as Vibrator
            if (android.os.Build.VERSION.SDK_INT >= 26)
                vib.vibrate(VibrationEffect.createOneShot(ms, VibrationEffect.DEFAULT_AMPLITUDE))
            else @Suppress("DEPRECATION") vib.vibrate(ms)
        }
    }

    // ── Tuning (Kp / Ki) ─────────────────────────────────────────────────────

    private fun setupTuning() {
        updateTuningDisplay()

        binding.switchAutoDeadband.isChecked = autoDeadbandEnabled
        binding.switchAutoDeadband.setOnCheckedChangeListener { _, checked ->
            autoDeadbandEnabled = checked
            if (!checked) {
                deadbandDeg = baseDeadbandDeg
                updateTuningDisplay()
            }
        }

        // Filter switch: Kalman ON = true, OFF = Complementary
        binding.switchKalman.isChecked = gpsManager.fusion.useKalman
        binding.switchKalman.setOnCheckedChangeListener { _, checked ->
            gpsManager.fusion.useKalman = checked
            gpsManager.fusion.resetFilter()   // reset both filters on switch
            showToast(if (checked) "Filter: Kalman" else "Filter: Complementary")
        }

        binding.btnKpPlus.setOnClickListener        { adjustKp(+KP_STEP) }
        binding.btnKpMinus.setOnClickListener       { adjustKp(-KP_STEP) }
        binding.btnKiPlus.setOnClickListener        { adjustKi(+KI_STEP) }
        binding.btnKiMinus.setOnClickListener       { adjustKi(-KI_STEP) }
        binding.btnDeadbandPlus.setOnClickListener  { adjustDeadband(+DEADBAND_STEP) }
        binding.btnDeadbandMinus.setOnClickListener { adjustDeadband(-DEADBAND_STEP) }

        binding.btnResetGains.setOnClickListener {
            Kp = KP_DEFAULT; Ki = KI_DEFAULT; baseDeadbandDeg = DEADBAND_DEFAULT
            deadbandDeg = DEADBAND_DEFAULT
            headingIntegral = 0f
            saveGains(); updateTuningDisplay()
            showToast("Gains reset to defaults")
        }
    }

    private fun adjustKp(delta: Float) {
        Kp = (Kp + delta).coerceIn(0.1f, 5.0f)
        saveGains(); updateTuningDisplay()
    }

    private fun adjustKi(delta: Float) {
        Ki = (Ki + delta).coerceIn(0.0f, 1.0f)
        headingIntegral = 0f
        saveGains(); updateTuningDisplay()
    }

    private fun adjustDeadband(delta: Float) {
        // Adjust the base deadband — sea state adds on top of this at runtime
        baseDeadbandDeg = (baseDeadbandDeg + delta).coerceIn(0.0f, 15.0f)
        headingIntegral = 0f
        saveGains(); updateTuningDisplay()
    }

    private fun saveGains() {
        prefs.edit()
            .putFloat("kp", Kp)
            .putFloat("ki", Ki)
            .putFloat("deadband", baseDeadbandDeg)
            .apply()
    }

    @SuppressLint("SetTextI18n")
    private fun updateTuningDisplay() {
        binding.tvKpValue.text       = "%.2f".format(Kp)
        binding.tvKiValue.text       = "%.3f".format(Ki)
        // Show base + effective: "3.0° (+2.1°sea)"
        val seaAdd = deadbandDeg - baseDeadbandDeg
        binding.tvDeadbandValue.text = if (seaAdd > 0.1f)
            "%.1f°(+%.1f°)".format(baseDeadbandDeg, seaAdd)
        else
            "%.1f".format(baseDeadbandDeg) + "°"
    }

    private fun setupBackPress() {
        onBackPressedDispatcher.addCallback(this) {
            disengage()
            finish()
        }
        binding.btnApBack.setOnClickListener {
            disengage()
            finish()
        }
    }

    // ── BLE Remote ────────────────────────────────────────────────────────────

    @SuppressLint("SetTextI18n")
    private fun connectRemote(device: BluetoothDevice) {
        remoteBle = RemoteBleManager(this)
        remoteBle!!.onConnected    = { runOnUiThread {
            binding.tvRemoteStatus.text = "🕹 ${device.name ?: "Remote"}"
        }}
        remoteBle!!.onDisconnected = { runOnUiThread {
            binding.tvRemoteStatus.text = ""
        }}
        remoteBle!!.onRemoteCommand = { cmd -> runOnUiThread { handleRemoteCommand(cmd) } }
        remoteBle!!.connectToDevice(device)
    }

    @SuppressLint("SetTextI18n")
    private fun handleRemoteCommand(cmd: RemoteBleManager.RemoteCommand) {
        when {
            cmd.isEngage     -> { engage();    showToast("Remote: ENGAGE") }
            cmd.isDisengage  -> { disengage(); showToast("Remote: DISENGAGE") }
            cmd.isHoldCourse -> {
                if (hasHeading) {
                    targetHeading = actualHeading
                    updateCourseDisplay()
                    showToast("Remote: HOLD ${targetHeading.toInt()}°")
                }
            }
            cmd.isStop -> {
                disengage()
                if (isConnected) bleManager.stopMotors()
                showToast("Remote: STOP")
            }
            cmd.isSpeedUp -> {
                baseSpeedPct = (baseSpeedPct + RemoteBleManager.SPEED_STEP).coerceAtMost(100)
                updateSpeedDisplay()
            }
            cmd.isSpeedDown -> {
                baseSpeedPct = (baseSpeedPct - RemoteBleManager.SPEED_STEP).coerceAtLeast(0)
                updateSpeedDisplay()
                if (baseSpeedPct == 0 && isConnected) bleManager.stopMotors()
            }
            cmd.isSpeedUp1   -> {
                baseSpeedPct = (baseSpeedPct + RemoteBleManager.SPEED_STEP_1).coerceAtMost(100)
                updateSpeedDisplay()
            }
            cmd.isSpeedDown1 -> {
                baseSpeedPct = (baseSpeedPct - RemoteBleManager.SPEED_STEP_1).coerceAtLeast(0)
                updateSpeedDisplay()
                if (baseSpeedPct == 0 && isConnected) bleManager.stopMotors()
            }
            cmd.isSetBothSpeed -> {
                baseSpeedPct = cmd.bothSpeedPct
                updateSpeedDisplay()
            }
            cmd.isCourse -> {
                targetHeading = ((targetHeading + cmd.courseDelta) + 360f) % 360f
                updateCourseDisplay()
            }
            // Individual motor commands — not applicable in autopilot (autopilot owns differential)
        }
    }
}