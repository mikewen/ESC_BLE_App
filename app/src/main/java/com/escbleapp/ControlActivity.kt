package com.escbleapp

import android.annotation.SuppressLint
import android.bluetooth.BluetoothDevice
import android.content.Context
import android.content.pm.PackageManager
import android.os.Bundle
import android.os.Handler
import android.os.Looper
import android.os.VibrationEffect
import android.os.VibratorManager
import android.os.Vibrator
import android.view.MotionEvent
import android.view.View
import android.widget.SeekBar
import android.widget.Toast
import androidx.activity.addCallback
import androidx.activity.result.contract.ActivityResultContracts
import androidx.appcompat.app.AlertDialog
import androidx.appcompat.app.AppCompatActivity
import com.escbleapp.databinding.ActivityControlBinding
import no.nordicsemi.android.ble.observer.ConnectionObserver

/**
 * Motor control screen.
 *
 * Mode locked at connect from device name:
 *   "ESC_PWM"  → ESC  (1000–2000µs, stop=1000)
 *   "BLDC_PWM" → BLDC (0–10000 duty, stop=0, default=500)
 *
 * Controls:
 *   ▲ / ▼  hold buttons  — ramp while held, one step on tap
 *   ⬆ / ⬇  BOTH buttons  — ramp both motors regardless of Sync
 *   Slider              — direct set
 *   Sync switch         — when ON, PORT ▲▼ also drives STBD (STBD column dimmed)
 *   STOP                — immediate stop both
 */
class ControlActivity : AppCompatActivity() {

    private lateinit var binding: ActivityControlBinding
    private lateinit var bleManager: AC6328BleManager
    private var remoteBle: RemoteBleManager? = null   // optional BLE remote controller
    private val handler = Handler(Looper.getMainLooper())

    private var escMode = true

    // Native units: ESC µs (1000–2000), BLDC duty (0–10000)
    private var portVal      = 0
    private var starboardVal = 0

    // GPS
    private lateinit var gpsManager: GpsManager
    private var speedUnitKnots = true   // false = km/h

    // Location permission launcher
    private val locationPermLauncher = registerForActivityResult(
        ActivityResultContracts.RequestPermission()
    ) { granted ->
        if (granted) gpsManager.startPhoneGps()
        else showToast("Location permission denied — phone GPS unavailable")
    }

    private val FEEDBACK_POLL_MS = 2_000L
    private val HOLD_INTERVAL_MS = 250L    // ramp tick — 4 steps/sec
    private var isConnected      = false

    companion object {
        const val EXTRA_DEVICE        = "extra_device"
        const val EXTRA_DEVICE_NAME   = "extra_device_name"
        const val EXTRA_REMOTE_DEVICE = "extra_remote_device"  // optional BLE remote
    }

    // ── Runnables ─────────────────────────────────────────────────────────────
    // NO continuous send loop — commands sent only on user action (send-on-change).
    // stopAll() sends CMD_STOP 3× for reliability.

    private val feedbackPollRunnable = object : Runnable {
        override fun run() {
            if (isConnected) { bleManager.readStatus(); handler.postDelayed(this, FEEDBACK_POLL_MS) }
        }
    }

    // ── Lifecycle ─────────────────────────────────────────────────────────────

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        binding = ActivityControlBinding.inflate(layoutInflater)
        setContentView(binding.root)

        val device: BluetoothDevice = if (android.os.Build.VERSION.SDK_INT >= 33) {
            intent.getParcelableExtra(EXTRA_DEVICE, BluetoothDevice::class.java)!!
        } else {
            @Suppress("DEPRECATION") intent.getParcelableExtra(EXTRA_DEVICE)!!
        }
        val deviceName = intent.getStringExtra(EXTRA_DEVICE_NAME) ?: "ESC Device"

        escMode = !deviceName.contains("BLDC", ignoreCase = true)

        setupBleManager(device, deviceName)
        setupModeUi()
        setupSliders()
        setupHoldButtons()
        setupMainButtons()
        setupGps()
        setupBackPress()
        updateUi(false)

        // Connect remote if one was found in MainActivity scan
        val remoteDevice: BluetoothDevice? = if (android.os.Build.VERSION.SDK_INT >= 33)
            intent.getParcelableExtra(EXTRA_REMOTE_DEVICE, BluetoothDevice::class.java)
        else @Suppress("DEPRECATION") intent.getParcelableExtra(EXTRA_REMOTE_DEVICE)
        remoteDevice?.let { connectRemote(it) }
    }

    override fun onDestroy() {
        super.onDestroy()
        handler.removeCallbacksAndMessages(null)
        gpsManager.stopLogging()
        gpsManager.stopPhoneGps()
        if (isConnected) { bleManager.stopMotors(); bleManager.disconnect().enqueue() }
        bleManager.close()
        remoteBle?.disconnect()?.enqueue()
        remoteBle?.close()
    }

    // ── Mode helpers ──────────────────────────────────────────────────────────

    private fun stopValue()    = if (escMode) AC6328BleManager.ESC_MIN     else AC6328BleManager.BLDC_MIN
    private fun maxValue()     = if (escMode) AC6328BleManager.ESC_MAX     else AC6328BleManager.BLDC_MAX
    private fun defaultValue() = if (escMode) AC6328BleManager.ESC_DEFAULT else AC6328BleManager.BLDC_DEFAULT
    // ESC:  5 duty/tick × 4/sec = 20 duty/sec → ~25s full range (500–1000, 500 units wide)
    //       5 duty = 10µs = 1% throttle — matches your old app
    // BLDC: 100/tick × 4/sec = 400/sec → ~25s full range (0–10000)
    private fun stepSize()     = if (escMode) 5 else 100

    private fun valToPct(v: Int): Int {
        val range = maxValue() - stopValue()
        return if (range <= 0) 0 else ((v - stopValue()) * 100 / range).coerceIn(0, 100)
    }

    @SuppressLint("SetTextI18n")
    private fun valToDisplay(v: Int): String = "${valToPct(v)}%"

    @SuppressLint("SetTextI18n")
    private fun valToShort(v: Int): String = "${valToPct(v)}%"

    private fun setupModeUi() {
        // #1: Do NOT set btnArm.visibility here — updateUi() is the single source of truth
        if (escMode) {
            binding.tvModeLabel.text = "ESC"
            binding.tvModeLabel.setTextColor(android.graphics.Color.parseColor("#14FFEC"))
            binding.tvModeHint.text  = "ESC · 50Hz · duty 500–1000 · stop=500 (=1000µs)"
        } else {
            binding.tvModeLabel.text = "BLDC"
            binding.tvModeLabel.setTextColor(android.graphics.Color.parseColor("#FFB300"))
            binding.tvModeHint.text  = "BLDC · duty 0–10000 · stop=0"
        }
    }

    // ── BLE ───────────────────────────────────────────────────────────────────

    private fun setupBleManager(device: BluetoothDevice, deviceName: String) {
        bleManager = AC6328BleManager(this)

        bleManager.setConnectionObserver(object : ConnectionObserver {
            override fun onDeviceConnecting(d: BluetoothDevice)    = runOnUiThread { binding.tvStatus.text = "Connecting…" }
            override fun onDeviceDisconnecting(d: BluetoothDevice) = runOnUiThread { binding.tvStatus.text = "Disconnecting…" }
            override fun onDeviceReady(d: BluetoothDevice) {}

            override fun onDeviceConnected(d: BluetoothDevice) = runOnUiThread {
                isConnected = true
                binding.tvStatus.text     = "Connected ✓"
                binding.tvDeviceName.text = deviceName
                updateUi(true)
                vibrate(50)
                handler.postDelayed({
                    if (escMode) bleManager.setEscMode() else bleManager.setBldcMode()
                    bleManager.stopMotors()          // send 1000µs stop — ESC holds it
                    portVal      = defaultValue()
                    starboardVal = defaultValue()
                    syncAllDisplays()
                    updateLivePwm()
                    showToast("${if (escMode) "ESC" else "BLDC"} ready")
                }, 400)
                handler.postDelayed(feedbackPollRunnable, 1_500)
            }

            override fun onDeviceFailedToConnect(d: BluetoothDevice, reason: Int) = runOnUiThread {
                isConnected = false; binding.tvStatus.text = "Failed ($reason)"; updateUi(false)
            }

            override fun onDeviceDisconnected(d: BluetoothDevice, reason: Int) = runOnUiThread {
                isConnected = false
                handler.removeCallbacks(feedbackPollRunnable)
                binding.tvLivePwm.text = ""
                binding.tvStatus.text = "Disconnected"
                updateUi(false)
                vibrate(200)
            }
        })

        bleManager.onFeedback = { fb -> runOnUiThread { applyFeedback(fb) } }
        bleManager.onAe02Raw  = { bytes -> gpsManager.feedAe02Bytes(bytes) }
        bleManager.onError = { msg -> runOnUiThread { showToast(msg) } }
        bleManager.connectToDevice(device)
    }

    // ── Sliders ───────────────────────────────────────────────────────────────

    private fun setupSliders() {
        binding.seekPort.max = 1000; binding.seekPort.progress = 0
        binding.seekPort.setOnSeekBarChangeListener(object : SeekBar.OnSeekBarChangeListener {
            override fun onProgressChanged(sb: SeekBar, p: Int, fromUser: Boolean) {
                if (!fromUser) return
                portVal = sliderToVal(p)
                if (binding.switchSync.isChecked) starboardVal = portVal
                refreshPort(); refreshStbd(); triggerSend()
            }
            override fun onStartTrackingTouch(sb: SeekBar) {}
            override fun onStopTrackingTouch(sb: SeekBar) {}
        })

        binding.seekStarboard.max = 1000; binding.seekStarboard.progress = 0
        binding.seekStarboard.setOnSeekBarChangeListener(object : SeekBar.OnSeekBarChangeListener {
            override fun onProgressChanged(sb: SeekBar, p: Int, fromUser: Boolean) {
                if (!fromUser) return
                starboardVal = sliderToVal(p)
                if (binding.switchSync.isChecked) portVal = starboardVal
                refreshPort(); refreshStbd(); triggerSend()
            }
            override fun onStartTrackingTouch(sb: SeekBar) {}
            override fun onStopTrackingTouch(sb: SeekBar) {}
        })
    }

    private fun sliderToVal(p: Int): Int =
        stopValue() + p * (maxValue() - stopValue()) / 1000

    // ── Hold buttons ──────────────────────────────────────────────────────────

    @SuppressLint("ClickableViewAccessibility")
    private fun setupHoldButtons() {

        // Helper: attach hold-ramp + single-tap to a button
        fun holdButton(btn: android.widget.Button, onStep: () -> Unit) {
            val ramp = object : Runnable {
                override fun run() { onStep(); handler.postDelayed(this, HOLD_INTERVAL_MS) }
            }
            btn.setOnTouchListener { _, ev ->
                when (ev.action) {
                    MotionEvent.ACTION_DOWN -> { handler.post(ramp); true }
                    MotionEvent.ACTION_UP,
                    MotionEvent.ACTION_CANCEL -> { handler.removeCallbacks(ramp); true }
                    else -> false
                }
            }
        }

        // PORT ▲▼ — when Sync ON also drives STBD
        holdButton(binding.btnPortUp) {
            portVal = (portVal + stepSize()).coerceIn(stopValue(), maxValue())
            if (binding.switchSync.isChecked) starboardVal = portVal
            refreshPort(); refreshStbd(); triggerSend()
        }
        holdButton(binding.btnPortDown) {
            portVal = (portVal - stepSize()).coerceIn(stopValue(), maxValue())
            if (binding.switchSync.isChecked) starboardVal = portVal
            refreshPort(); refreshStbd(); triggerSend()
        }

        // STBD ▲▼ — when Sync ON also drives PORT
        holdButton(binding.btnStbdUp) {
            starboardVal = (starboardVal + stepSize()).coerceIn(stopValue(), maxValue())
            if (binding.switchSync.isChecked) portVal = starboardVal
            refreshPort(); refreshStbd(); triggerSend()
        }
        holdButton(binding.btnStbdDown) {
            starboardVal = (starboardVal - stepSize()).coerceIn(stopValue(), maxValue())
            if (binding.switchSync.isChecked) portVal = starboardVal
            refreshPort(); refreshStbd(); triggerSend()
        }

        // BOTH ▲▼ — always moves both, regardless of Sync state
        holdButton(binding.btnBothUp) {
            portVal      = (portVal      + stepSize()).coerceIn(stopValue(), maxValue())
            starboardVal = (starboardVal + stepSize()).coerceIn(stopValue(), maxValue())
            refreshPort(); refreshStbd(); triggerSend()
        }
        holdButton(binding.btnBothDown) {
            portVal      = (portVal      - stepSize()).coerceIn(stopValue(), maxValue())
            starboardVal = (starboardVal - stepSize()).coerceIn(stopValue(), maxValue())
            refreshPort(); refreshStbd(); triggerSend()
        }

        // Sync switch — when turned ON, snap both to the average of current values
        binding.switchSync.setOnCheckedChangeListener { _, checked ->
            binding.tvSyncLabel.text = if (checked) "Sync ✓" else "Sync"
            if (checked) {
                // Snap both to average so neither jumps unexpectedly
                val avg = (portVal + starboardVal) / 2
                portVal = avg; starboardVal = avg
                refreshPort(); refreshStbd(); triggerSend()
            }
        }
    }

    // ── Main buttons ──────────────────────────────────────────────────────────

    private fun setupMainButtons() {
        binding.btnAutopilot.setOnClickListener {
            val intent = android.content.Intent(this, AutopilotActivity::class.java).apply {
                putExtra(AutopilotActivity.EXTRA_DEVICE,      intent.getParcelableExtra<BluetoothDevice>(EXTRA_DEVICE))
                putExtra(AutopilotActivity.EXTRA_DEVICE_NAME, intent.getStringExtra(EXTRA_DEVICE_NAME))
                putExtra(AutopilotActivity.EXTRA_ESC_MODE,    escMode)
                val pct = valToPct(portVal)
                putExtra(AutopilotActivity.EXTRA_INIT_SPEED_PCT, pct)
                // Pass remote device so autopilot can also receive remote commands
                remoteBle?.disconnect()?.enqueue()   // disconnect here — autopilot will reconnect
                val remDev: BluetoothDevice? = if (android.os.Build.VERSION.SDK_INT >= 33)
                    this@ControlActivity.intent.getParcelableExtra(EXTRA_REMOTE_DEVICE, BluetoothDevice::class.java)
                else @Suppress("DEPRECATION") this@ControlActivity.intent.getParcelableExtra(EXTRA_REMOTE_DEVICE)
                remDev?.let { putExtra(AutopilotActivity.EXTRA_REMOTE_DEVICE, it) }
            }
            startActivity(intent)
        }

        binding.btnStop.setOnClickListener { stopAll(); vibrate(80) }

        binding.btnArm.setOnClickListener {
            portVal = AC6328BleManager.ESC_MIN; starboardVal = AC6328BleManager.ESC_MIN
            syncAllDisplays(); sendCurrentVal(force = true)
            showToast("Arm: 1000µs sent")
        }

        binding.btnFullFwd.setOnClickListener {
            AlertDialog.Builder(this)
                .setTitle("⚠ Full Throttle")
                .setMessage("Send ${valToDisplay(maxValue())} to BOTH motors?\nClear propellers first!")
                .setPositiveButton("GO") { _, _ ->
                    portVal = maxValue(); starboardVal = maxValue()
                    syncAllDisplays(); sendCurrentVal(force = true)
                }
                .setNegativeButton("Cancel", null).show()
        }

        binding.btnDisconnect.setOnClickListener {
            bleManager.stopMotors()
            handler.postDelayed({ bleManager.disconnect().enqueue() }, 200)
        }

        binding.btnReadFeedback.setOnClickListener { bleManager.readStatus() }
    }

    // ── Send ──────────────────────────────────────────────────────────────────
    // Send-on-change only — one packet per user action, no continuous loop.
    // ESC does not need heartbeat; it holds last command until next.

    private fun triggerSend() {
        if (isConnected) sendCurrentVal()
    }

    private fun sendCurrentVal(force: Boolean = false) {
        if (!isConnected) return
        if (escMode) bleManager.sendEscPwm(portVal, starboardVal)
        else         bleManager.sendBldc(portVal, starboardVal)
        updateTxTelemetry()
        updateLivePwm()
    }

    private fun stopAll() {
        portVal = stopValue(); starboardVal = stopValue()
        handler.removeCallbacksAndMessages(null)
        if (isConnected) handler.postDelayed(feedbackPollRunnable, FEEDBACK_POLL_MS)
        syncAllDisplays()
        updateLivePwm()
        // Send STOP 3× staggered for reliability
        if (isConnected) {
            bleManager.stopMotors()
            handler.postDelayed({ bleManager.stopMotors() }, 50)
            handler.postDelayed({ bleManager.stopMotors() }, 100)
        }
    }

    // ── Display helpers ───────────────────────────────────────────────────────

    /** Update the live PWM badge in the mode bar — always shows actual values sent */
    @SuppressLint("SetTextI18n")
    private fun updateLivePwm() {
        if (!isConnected) { binding.tvLivePwm.text = ""; return }
        val pPct = valToPct(portVal);  val sPct = valToPct(starboardVal)
        binding.tvLivePwm.text = if (escMode) {
            // Show duty + µs equivalent: duty=700 → 1400µs
            val pUs = bleManager.dutyToUs(portVal);  val sUs = bleManager.dutyToUs(starboardVal)
            "P:${portVal}(${pUs}µs)  S:${starboardVal}(${sUs}µs)"
        } else {
            "P:$pPct%  S:$sPct%"
        }
    }

    private fun refreshPort() {
        val pct = valToPct(portVal)
        binding.tvPortBig2.text  = valToShort(portVal)
        val range = maxValue() - stopValue()
        if (range > 0) binding.seekPort.progress = (portVal - stopValue()) * 1000 / range
    }

    private fun refreshStbd() {
        val pct = valToPct(starboardVal)
        binding.tvStbdBig2.text  = valToShort(starboardVal)
        val range = maxValue() - stopValue()
        if (range > 0) binding.seekStarboard.progress = (starboardVal - stopValue()) * 1000 / range
    }

    private fun syncAllDisplays() { refreshPort(); refreshStbd() }

    @SuppressLint("SetTextI18n")
    private fun updateTxTelemetry() {
        val p    = portVal;  val s    = starboardVal
        val pPct = valToPct(p); val sPct = valToPct(s)
        binding.tvTxPacket.text = "TX  PORT:$pPct%(${p})  STBD:$sPct%(${s})"
    }

    // ── Feedback ──────────────────────────────────────────────────────────────

    @SuppressLint("SetTextI18n")
    private fun applyFeedback(fb: AC6328BleManager.FeedbackData) {
        when (fb.source) {
            "ae10-read" -> {
                if (fb.batteryMv > 0) {
                    val pct = bleManager.battMvToPercent(fb.batteryMv)
                    binding.tvBatteryMv.text  = "${fb.batteryMv}mV"
                    binding.tvBatteryPct.text = "$pct%"
                    binding.progressBattery.progress = pct
                    val col = when {
                        pct < 20 -> 0xFFFF5252.toInt()
                        pct < 50 -> 0xFFFFB300.toInt()
                        else     -> 0xFF14FFEC.toInt()
                    }
                    binding.tvBatteryMv.setTextColor(col)
                    binding.progressBattery.progressTintList =
                        android.content.res.ColorStateList.valueOf(col)
                }
                if (fb.uptimeMin >= 0) binding.tvUptime.text = "Up: ${fb.uptimeMin}min"
                binding.tvAe10Raw.text = "ae10: \"${fb.rawAe10}\""
            }
            "ae02-echo" -> {
                val cmdName = when (fb.echoCmd) {
                    AC6328BleManager.CMD_ESC_PWM.toInt()   -> "ESC"
                    AC6328BleManager.CMD_BLDC_DUTY.toInt() -> "BLDC"
                    AC6328BleManager.CMD_STOP.toInt()      -> "STOP"
                    else -> "0x%02X".format(fb.echoCmd)
                }
                binding.tvAe04Raw.text =
                    "echo $cmdName: ${fb.rawAe02.joinToString(" ") { "%02X".format(it) }}"
                if (fb.echoPort >= 0) {
                    val drift = maxOf(Math.abs(fb.echoPort - portVal), Math.abs(fb.echoStbd - starboardVal))
                    binding.tvPwmDrift.text = if (drift > 10) "⚠ Drift ±$drift" else "✓ Confirmed"
                }
            }
        }
    }

    // ── UI state ──────────────────────────────────────────────────────────────

    private fun updateUi(connected: Boolean) {
        binding.layoutControls.visibility   = if (connected) View.VISIBLE else View.GONE
        binding.layoutConnecting.visibility = if (!connected) View.VISIBLE else View.GONE
        listOf(binding.btnStop, binding.btnDisconnect,
            binding.btnReadFeedback, binding.btnFullFwd).forEach { it.isEnabled = connected }
        // ARM: only for ESC mode, always re-apply visibility here
        binding.btnArm.visibility = if (escMode) View.VISIBLE else View.GONE
        binding.btnArm.isEnabled  = connected && escMode
    }

    // ── Utils ─────────────────────────────────────────────────────────────────

    private fun showToast(msg: String) = Toast.makeText(this, msg, Toast.LENGTH_SHORT).show()

    private fun vibrate(ms: Long) {
        if (android.os.Build.VERSION.SDK_INT >= android.os.Build.VERSION_CODES.S) {
            (getSystemService(Context.VIBRATOR_MANAGER_SERVICE) as VibratorManager)
                .defaultVibrator
                .vibrate(VibrationEffect.createOneShot(ms, VibrationEffect.DEFAULT_AMPLITUDE))
        } else {
            @Suppress("DEPRECATION")
            val vib = getSystemService(Context.VIBRATOR_SERVICE) as Vibrator
            if (android.os.Build.VERSION.SDK_INT >= 26)
                vib.vibrate(VibrationEffect.createOneShot(ms, VibrationEffect.DEFAULT_AMPLITUDE))
            else @Suppress("DEPRECATION") vib.vibrate(ms)
        }
    }

    // ── GPS ───────────────────────────────────────────────────────────────────

    private fun setupGps() {
        gpsManager = GpsManager(this)
        gpsManager.accelRotated180 = true   // QMI8658C ax/ay flipped 180° vs MMC5603 on this PCB

        gpsManager.onUpdate = { data -> runOnUiThread { updateGpsUi(data) } }

        gpsManager.onNmeaDebug = { hex -> runOnUiThread {
            binding.tvAe02.text = "ae02: $hex"
        }}

        gpsManager.onLogStatus = { msg -> runOnUiThread { showToast(msg) } }

        // Show permission state immediately
        val hasPerm = gpsManager.hasLocationPermission()
        binding.tvGpsStatus.text = if (hasPerm) "GPS: permission OK, starting…"
        else "GPS: ⚠ NO PERMISSION — tap to fix"

        binding.btnSpeedUnit.setOnClickListener {
            speedUnitKnots = !speedUnitKnots
            binding.btnSpeedUnit.text = if (speedUnitKnots) "kt" else "km/h"
            binding.tvTripDistUnit.text = if (speedUnitKnots) "nm" else "km"
            updateGpsUi(gpsManager.getCurrentData())
        }

        binding.btnGpsSource.setOnClickListener {
            val preferBle = binding.btnGpsSource.text == "BLE"
            if (preferBle) {
                gpsManager.setPreferBleGps()
                binding.btnGpsSource.text = "📱"
                showToast("Preferring BLE GPS module")
            } else {
                gpsManager.setPreferPhoneGps()
                binding.btnGpsSource.text = "BLE"
                requestPhoneGps()
            }
        }

        binding.btnGpsLog.setOnClickListener {
            if (gpsManager.isLogging) {
                gpsManager.stopLogging()
                binding.btnGpsLog.text = "⏺ LOG"
                binding.btnGpsLog.setTextColor(android.graphics.Color.parseColor("#44556677"))
            } else {
                val path = gpsManager.startLogging()
                if (path != null) {
                    binding.btnGpsLog.text = "⏹ REC"
                    binding.btnGpsLog.setTextColor(android.graphics.Color.parseColor("#FF3333"))
                    showToast("Logging to Downloads/")
                } else {
                    showToast("Failed to start log — check storage permission")
                }
            }
        }

        binding.btnResetTrip.setOnClickListener {
            gpsManager.resetTrip()
            binding.tvTripDistance.text = "0.0"
            binding.tvMaxSpeed.text     = "—"
            showToast("Trip reset")
        }

        requestPhoneGps()
    }

    private fun requestPhoneGps() {
        if (gpsManager.hasLocationPermission()) {
            gpsManager.startPhoneGps()
        } else {
            locationPermLauncher.launch(android.Manifest.permission.ACCESS_FINE_LOCATION)
        }
    }

    @SuppressLint("SetTextI18n")
    private fun updateGpsUi(data: GpsManager.GpsData) {
        // Speed — always show value, dim color when accuracy is poor
        val speedStr = if (speedUnitKnots) "%.1f".format(data.speedKnots)
        else "%.1f".format(data.speedKmh)
        binding.tvSpeed.text = speedStr
        binding.tvSpeed.setTextColor(
            when {
                data.hasFix  -> android.graphics.Color.parseColor("#14FFEC")  // good fix
                data.source != GpsManager.Source.NONE -> android.graphics.Color.parseColor("#557766") // weak fix
                else -> android.graphics.Color.parseColor("#334455")          // no data
            }
        )

        // Heading
        binding.tvHeading.text = if (data.hasHeading) "%.0f°".format(data.headingDeg) else "—°"
        binding.tvHeadingCardinal.text = if (data.hasHeading) data.headingCardinal else ""

        // Compass
        binding.compassView.headingDeg = data.headingDeg
        binding.compassView.hasFix     = data.hasFix

        // GPS diagnostic status line
        binding.tvGpsStatus.text = when {
            data.source == GpsManager.Source.NONE -> "GPS: no data — check Settings → Location"
            !data.hasFix -> "GPS: weak signal (acc>200m) — move outdoors"
            else -> "GPS: fix OK · acc<200m"
        }

        // Source icon
        binding.tvGpsSource.text = when (data.source) {
            GpsManager.Source.PHONE -> "📱"
            GpsManager.Source.BLE   -> "📡"
            GpsManager.Source.NONE  -> "—"
        }
        binding.tvGpsSats.text = if (data.satellites > 0) "×${data.satellites}" else ""

        // Trip distance
        val dist = gpsManager.tripDistanceNm
        binding.tvTripDistance.text = if (speedUnitKnots)
            "%.2f".format(dist)
        else
            "%.2f".format(dist * 1.852)   // nm → km

        // Max speed
        val maxKt = gpsManager.maxSpeedKnots
        binding.tvMaxSpeed.text = if (maxKt > 0f) {
            if (speedUnitKnots) "%.1f".format(maxKt)
            else "%.1f".format(maxKt * 1.852f)
        } else "—"
    }

    private fun setupBackPress() {
        onBackPressedDispatcher.addCallback(this) { stopAll(); finish() }
    }

    // ── BLE Remote ────────────────────────────────────────────────────────────

    @SuppressLint("SetTextI18n")
    private fun connectRemote(device: BluetoothDevice) {
        remoteBle = RemoteBleManager(this)
        remoteBle!!.onConnected    = { runOnUiThread { binding.tvRemoteStatus.text = "🕹 ${device.name ?: "Remote"}" } }
        remoteBle!!.onDisconnected = { runOnUiThread { binding.tvRemoteStatus.text = "" } }
        remoteBle!!.onRemoteCommand = { cmd -> runOnUiThread { handleRemoteCommand(cmd) } }
        remoteBle!!.connectToDevice(device)
    }

    @SuppressLint("SetTextI18n")
    private fun handleRemoteCommand(cmd: RemoteBleManager.RemoteCommand) {
        if (!isConnected) return
        val step5 = if (escMode)
            (AC6328BleManager.ESC_MAX - AC6328BleManager.ESC_MIN) * RemoteBleManager.SPEED_STEP / 100
        else AC6328BleManager.BLDC_MAX * RemoteBleManager.SPEED_STEP / 100
        val step1 = if (escMode)
            (AC6328BleManager.ESC_MAX - AC6328BleManager.ESC_MIN) * RemoteBleManager.SPEED_STEP_1 / 100
        else AC6328BleManager.BLDC_MAX * RemoteBleManager.SPEED_STEP_1 / 100
        val range = maxValue() - stopValue()

        when {
            // Both motors
            cmd.isStop        -> { stopAll(); showToast("Remote: STOP") }
            cmd.isSpeedUp     -> {
                portVal = (portVal + step5).coerceAtMost(maxValue())
                starboardVal = (starboardVal + step5).coerceAtMost(maxValue())
                syncAllDisplays(); triggerSend()
            }
            cmd.isSpeedDown   -> {
                portVal = (portVal - step5).coerceAtLeast(stopValue())
                starboardVal = (starboardVal - step5).coerceAtLeast(stopValue())
                syncAllDisplays(); triggerSend()
            }
            cmd.isSpeedUp1    -> {
                portVal = (portVal + step1).coerceAtMost(maxValue())
                starboardVal = (starboardVal + step1).coerceAtMost(maxValue())
                syncAllDisplays(); triggerSend()
            }
            cmd.isSpeedDown1  -> {
                portVal = (portVal - step1).coerceAtLeast(stopValue())
                starboardVal = (starboardVal - step1).coerceAtLeast(stopValue())
                syncAllDisplays(); triggerSend()
            }
            cmd.isSetBothSpeed -> {
                portVal = stopValue() + range * cmd.bothSpeedPct / 100
                starboardVal = portVal
                syncAllDisplays(); triggerSend()
            }

            // Sync toggle
            cmd.isSyncOn  -> { binding.switchSync.isChecked = true  }
            cmd.isSyncOff -> { binding.switchSync.isChecked = false }

            // PORT motor
            cmd.isPortCmd && cmd.isMotorUp       -> {
                portVal = (portVal + step5).coerceAtMost(maxValue())
                refreshPort(); triggerSend()
            }
            cmd.isPortCmd && cmd.isMotorDown     -> {
                portVal = (portVal - step5).coerceAtLeast(stopValue())
                refreshPort(); triggerSend()
            }
            cmd.isPortCmd && cmd.isMotorUp1      -> {
                portVal = (portVal + step1).coerceAtMost(maxValue())
                refreshPort(); triggerSend()
            }
            cmd.isPortCmd && cmd.isMotorDown1    -> {
                portVal = (portVal - step1).coerceAtLeast(stopValue())
                refreshPort(); triggerSend()
            }
            cmd.isPortCmd && cmd.isMotorAbsolute -> {
                portVal = stopValue() + range * cmd.motorAbsPct / 100
                refreshPort(); triggerSend()
            }

            // STBD motor
            cmd.isStbdCmd && cmd.isMotorUp       -> {
                starboardVal = (starboardVal + step5).coerceAtMost(maxValue())
                refreshStbd(); triggerSend()
            }
            cmd.isStbdCmd && cmd.isMotorDown     -> {
                starboardVal = (starboardVal - step5).coerceAtLeast(stopValue())
                refreshStbd(); triggerSend()
            }
            cmd.isStbdCmd && cmd.isMotorUp1      -> {
                starboardVal = (starboardVal + step1).coerceAtMost(maxValue())
                refreshStbd(); triggerSend()
            }
            cmd.isStbdCmd && cmd.isMotorDown1    -> {
                starboardVal = (starboardVal - step1).coerceAtLeast(stopValue())
                refreshStbd(); triggerSend()
            }
            cmd.isStbdCmd && cmd.isMotorAbsolute -> {
                starboardVal = stopValue() + range * cmd.motorAbsPct / 100
                refreshStbd(); triggerSend()
            }
        }
    }
}