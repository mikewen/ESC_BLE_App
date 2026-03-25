package com.escbleapp

import android.annotation.SuppressLint
import android.content.Context
import android.content.SharedPreferences
import android.os.Bundle
import android.os.Handler
import android.os.Looper
import android.util.Log
import androidx.appcompat.app.AppCompatActivity
import com.escbleapp.databinding.ActivityCalibrationBinding

/**
 * CalibrationActivity — dock-side sensor calibration.
 *
 * MMC5603 Hard-Iron Calibration:
 *   Rotate the boat 360° slowly in calm water.
 *   Collects raw mx/my samples, computes min/max circle → hard-iron offsets.
 *   Progress ring fills as heading coverage increases (36 × 10° sectors).
 *
 * QMI8658C Calibration:
 *   Level cal: place sensor on level surface → accel bias (gravity reference).
 *   Gyro bias: hold still 5 seconds → gyro zero-rate offset.
 *
 * Results saved to SharedPreferences("calibration_prefs").
 * SensorFusion loads them on next connection.
 *
 * Accessible from MainActivity after BLE scan results.
 */
class CalibrationActivity : AppCompatActivity() {

    private lateinit var binding: ActivityCalibrationBinding
    private lateinit var prefs:   SharedPreferences
    private val handler = Handler(Looper.getMainLooper())

    // Live SensorFusion instance to receive A1 packets during calibration
    private lateinit var fusion: SensorFusion

    // ── Mag calibration state ─────────────────────────────────────────────────

    private val sectorCoverage = BooleanArray(36)   // 36 sectors × 10° each
    private var magCalActive   = false
    private var samplesCollected = 0

    // ── Gyro bias state ───────────────────────────────────────────────────────

    private var gyroCalActive  = false
    private val GYRO_CAL_SECS  = 5
    private var gyroCountdown  = GYRO_CAL_SECS

    // ── Lifecycle ─────────────────────────────────────────────────────────────

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        binding = ActivityCalibrationBinding.inflate(layoutInflater)
        setContentView(binding.root)
        prefs  = getSharedPreferences("calibration_prefs", Context.MODE_PRIVATE)
        fusion = SensorFusion()

        setupFusion()
        loadAndDisplaySaved()
        setupButtons()
    }

    override fun onDestroy() {
        super.onDestroy()
        handler.removeCallbacksAndMessages(null)
    }

    // ── SensorFusion wiring ───────────────────────────────────────────────────

    private fun setupFusion() {
        fusion.onFusedHeading = { fs ->
            runOnUiThread {
                if (magCalActive) updateMagProgress(fs.headingDeg)
            }
        }
    }

    /** Called from MainActivity/BLE when A1 packets arrive during calibration */
    fun feedA1Packet(b: ByteArray) {
        if (b.size < 20) return
        val ax = getI2(b, 2); val ay = getI2(b, 4); val az = getI2(b, 6)
        val gx = getI2(b, 8); val gy = getI2(b, 10); val gz = getI2(b, 12)
        val mx = getI2(b, 14); val my = getI2(b, 16); val mz = getI2(b, 18)

        if (magCalActive) {
            fusion.feedManualMagSample(mx, my)
            samplesCollected++
        }
        if (gyroCalActive) {
            fusion.feedGyroBiasSample(gx, gy, gz)
        }
        fusion.processA1(ax, ay, az, gx, gy, gz, mx, my, mz, System.currentTimeMillis())
    }

    // ── MMC5603 Calibration ───────────────────────────────────────────────────

    @SuppressLint("SetTextI18n")
    private fun updateMagProgress(currentHeading: Float) {
        val sector = (currentHeading / 10f).toInt().coerceIn(0, 35)
        sectorCoverage[sector] = true
        val covered = sectorCoverage.count { it }
        val pct     = covered * 100 / 36

        binding.progressMagCal.progress = pct
        binding.tvMagCalStatus.text     = "Rotating… $covered/36 sectors ($pct%)"
        samplesCollected++

        // Auto-finish when all sectors covered
        if (covered >= 36) finishMagCal()
    }

    private fun startMagCal() {
        sectorCoverage.fill(false)
        samplesCollected = 0
        magCalActive     = true
        fusion.startManualMagCal()

        binding.progressMagCal.progress = 0
        binding.tvMagCalStatus.text     = "Rotate boat 360° slowly…"
        binding.btnMagCalStart.isEnabled  = false
        binding.btnMagCalFinish.isEnabled = true
        binding.btnMagCalFinish.text      = "DONE (manual)"
    }

    @SuppressLint("SetTextI18n")
    private fun finishMagCal() {
        magCalActive = false
        binding.btnMagCalStart.isEnabled  = true
        binding.btnMagCalFinish.isEnabled = false

        val ok = fusion.finishManualMagCal()
        if (ok) {
            prefs.edit()
                .putFloat("mag_hard_iron_x", fusion.manualCalHardIronX)
                .putFloat("mag_hard_iron_y", fusion.manualCalHardIronY)
                .putBoolean("mag_cal_done", true)
                .apply()
            binding.tvMagCalStatus.text =
                "✓ Saved  X=${"%.1f".format(fusion.manualCalHardIronX)}  " +
                "Y=${"%.1f".format(fusion.manualCalHardIronY)}"
            binding.tvMagCalSaved.text  = "Hard-iron: X=${"%.1f".format(fusion.manualCalHardIronX)}  Y=${"%.1f".format(fusion.manualCalHardIronY)}"
        } else {
            binding.tvMagCalStatus.text = "⚠ Not enough coverage — rotate more"
        }
    }

    // ── QMI8658C Gyro Bias Calibration ───────────────────────────────────────

    private fun startGyroCal() {
        gyroCalActive = true
        gyroCountdown = GYRO_CAL_SECS
        fusion.startGyroBiasCal()
        binding.btnGyroCalStart.isEnabled = false
        binding.tvGyroCalStatus.text      = "Hold still… $gyroCountdown s"

        handler.postDelayed(object : Runnable {
            override fun run() {
                gyroCountdown--
                binding.tvGyroCalStatus.text = "Hold still… $gyroCountdown s"
                if (gyroCountdown > 0) handler.postDelayed(this, 1_000)
                else finishGyroCal()
            }
        }, 1_000)
    }

    @SuppressLint("SetTextI18n")
    private fun finishGyroCal() {
        gyroCalActive = false
        val ok = fusion.finishGyroBiasCal()
        binding.btnGyroCalStart.isEnabled = true
        if (ok) {
            prefs.edit()
                .putFloat("gyro_bias_x", fusion.gyroBiasX)
                .putFloat("gyro_bias_y", fusion.gyroBiasY)
                .putFloat("gyro_bias_z", fusion.gyroBiasZ)
                .putBoolean("gyro_cal_done", true)
                .apply()
            binding.tvGyroCalStatus.text =
                "✓ Saved  Z-bias=${"%.2f".format(fusion.gyroBiasZ)} LSB"
            binding.tvGyroCalSaved.text  =
                "Gyro bias  X=${"%.1f".format(fusion.gyroBiasX)}  Y=${"%.1f".format(fusion.gyroBiasY)}  Z=${"%.1f".format(fusion.gyroBiasZ)}"
        } else {
            binding.tvGyroCalStatus.text = "⚠ Too few samples — try again"
        }
    }

    // ── Load saved values ─────────────────────────────────────────────────────

    @SuppressLint("SetTextI18n")
    private fun loadAndDisplaySaved() {
        if (prefs.getBoolean("mag_cal_done", false)) {
            val x = prefs.getFloat("mag_hard_iron_x", 0f)
            val y = prefs.getFloat("mag_hard_iron_y", 0f)
            binding.tvMagCalSaved.text = "Hard-iron: X=${"%.1f".format(x)}  Y=${"%.1f".format(y)}"
            binding.tvMagCalStatus.text = "✓ Previously calibrated"
        } else {
            binding.tvMagCalSaved.text  = "Not calibrated"
            binding.tvMagCalStatus.text = "Tap START then rotate boat 360°"
        }

        if (prefs.getBoolean("gyro_cal_done", false)) {
            val z = prefs.getFloat("gyro_bias_z", 0f)
            binding.tvGyroCalSaved.text  = "Gyro bias Z=${"%.1f".format(z)}"
            binding.tvGyroCalStatus.text = "✓ Previously calibrated"
        } else {
            binding.tvGyroCalSaved.text  = "Not calibrated"
            binding.tvGyroCalStatus.text = "Place on level surface and tap START"
        }
    }

    // ── Buttons ───────────────────────────────────────────────────────────────

    private fun setupButtons() {
        binding.btnMagCalStart.setOnClickListener  { startMagCal() }
        binding.btnMagCalFinish.setOnClickListener { finishMagCal() }
        binding.btnGyroCalStart.setOnClickListener { startGyroCal() }
        binding.btnBack.setOnClickListener         { finish() }

        binding.btnMagCalClear.setOnClickListener {
            prefs.edit().remove("mag_hard_iron_x").remove("mag_hard_iron_y")
                .putBoolean("mag_cal_done", false).apply()
            binding.tvMagCalSaved.text  = "Cleared"
            binding.tvMagCalStatus.text = "Tap START to recalibrate"
        }
        binding.btnGyroCalClear.setOnClickListener {
            prefs.edit().remove("gyro_bias_x").remove("gyro_bias_y").remove("gyro_bias_z")
                .putBoolean("gyro_cal_done", false).apply()
            binding.tvGyroCalSaved.text  = "Cleared"
            binding.tvGyroCalStatus.text = "Tap START to recalibrate"
        }

        binding.btnMagCalFinish.isEnabled = false
    }

    // ── Binary helpers ────────────────────────────────────────────────────────

    private fun getI2(b: ByteArray, o: Int): Short =
        ((b[o].toInt() and 0xFF) or ((b[o+1].toInt() and 0xFF) shl 8)).toShort()

    companion object {
        /** Load calibration from prefs into a SensorFusion instance */
        fun loadCalibration(prefs: SharedPreferences, fusion: SensorFusion) {
            if (prefs.getBoolean("mag_cal_done", false)) {
                fusion.manualCalHardIronX = prefs.getFloat("mag_hard_iron_x", 0f)
                fusion.manualCalHardIronY = prefs.getFloat("mag_hard_iron_y", 0f)
                Log.i("CalibrationActivity",
                    "Mag cal loaded: X=${fusion.manualCalHardIronX} Y=${fusion.manualCalHardIronY}")
            }
            if (prefs.getBoolean("gyro_cal_done", false)) {
                fusion.gyroBiasX = prefs.getFloat("gyro_bias_x", 0f)
                fusion.gyroBiasY = prefs.getFloat("gyro_bias_y", 0f)
                fusion.gyroBiasZ = prefs.getFloat("gyro_bias_z", 0f)
                Log.i("CalibrationActivity", "Gyro cal loaded: Z=${fusion.gyroBiasZ}")
            }
        }
    }
}
