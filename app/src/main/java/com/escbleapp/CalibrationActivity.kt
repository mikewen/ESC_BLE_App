package com.escbleapp

import android.Manifest
import android.annotation.SuppressLint
import android.bluetooth.BluetoothAdapter
import android.bluetooth.BluetoothDevice
import android.bluetooth.BluetoothManager
import android.bluetooth.le.BluetoothLeScanner
import android.bluetooth.le.ScanCallback
import android.bluetooth.le.ScanResult
import android.bluetooth.le.ScanSettings
import android.content.Context
import android.content.SharedPreferences
import android.content.pm.PackageManager
import android.os.Build
import android.os.Bundle
import android.os.Handler
import android.os.Looper
import android.util.Log
import android.view.View
import android.widget.Toast
import androidx.activity.result.contract.ActivityResultContracts
import androidx.appcompat.app.AppCompatActivity
import androidx.core.content.ContextCompat
import com.escbleapp.databinding.ActivityCalibrationBinding
import no.nordicsemi.android.ble.observer.ConnectionObserver

/**
 * CalibrationActivity — dock-side sensor calibration with its own BLE connection.
 *
 * Opens independently from motor control — scans for and connects to the
 * AC6329C sensor device to receive A1 packets for calibration.
 *
 * MMC5603 Hard-Iron Calibration:
 *   Rotate boat 360° slowly → collect raw mx/my → compute hard-iron offsets.
 *
 * QMI8658C Gyro Bias Calibration:
 *   Hold still 5 seconds → gyro zero-rate offsets.
 *
 * Results saved to SharedPreferences("calibration_prefs").
 */
class CalibrationActivity : AppCompatActivity() {

    private lateinit var binding: ActivityCalibrationBinding
    private lateinit var prefs:   SharedPreferences
    private val handler = Handler(Looper.getMainLooper())

    // ── BLE ───────────────────────────────────────────────────────────────────
    private var bleManager:    AC6328BleManager? = null
    private var bleScanner:    BluetoothLeScanner? = null
    private var isScanning     = false
    private var isConnected    = false
    private val SCAN_PERIOD_MS = 10_000L

    private val btAdapter: BluetoothAdapter? by lazy {
        (getSystemService(Context.BLUETOOTH_SERVICE) as BluetoothManager).adapter
    }

    private val permLauncher = registerForActivityResult(
        ActivityResultContracts.RequestMultiplePermissions()
    ) { grants ->
        if (grants.values.all { it }) startScan()
        else showToast("Bluetooth/Location permission required for BLE scan")
    }

    // ── Fusion engine for calibration ─────────────────────────────────────────
    private val fusion = SensorFusion()

    // ── Mag calibration state ─────────────────────────────────────────────────
    private val sectorCoverage = BooleanArray(36)   // 36 × 10° sectors
    private var magCalActive   = false

    // ── Gyro bias state ───────────────────────────────────────────────────────
    private var gyroCalActive = false
    private val GYRO_CAL_SECS = 5
    private var gyroCountdown = GYRO_CAL_SECS
    private var accumulatedGyroZ = 0f
    private var lastA1TimeMs = 0L

    // ── Lifecycle ─────────────────────────────────────────────────────────────

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        binding = ActivityCalibrationBinding.inflate(layoutInflater)
        setContentView(binding.root)
        prefs = getSharedPreferences("calibration_prefs", Context.MODE_PRIVATE)

        setupFusion()
        loadAndDisplaySaved()
        setupButtons()
        updateBleStatus("Tap SCAN to connect sensor")
    }

    override fun onDestroy() {
        super.onDestroy()
        handler.removeCallbacksAndMessages(null)
        stopScan()
        bleManager?.disconnect()?.enqueue()
        bleManager?.close()
    }

    // ── BLE scan ──────────────────────────────────────────────────────────────

    private fun checkPermissionsAndScan() {
        val perms = if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S)
            arrayOf(Manifest.permission.BLUETOOTH_SCAN, Manifest.permission.BLUETOOTH_CONNECT,
                Manifest.permission.ACCESS_FINE_LOCATION)
        else
            arrayOf(Manifest.permission.ACCESS_FINE_LOCATION)

        val missing = perms.filter {
            ContextCompat.checkSelfPermission(this, it) != PackageManager.PERMISSION_GRANTED
        }
        if (missing.isEmpty()) startScan() else permLauncher.launch(missing.toTypedArray())
    }

    @SuppressLint("MissingPermission")
    private fun startScan() {
        if (isScanning || isConnected) return
        val adapter = btAdapter ?: run { showToast("Bluetooth not available"); return }
        bleScanner = adapter.bluetoothLeScanner
        isScanning = true
        updateBleStatus("Scanning…")
        binding.btnBleScan.text = "STOP"
        //binding.rvDevices.visibility = View.VISIBLE

        bleScanner?.startScan(null,
            ScanSettings.Builder().setScanMode(ScanSettings.SCAN_MODE_LOW_LATENCY).build(),
            scanCallback)

        handler.postDelayed({ stopScan() }, SCAN_PERIOD_MS)
    }

    @SuppressLint("MissingPermission")
    private fun stopScan() {
        if (!isScanning) return
        isScanning = false
        bleScanner?.stopScan(scanCallback)
        binding.btnBleScan.text = "SCAN"
        if (!isConnected) updateBleStatus("Scan complete — tap a device to connect")
    }

    private val foundDevices = mutableListOf<Pair<String, BluetoothDevice>>()

    @SuppressLint("MissingPermission")
    private val scanCallback = object : ScanCallback() {
        override fun onScanResult(callbackType: Int, result: ScanResult) {
            val name = result.device.name ?: return
            if (foundDevices.none { it.second.address == result.device.address }) {
                foundDevices.add(Pair(name, result.device))
                runOnUiThread { updateDeviceList() }
            }
        }
    }

    @SuppressLint("SetTextI18n")
    private fun updateDeviceList() {
        // Build a simple text list — user taps btnConnectDevice to connect first found AC6329
        val sb = StringBuilder()
        foundDevices.forEachIndexed { i, (name, dev) ->
            sb.append("${i + 1}. $name  ${dev.address}\n")
        }
        binding.tvScanResults.text = sb.toString()
        // Auto-connect to first AC6329/GPS/IMU device found
        val sensor = foundDevices.firstOrNull { (name, _) ->
            name.contains("AC6329", true) || name.contains("GPS_PWM", true) ||
                    name.contains("IMU_PWM", true) || name.contains("ESC_PWM", true) ||
                    name.contains("BLDC_PWM", true)
        }
        if (sensor != null) {
            stopScan()
            connectSensor(sensor.second, sensor.first)
        }
    }

    // ── BLE connect ───────────────────────────────────────────────────────────

    private fun connectSensor(device: BluetoothDevice, name: String) {
        updateBleStatus("Connecting $name…")
        bleManager = AC6328BleManager(this)

        bleManager!!.setConnectionObserver(object : ConnectionObserver {
            override fun onDeviceConnecting(d: BluetoothDevice)    = Unit
            override fun onDeviceDisconnecting(d: BluetoothDevice) = Unit
            override fun onDeviceReady(d: BluetoothDevice)         = Unit
            override fun onDeviceConnected(d: BluetoothDevice) = runOnUiThread {
                isConnected = true
                updateBleStatus("✓ Connected: $name — ready to calibrate")
                binding.btnBleScan.text = "DISCONNECT"
            }
            override fun onDeviceFailedToConnect(d: BluetoothDevice, reason: Int) = runOnUiThread {
                isConnected = false
                updateBleStatus("Connection failed (code $reason) — try SCAN again")
                binding.btnBleScan.text = "SCAN"
            }
            override fun onDeviceDisconnected(d: BluetoothDevice, reason: Int) = runOnUiThread {
                isConnected = false
                updateBleStatus("Disconnected — tap SCAN to reconnect")
                binding.btnBleScan.text = "SCAN"
            }
        })

        // A1 packets feed the fusion engine for calibration
        bleManager!!.onAe02Raw = { bytes -> feedA1Packet(bytes) }
        bleManager!!.connectToDevice(device)
    }

    // ── Feed A1 packets ───────────────────────────────────────────────────────

    private fun feedA1Packet(bytes: ByteArray) {
        if (bytes.isEmpty() || bytes[0] != 0xA1.toByte() || bytes.size < 20) return
        val ax = getI2(bytes, 2); val ay = getI2(bytes, 4); val az = getI2(bytes, 6)
        val gx = getI2(bytes, 8); val gy = getI2(bytes, 10); val gz = getI2(bytes, 12)
        val mx = getI2(bytes, 14); val my = getI2(bytes, 16); val mz = getI2(bytes, 18)

        val now = System.currentTimeMillis()
        val dt = if (lastA1TimeMs > 0) (now - lastA1TimeMs) / 1000f else 0.02f
        lastA1TimeMs = now

        // Integrate gyro Z for display (testing scale/direction)
        val gzCorrected = gz - fusion.gyroBiasZ
        val gzDegS = gzCorrected * fusion.gyroScaleDegS * (if (fusion.gyroZFlipped) -1f else 1f)
        accumulatedGyroZ += gzDegS * dt
        runOnUiThread {
            binding.tvGyroRotateDeg.text = "Rotate: ${"%.1f".format(accumulatedGyroZ)}°"
        }

        if (magCalActive) {
            fusion.feedManualMagSample(mx, my)
            val heading = fusion.getState().headingDeg
            runOnUiThread { updateMagProgress(heading) }
        }
        if (gyroCalActive) {
            fusion.feedGyroBiasSample(gx, gy, gz)
        }
        fusion.processA1(ax, ay, az, gx, gy, gz, mx, my, mz, now)
    }

    // ── MMC5603 calibration ───────────────────────────────────────────────────

    @SuppressLint("SetTextI18n")
    private fun updateMagProgress(currentHeading: Float) {
        if (!magCalActive) return
        val sector = (currentHeading / 10f).toInt().coerceIn(0, 35)
        sectorCoverage[sector] = true
        val covered = sectorCoverage.count { it }
        val pct = covered * 100 / 36
        binding.progressMagCal.progress = pct
        binding.tvMagCalStatus.text = "Rotating… $covered/36 sectors ($pct%)"
        if (covered >= 36) finishMagCal()
    }

    private fun startMagCal() {
        if (!isConnected) { showToast("Connect sensor first"); return }
        sectorCoverage.fill(false)
        magCalActive = true
        fusion.startManualMagCal()
        binding.progressMagCal.progress = 0
        binding.tvMagCalStatus.text = "Rotate boat slowly 360°…"
        binding.btnMagCalStart.isEnabled  = false
        binding.btnMagCalFinish.isEnabled = true
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
                .putBoolean("mag_cal_done", true).apply()
            binding.tvMagCalStatus.text = "✓ Saved  X=${"%.1f".format(fusion.manualCalHardIronX)}  Y=${"%.1f".format(fusion.manualCalHardIronY)}"
            binding.tvMagCalSaved.text  = "Hard-iron: X=${"%.1f".format(fusion.manualCalHardIronX)}  Y=${"%.1f".format(fusion.manualCalHardIronY)}"
        } else {
            binding.tvMagCalStatus.text = "⚠ Not enough coverage — rotate more"
        }
    }

    // ── QMI8658C gyro bias calibration ────────────────────────────────────────

    private fun startGyroCal() {
        if (!isConnected) { showToast("Connect sensor first"); return }
        gyroCalActive = true
        gyroCountdown = GYRO_CAL_SECS
        fusion.startGyroBiasCal()
        binding.btnGyroCalStart.isEnabled = false
        binding.tvGyroCalStatus.text = "Hold still… $gyroCountdown s"
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
        binding.btnGyroCalStart.isEnabled = true
        val ok = fusion.finishGyroBiasCal()
        if (ok) {
            prefs.edit()
                .putFloat("gyro_bias_x", fusion.gyroBiasX)
                .putFloat("gyro_bias_y", fusion.gyroBiasY)
                .putFloat("gyro_bias_z", fusion.gyroBiasZ)
                .putBoolean("gyro_cal_done", true).apply()
            binding.tvGyroCalStatus.text = "✓ Saved  Z=${"%.2f".format(fusion.gyroBiasZ)} LSB"
            binding.tvGyroCalSaved.text  = "Gyro bias  X=${"%.1f".format(fusion.gyroBiasX)}  Y=${"%.1f".format(fusion.gyroBiasY)}  Z=${"%.1f".format(fusion.gyroBiasZ)}"
        } else {
            binding.tvGyroCalStatus.text = "⚠ Too few samples — try again"
        }
    }

    // ── Load saved calibration ────────────────────────────────────────────────

    @SuppressLint("SetTextI18n")
    private fun loadAndDisplaySaved() {
        if (prefs.getBoolean("mag_cal_done", false)) {
            val x = prefs.getFloat("mag_hard_iron_x", 0f)
            val y = prefs.getFloat("mag_hard_iron_y", 0f)
            binding.tvMagCalSaved.text  = "Hard-iron: X=${"%.1f".format(x)}  Y=${"%.1f".format(y)}"
            binding.tvMagCalStatus.text = "✓ Previously calibrated"
        } else {
            binding.tvMagCalSaved.text  = "Not calibrated"
            binding.tvMagCalStatus.text = "Connect sensor then tap START"
        }
        if (prefs.getBoolean("gyro_cal_done", false)) {
            val z = prefs.getFloat("gyro_bias_z", 0f)
            binding.tvGyroCalSaved.text  = "Gyro bias Z=${"%.1f".format(z)}"
            binding.tvGyroCalStatus.text = "✓ Previously calibrated"
        } else {
            binding.tvGyroCalSaved.text  = "Not calibrated"
            binding.tvGyroCalStatus.text = "Connect sensor then tap START"
        }
    }

    // ── SensorFusion wiring ───────────────────────────────────────────────────

    private fun setupFusion() {
        CalibrationActivity.loadCalibration(prefs, fusion)
    }

    // ── Buttons ───────────────────────────────────────────────────────────────

    @SuppressLint("MissingPermission")
    private fun setupButtons() {
        binding.btnBleScan.setOnClickListener {
            when {
                isConnected -> {
                    bleManager?.disconnect()?.enqueue()
                    binding.btnBleScan.text = "SCAN"
                }
                isScanning  -> stopScan()
                else        -> {
                    foundDevices.clear()
                    binding.tvScanResults.text = ""
                    checkPermissionsAndScan()
                }
            }
        }

        binding.btnMagCalStart.setOnClickListener  { startMagCal() }
        binding.btnMagCalFinish.setOnClickListener { finishMagCal() }
        binding.btnGyroCalStart.setOnClickListener { startGyroCal() }
        binding.btnBack.setOnClickListener         { finish() }

        binding.btnMagCalClear.setOnClickListener {
            prefs.edit().remove("mag_hard_iron_x").remove("mag_hard_iron_y")
                .putBoolean("mag_cal_done", false).apply()
            binding.tvMagCalSaved.text  = "Cleared"
            binding.tvMagCalStatus.text = "Connect sensor then tap START"
        }
        binding.btnGyroCalClear.setOnClickListener {
            prefs.edit().remove("gyro_bias_x").remove("gyro_bias_y").remove("gyro_bias_z")
                .putBoolean("gyro_cal_done", false).apply()
            binding.tvGyroCalSaved.text  = "Cleared"
            binding.tvGyroCalStatus.text = "Connect sensor then tap START"
            accumulatedGyroZ = 0f
            binding.tvGyroRotateDeg.text = "Rotate: 0.0°"
        }

        binding.btnMagCalFinish.isEnabled = false
        //binding.rvDevices.visibility = View.GONE
    }

    // ── Helpers ───────────────────────────────────────────────────────────────

    private fun updateBleStatus(msg: String) {
        binding.tvBleStatus.text = msg
    }

    private fun showToast(msg: String) =
        Toast.makeText(this, msg, Toast.LENGTH_SHORT).show()

    private fun getI2(b: ByteArray, o: Int): Short =
        ((b[o].toInt() and 0xFF) or ((b[o+1].toInt() and 0xFF) shl 8)).toShort()

    companion object {
        fun loadCalibration(prefs: SharedPreferences, fusion: SensorFusion) {
            if (prefs.getBoolean("mag_cal_done", false)) {
                fusion.manualCalHardIronX = prefs.getFloat("mag_hard_iron_x", 0f)
                fusion.manualCalHardIronY = prefs.getFloat("mag_hard_iron_y", 0f)
                Log.i("CalibrationActivity", "Mag cal loaded: X=${fusion.manualCalHardIronX} Y=${fusion.manualCalHardIronY}")
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