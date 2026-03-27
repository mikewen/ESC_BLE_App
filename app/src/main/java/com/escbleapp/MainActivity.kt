package com.escbleapp

import android.Manifest
import android.annotation.SuppressLint
import android.bluetooth.BluetoothAdapter
import android.bluetooth.BluetoothManager
import android.bluetooth.le.BluetoothLeScanner
import android.bluetooth.le.ScanCallback
import android.bluetooth.le.ScanResult
import android.bluetooth.le.ScanSettings
import android.content.Context
import android.content.Intent
import android.content.pm.PackageManager
import android.os.Build
import android.os.Bundle
import android.os.Handler
import android.os.Looper
import android.util.Log
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import android.widget.Toast
import androidx.activity.result.contract.ActivityResultContracts
import androidx.appcompat.app.AppCompatActivity
import androidx.core.content.ContextCompat
import androidx.recyclerview.widget.LinearLayoutManager
import androidx.recyclerview.widget.RecyclerView
import com.escbleapp.databinding.ActivityMainBinding
import com.escbleapp.databinding.ItemDeviceBinding

data class BleDeviceItem(
    val name: String,
    val address: String,
    val rssi: Int,
    val device: android.bluetooth.BluetoothDevice
)

class MainActivity : AppCompatActivity() {

    private lateinit var binding: ActivityMainBinding
    private lateinit var adapter: DeviceAdapter

    private val bluetoothAdapter: BluetoothAdapter? by lazy {
        (getSystemService(Context.BLUETOOTH_SERVICE) as BluetoothManager).adapter
    }
    private var bleScanner: BluetoothLeScanner? = null
    private var scanning = false
    private val handler = Handler(Looper.getMainLooper())
    private val SCAN_PERIOD_MS = 15_000L

    private val devices = mutableListOf<BleDeviceItem>()
    private val seenAddresses = mutableSetOf<String>()

    // Permission launcher
    private val permissionLauncher = registerForActivityResult(
        ActivityResultContracts.RequestMultiplePermissions()
    ) { results ->
        if (results.values.all { it }) startScan()
        else Toast.makeText(this, "Bluetooth permissions required", Toast.LENGTH_LONG).show()
    }

    // Enable Bluetooth launcher
    private val enableBtLauncher = registerForActivityResult(
        ActivityResultContracts.StartActivityForResult()
    ) { if (it.resultCode == RESULT_OK) checkPermissionsAndScan() }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        binding = ActivityMainBinding.inflate(layoutInflater)
        setContentView(binding.root)

        adapter = DeviceAdapter(devices) { item ->
            stopScan()
            val intent = Intent(this, ControlActivity::class.java).apply {
                putExtra(ControlActivity.EXTRA_DEVICE, item.device)
                putExtra(ControlActivity.EXTRA_DEVICE_NAME, item.name)
            }
            startActivity(intent)
        }

        binding.recyclerView.layoutManager = LinearLayoutManager(this)
        binding.recyclerView.adapter = adapter

        binding.btnScan.setOnClickListener {
            if (scanning) stopScan() else checkPermissionsAndScan()
        }

        binding.btnCalibrate.setOnClickListener {
            startActivity(android.content.Intent(this, CalibrationActivity::class.java))
        }

        binding.chipFilterEsc.setOnCheckedChangeListener { _, _ -> /* filter applied in callback */ }
    }

    private fun checkPermissionsAndScan() {
        val required = if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
            arrayOf(
                Manifest.permission.BLUETOOTH_SCAN,
                Manifest.permission.BLUETOOTH_CONNECT
            )
        } else {
            arrayOf(
                Manifest.permission.BLUETOOTH,
                Manifest.permission.ACCESS_FINE_LOCATION
            )
        }

        val missing = required.filter {
            ContextCompat.checkSelfPermission(this, it) != PackageManager.PERMISSION_GRANTED
        }

        if (missing.isEmpty()) {
            val btAdapter = bluetoothAdapter
            if (btAdapter == null || !btAdapter.isEnabled) {
                enableBtLauncher.launch(Intent(BluetoothAdapter.ACTION_REQUEST_ENABLE))
                return
            }
            startScan()
        } else {
            permissionLauncher.launch(missing.toTypedArray())
        }
    }

    @SuppressLint("MissingPermission")
    private fun startScan() {
        bleScanner = bluetoothAdapter?.bluetoothLeScanner ?: return
        devices.clear()
        seenAddresses.clear()
        adapter.notifyDataSetChanged()

        scanning = true
        binding.btnScan.text = "Stop Scan"
        binding.progressBar.visibility = View.VISIBLE
        binding.tvEmpty.visibility = View.GONE

        val settings = ScanSettings.Builder()
            .setScanMode(ScanSettings.SCAN_MODE_LOW_LATENCY)
            .build()

        // The AC6328 advertises service UUID 0xAF30 in the adv packet
        // (adv data uses 0xAF30; GATT service is ae30 — firmware quirk)
        // We scan with NO filter and match by name, to catch both variants
        bleScanner?.startScan(null, settings, scanCallback)
        handler.postDelayed({ stopScan() }, SCAN_PERIOD_MS)

        Log.d(TAG, "BLE scan started")
    }

    @SuppressLint("MissingPermission")
    private fun stopScan() {
        if (!scanning) return
        scanning = false
        handler.removeCallbacksAndMessages(null)
        bleScanner?.stopScan(scanCallback)
        binding.btnScan.text = "Scan [ESC|BLDC|GPS|IMU]_PWM"
        binding.progressBar.visibility = View.GONE
        if (devices.isEmpty()) binding.tvEmpty.visibility = View.VISIBLE
        Log.d(TAG, "BLE scan stopped")
    }

    @SuppressLint("MissingPermission")
    private val scanCallback = object : ScanCallback() {
        override fun onScanResult(callbackType: Int, result: ScanResult) {
            val device = result.device
            val address = device.address
            if (address in seenAddresses) {
                // Update RSSI for existing entry
                val idx = devices.indexOfFirst { it.address == address }
                if (idx >= 0) {
                    devices[idx] = devices[idx].copy(rssi = result.rssi)
                    adapter.notifyItemChanged(idx)
                }
                return
            }
            seenAddresses.add(address)

            val name = device.name ?: result.scanRecord?.deviceName ?: "Unknown"
            // Show all devices if filter chip unchecked, else only ESC_PWM/BLDC_PWM/GPS_PWM
            if (binding.chipFilterEsc.isChecked &&
                !name.contains("ESC_PWM", true) &&
                !name.contains("BLDC_PWM", true) &&
                !name.contains("GPS_PWM", true) &&
                !name.contains("IMU_PWM", true) &&
                !name.contains("AC6328", true) &&
                !name.contains("AC6329", true)) {
                return
            }

            val item = BleDeviceItem(name, address, result.rssi, device)
            devices.add(item)
            // Sort by RSSI descending
            devices.sortByDescending { it.rssi }
            adapter.notifyDataSetChanged()
            binding.tvEmpty.visibility = View.GONE
        }

        override fun onScanFailed(errorCode: Int) {
            Log.e(TAG, "Scan failed: $errorCode")
            Toast.makeText(this@MainActivity, "Scan failed ($errorCode)", Toast.LENGTH_SHORT).show()
            stopScan()
        }
    }

    override fun onPause() {
        super.onPause()
        stopScan()
    }

    companion object {
        private const val TAG = "MainActivity"
    }
}

// ── RecyclerView Adapter ──────────────────────────────────────────────────────

class DeviceAdapter(
    private val items: List<BleDeviceItem>,
    private val onClick: (BleDeviceItem) -> Unit
) : RecyclerView.Adapter<DeviceAdapter.VH>() {

    inner class VH(val binding: ItemDeviceBinding) : RecyclerView.ViewHolder(binding.root)

    override fun onCreateViewHolder(parent: ViewGroup, viewType: Int): VH {
        val b = ItemDeviceBinding.inflate(LayoutInflater.from(parent.context), parent, false)
        return VH(b)
    }

    @SuppressLint("SetTextI18n")
    override fun onBindViewHolder(holder: VH, position: Int) {
        val item = items[position]
        holder.binding.tvName.text = item.name
        holder.binding.tvAddress.text = item.address
        holder.binding.tvRssi.text = "${item.rssi} dBm"
        holder.binding.ivSignal.setImageLevel(rssiToLevel(item.rssi))
        holder.itemView.setOnClickListener { onClick(item) }
    }

    override fun getItemCount() = items.size

    private fun rssiToLevel(rssi: Int) = when {
        rssi >= -60 -> 3
        rssi >= -75 -> 2
        rssi >= -85 -> 1
        else -> 0
    }
}