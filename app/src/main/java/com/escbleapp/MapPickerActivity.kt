package com.escbleapp

import android.annotation.SuppressLint
import android.content.Intent
import android.os.Bundle
import android.preference.PreferenceManager
import android.view.View
import androidx.appcompat.app.AlertDialog
import androidx.appcompat.app.AppCompatActivity
import com.escbleapp.databinding.ActivityMapPickerBinding
import com.google.android.material.chip.Chip
import org.json.JSONArray
import org.json.JSONObject
import org.osmdroid.config.Configuration
import org.osmdroid.events.MapEventsReceiver
import org.osmdroid.tileprovider.tilesource.OnlineTileSourceBase
import org.osmdroid.tileprovider.tilesource.TileSourceFactory
import org.osmdroid.util.GeoPoint
import org.osmdroid.util.MapTileIndex
import org.osmdroid.views.overlay.MapEventsOverlay
import org.osmdroid.views.overlay.Marker
import org.osmdroid.views.overlay.mylocation.GpsMyLocationProvider
import org.osmdroid.views.overlay.mylocation.MyLocationNewOverlay
import kotlin.math.*

/**
 * Full-screen map for picking an autopilot waypoint.
 *
 * Tap anywhere on the map to drop a target pin.
 * 💾 saves the current pin with a user-defined name.
 * Saved targets appear as chips — tap to restore, long-press to delete.
 * Targets persisted to SharedPreferences as JSON in key "saved_map_targets".
 */
class MapPickerActivity : AppCompatActivity() {

    private lateinit var binding: ActivityMapPickerBinding
    private var locationOverlay: MyLocationNewOverlay? = null
    private var targetMarker: Marker? = null
    private var targetGeoPoint: GeoPoint? = null
    private var currentLat = 0.0
    private var currentLon = 0.0
    private var hasCurrentLoc = false

    data class SavedTarget(val name: String, val lat: Double, val lon: Double)
    private val savedTargets = mutableListOf<SavedTarget>()

    companion object {
        const val RESULT_TARGET_BEARING = "result_bearing"
        const val RESULT_TARGET_LAT     = "result_lat"
        const val RESULT_TARGET_LON     = "result_lon"
        const val EXTRA_CURRENT_LAT     = "current_lat"
        const val EXTRA_CURRENT_LON     = "current_lon"
        const val EXTRA_CURRENT_HEADING = "current_heading"
        private const val PREFS_KEY     = "saved_map_targets"
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        Configuration.getInstance().load(this, PreferenceManager.getDefaultSharedPreferences(this))
        Configuration.getInstance().userAgentValue = packageName
        binding = ActivityMapPickerBinding.inflate(layoutInflater)
        setContentView(binding.root)

        currentLat    = intent.getDoubleExtra(EXTRA_CURRENT_LAT, 0.0)
        currentLon    = intent.getDoubleExtra(EXTRA_CURRENT_LON, 0.0)
        hasCurrentLoc = currentLat != 0.0 || currentLon != 0.0

        loadSavedTargets()
        setupMap()
        setupButtons()
        refreshChips()
    }

    // ── Persistence ───────────────────────────────────────────────────────────

    private fun loadSavedTargets() {
        val json = getSharedPreferences("map_prefs", MODE_PRIVATE)
            .getString(PREFS_KEY, "[]") ?: "[]"
        savedTargets.clear()
        try {
            val arr = JSONArray(json)
            for (i in 0 until arr.length()) {
                val o = arr.getJSONObject(i)
                savedTargets.add(SavedTarget(o.getString("name"), o.getDouble("lat"), o.getDouble("lon")))
            }
        } catch (_: Exception) {}
    }

    private fun persistTargets() {
        val arr = JSONArray()
        savedTargets.forEach { arr.put(JSONObject().put("name", it.name).put("lat", it.lat).put("lon", it.lon)) }
        getSharedPreferences("map_prefs", MODE_PRIVATE)
            .edit().putString(PREFS_KEY, arr.toString()).apply()
    }

    private fun promptSave() {
        val pt = targetGeoPoint ?: return
        val input = android.widget.EditText(this).apply {
            hint = "Target name"
            setSingleLine()
        }
        AlertDialog.Builder(this)
            .setTitle("Save target")
            .setView(input)
            .setPositiveButton("Save") { _, _ ->
                val name = input.text.toString().trim()
                    .ifEmpty { "%.4f, %.4f".format(pt.latitude, pt.longitude) }
                savedTargets.removeAll { it.name == name }   // replace if same name
                savedTargets.add(SavedTarget(name, pt.latitude, pt.longitude))
                persistTargets()
                refreshChips()
            }
            .setNegativeButton("Cancel", null)
            .show()
    }

    // ── Chips ─────────────────────────────────────────────────────────────────

    private fun refreshChips() {
        binding.chipGroupTargets.removeAllViews()
        binding.scrollSavedTargets.visibility =
            if (savedTargets.isEmpty()) View.GONE else View.VISIBLE

        savedTargets.forEachIndexed { i, target ->
            val chip = Chip(this).apply {
                text = target.name
                isClickable = true; isCheckable = false
                chipBackgroundColor = android.content.res.ColorStateList
                    .valueOf(android.graphics.Color.parseColor("#0D2233"))
                setTextColor(android.graphics.Color.parseColor("#88CCFF"))
                textSize = 11f

                setOnClickListener {
                    val pt = GeoPoint(target.lat, target.lon)
                    placeTargetMarker(pt)
                    binding.mapView.controller.animateTo(pt)
                }

                setOnLongClickListener {
                    AlertDialog.Builder(this@MapPickerActivity)
                        .setTitle("Delete \"${target.name}\"?")
                        .setPositiveButton("Delete") { _, _ ->
                            savedTargets.removeAt(i); persistTargets(); refreshChips()
                        }
                        .setNegativeButton("Cancel", null).show()
                    true
                }
            }
            binding.chipGroupTargets.addView(chip)
        }
    }

    // ── Map ───────────────────────────────────────────────────────────────────

    private var isSatellite = false

    private val ESRI_SATELLITE = object : OnlineTileSourceBase(
        "ESRI_Satellite", 0, 19, 256, ".jpg",
        arrayOf("https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/")
    ) {
        override fun getTileURLString(pMapTileIndex: Long) =
            "${baseUrl}${MapTileIndex.getZoom(pMapTileIndex)}/${MapTileIndex.getY(pMapTileIndex)}/${MapTileIndex.getX(pMapTileIndex)}.jpg"
    }

    private fun setupMap() {
        val map = binding.mapView
        map.setTileSource(TileSourceFactory.MAPNIK)
        map.setMultiTouchControls(true)
        val start = if (hasCurrentLoc) GeoPoint(currentLat, currentLon) else GeoPoint(0.0, 0.0)
        map.controller.setZoom(if (hasCurrentLoc) 14.0 else 3.0)
        map.controller.setCenter(start)
        locationOverlay = MyLocationNewOverlay(GpsMyLocationProvider(this), map).apply { enableMyLocation() }
        map.overlays.add(locationOverlay)
        map.overlays.add(MapEventsOverlay(object : MapEventsReceiver {
            override fun singleTapConfirmedHelper(p: GeoPoint): Boolean { placeTargetMarker(p); return true }
            override fun longPressHelper(p: GeoPoint) = false
        }))
    }

    private fun toggleSatellite() {
        isSatellite = !isSatellite
        binding.mapView.setTileSource(if (isSatellite) ESRI_SATELLITE else TileSourceFactory.MAPNIK)
        binding.mapView.invalidate()
        binding.btnMapLayer.text = if (isSatellite) "🗺 MAP" else "🛰 SAT"
    }

    @SuppressLint("SetTextI18n")
    private fun placeTargetMarker(point: GeoPoint) {
        val map = binding.mapView
        targetMarker?.let { map.overlays.remove(it) }
        targetMarker = Marker(map).apply {
            position = point; title = "Target"
            setAnchor(Marker.ANCHOR_CENTER, Marker.ANCHOR_BOTTOM)
        }
        map.overlays.add(targetMarker)
        map.invalidate()
        targetGeoPoint = point

        if (hasCurrentLoc) {
            val bearing = bearingTo(currentLat, currentLon, point.latitude, point.longitude)
            val distNm  = haversineNm(currentLat, currentLon, point.latitude, point.longitude)
            binding.tvMapBearing.text = "%.0f°".format(bearing)
            binding.tvMapCoords.text  =
                "%.5f, %.5f   → %.0f°  %.2f nm".format(point.latitude, point.longitude, bearing, distNm)
        } else {
            binding.tvMapBearing.text = ""
            binding.tvMapCoords.text  = "%.5f, %.5f".format(point.latitude, point.longitude)
        }

        binding.tvMapInstruction.text   = "Target set — tap SET TARGET or 💾"
        binding.btnMapConfirm.isEnabled = true
        binding.btnSaveTarget.isEnabled = true
    }

    private fun setupButtons() {
        binding.btnMapCancel.setOnClickListener  { setResult(RESULT_CANCELED); finish() }
        binding.btnMapLayer.setOnClickListener   { toggleSatellite() }
        binding.btnSaveTarget.setOnClickListener { promptSave() }

        binding.btnMapConfirm.setOnClickListener {
            val pt = targetGeoPoint ?: return@setOnClickListener
            val bearing = if (hasCurrentLoc)
                bearingTo(currentLat, currentLon, pt.latitude, pt.longitude) else 0f
            setResult(RESULT_OK, Intent().apply {
                putExtra(RESULT_TARGET_BEARING, bearing)
                putExtra(RESULT_TARGET_LAT,     pt.latitude)
                putExtra(RESULT_TARGET_LON,     pt.longitude)
            })
            finish()
        }
    }

    override fun onResume()  { super.onResume();  binding.mapView.onResume()  }
    override fun onPause()   { super.onPause();   binding.mapView.onPause()   }
    override fun onDestroy() { super.onDestroy(); locationOverlay?.disableMyLocation() }

    // ── Geo math ──────────────────────────────────────────────────────────────

    private fun bearingTo(lat1: Double, lon1: Double, lat2: Double, lon2: Double): Float {
        val φ1 = Math.toRadians(lat1); val φ2 = Math.toRadians(lat2)
        val Δλ = Math.toRadians(lon2 - lon1)
        return ((Math.toDegrees(atan2(sin(Δλ) * cos(φ2),
            cos(φ1) * sin(φ2) - sin(φ1) * cos(φ2) * cos(Δλ))) + 360) % 360).toFloat()
    }

    private fun haversineNm(lat1: Double, lon1: Double, lat2: Double, lon2: Double): Double {
        val R = 3440.065
        val dLat = Math.toRadians(lat2 - lat1); val dLon = Math.toRadians(lon2 - lon1)
        val a = sin(dLat / 2).pow(2) +
                cos(Math.toRadians(lat1)) * cos(Math.toRadians(lat2)) * sin(dLon / 2).pow(2)
        return R * 2 * asin(sqrt(a))
    }
}