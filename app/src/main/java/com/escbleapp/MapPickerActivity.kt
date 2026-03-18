package com.escbleapp

import android.annotation.SuppressLint
import android.content.Intent
import android.os.Bundle
import android.preference.PreferenceManager
import android.view.MotionEvent
import androidx.appcompat.app.AppCompatActivity
import com.escbleapp.databinding.ActivityMapPickerBinding
import org.osmdroid.config.Configuration
import org.osmdroid.events.MapEventsReceiver
import org.osmdroid.tileprovider.tilesource.TileSourceFactory
import org.osmdroid.util.GeoPoint
import org.osmdroid.views.overlay.MapEventsOverlay
import org.osmdroid.views.overlay.Marker
import org.osmdroid.views.overlay.mylocation.GpsMyLocationProvider
import org.osmdroid.views.overlay.mylocation.MyLocationNewOverlay
import kotlin.math.*

/**
 * Full-screen map for picking an autopilot waypoint.
 *
 * Uses OSMDroid (OpenStreetMap) — no API key required.
 * Tap anywhere on the map to drop a target pin.
 * Calculates bearing from current position to target.
 * Returns: RESULT_TARGET_BEARING (float), RESULT_TARGET_LAT, RESULT_TARGET_LON.
 */
class MapPickerActivity : AppCompatActivity() {

    private lateinit var binding: ActivityMapPickerBinding
    private var locationOverlay: MyLocationNewOverlay? = null
    private var targetMarker: Marker? = null
    private var targetGeoPoint: GeoPoint? = null
    private var currentLat = 0.0
    private var currentLon = 0.0
    private var hasCurrentLoc = false

    companion object {
        const val RESULT_TARGET_BEARING = "result_bearing"
        const val RESULT_TARGET_LAT     = "result_lat"
        const val RESULT_TARGET_LON     = "result_lon"
        const val EXTRA_CURRENT_LAT     = "current_lat"
        const val EXTRA_CURRENT_LON     = "current_lon"
        const val EXTRA_CURRENT_HEADING = "current_heading"
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)

        // OSMDroid config — must be before setContentView
        Configuration.getInstance().load(this,
            PreferenceManager.getDefaultSharedPreferences(this))
        Configuration.getInstance().userAgentValue = packageName

        binding = ActivityMapPickerBinding.inflate(layoutInflater)
        setContentView(binding.root)

        // Starting position from intent
        currentLat  = intent.getDoubleExtra(EXTRA_CURRENT_LAT, 0.0)
        currentLon  = intent.getDoubleExtra(EXTRA_CURRENT_LON, 0.0)
        hasCurrentLoc = currentLat != 0.0 || currentLon != 0.0

        setupMap()
        setupButtons()
    }

    private fun setupMap() {
        val map = binding.mapView
        map.setTileSource(TileSourceFactory.MAPNIK)
        map.setMultiTouchControls(true)

        // Start centred on current location or world view
        val startPoint = if (hasCurrentLoc)
            GeoPoint(currentLat, currentLon) else GeoPoint(0.0, 0.0)
        map.controller.setZoom(if (hasCurrentLoc) 14.0 else 3.0)
        map.controller.setCenter(startPoint)

        // My location overlay
        locationOverlay = MyLocationNewOverlay(GpsMyLocationProvider(this), map).apply {
            enableMyLocation()
            enableFollowLocation()
        }
        map.overlays.add(locationOverlay)

        // Tap overlay — drop marker where user taps
        val eventsReceiver = object : MapEventsReceiver {
            override fun singleTapConfirmedHelper(p: GeoPoint): Boolean {
                placeTargetMarker(p)
                return true
            }
            override fun longPressHelper(p: GeoPoint) = false
        }
        map.overlays.add(MapEventsOverlay(eventsReceiver))
    }

    @SuppressLint("SetTextI18n")
    private fun placeTargetMarker(point: GeoPoint) {
        val map = binding.mapView

        // Remove old marker
        targetMarker?.let { map.overlays.remove(it) }

        // Add new marker
        targetMarker = Marker(map).apply {
            position  = point
            title     = "Target"
            setAnchor(Marker.ANCHOR_CENTER, Marker.ANCHOR_BOTTOM)
        }
        map.overlays.add(targetMarker)
        map.invalidate()

        targetGeoPoint = point

        // Calculate bearing from current position to target
        val bearingStr: String
        if (hasCurrentLoc) {
            val bearing = bearingTo(currentLat, currentLon, point.latitude, point.longitude)
            val distNm  = haversineNm(currentLat, currentLon, point.latitude, point.longitude)
            bearingStr  = "→ %.0f°  %.2f nm".format(bearing, distNm)
            binding.tvMapBearing.text = "%.0f°".format(bearing)
        } else {
            bearingStr = "No current position"
            binding.tvMapBearing.text = ""
        }

        binding.tvMapCoords.text =
            "%.5f, %.5f   $bearingStr".format(point.latitude, point.longitude)
        binding.tvMapInstruction.text = "Target set — confirm below"
        binding.btnMapConfirm.isEnabled = true
    }

    private fun setupButtons() {
        binding.btnMapCancel.setOnClickListener {
            setResult(RESULT_CANCELED)
            finish()
        }

        binding.btnMapConfirm.setOnClickListener {
            val pt = targetGeoPoint ?: return@setOnClickListener
            val bearing = if (hasCurrentLoc)
                bearingTo(currentLat, currentLon, pt.latitude, pt.longitude)
            else 0f

            val result = Intent().apply {
                putExtra(RESULT_TARGET_BEARING, bearing)
                putExtra(RESULT_TARGET_LAT,     pt.latitude)
                putExtra(RESULT_TARGET_LON,     pt.longitude)
            }
            setResult(RESULT_OK, result)
            finish()
        }
    }

    override fun onResume()  { super.onResume();  binding.mapView.onResume()  }
    override fun onPause()   { super.onPause();   binding.mapView.onPause()   }
    override fun onDestroy() { super.onDestroy(); locationOverlay?.disableMyLocation() }

    // ── Geo math ──────────────────────────────────────────────────────────────

    /** Forward azimuth (bearing) from point 1 to point 2, degrees 0–360 */
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
}
