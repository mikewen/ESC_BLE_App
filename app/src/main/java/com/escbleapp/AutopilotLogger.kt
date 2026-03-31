package com.escbleapp

import android.os.Environment
import android.util.Log
import java.io.File
import java.io.PrintWriter
import java.io.FileWriter
import java.text.SimpleDateFormat
import java.util.Date
import java.util.Locale

/**
 * AutopilotLogger — rich CSV log of autopilot session.
 *
 * Started automatically on ENGAGE, stopped on DISENGAGE.
 * File: Downloads/AP_yyyyMMdd_HHmmss.csv
 *
 * Row types (column "type"):
 *
 *   CTRL — autopilot control step (5 Hz)
 *     timestamp, type,
 *     target_hdg, actual_hdg, mag_hdg, heading_error,
 *     deadband, in_deadband,
 *     p_term, i_term, raw_diff, final_diff,
 *     base_speed, port_pct, stbd_pct,
 *     heading_conf, speed_kt,
 *     lat, lon,
 *     sea_state, filter(CF/KF), kalman_bias,
 *     source, engaged
 *
 *   SENS — IMU + Mag raw snapshot (decimated from 50Hz to senseSampleHz)
 *     timestamp, type,
 *     ax, ay, az, gx, gy, gz, mx, my, mz,
 *     raw_mag_hdg, fused_hdg, gyro_z_dps,
 *     tilt_deg, sea_state
 *
 *   GPS — GNSS/A2 update (1 Hz when available)
 *     timestamp, type,
 *     tar_hdg, rmc_cog, blended_hdg, gnss_quality,
 *     tar_acc_deg, satellites, speed_kt,
 *     lat, lon, misalign_deg, misalign_cal
 *
 *   CMD — motor command sent (logged on every sendMotors call)
 *     timestamp, type,
 *     port_pct, stbd_pct, port_duty, stbd_duty, mode(ESC/BLDC)
 */
class AutopilotLogger {

    companion object {
        private const val TAG = "AutopilotLogger"

        // Reduce SENS log rate to avoid huge files (50Hz → 10Hz default)
        const val DEFAULT_SENSE_HZ = 10

        val CTRL_HEADER = listOf(
            "timestamp", "type",
            "target_hdg", "actual_hdg", "mag_hdg", "heading_error",
            "deadband", "in_deadband",
            "p_term", "i_term", "integral", "raw_diff", "final_diff",
            "base_speed", "port_pct", "stbd_pct",
            "heading_conf", "speed_kt",
            "lat", "lon",
            "sea_state", "auto_deadband",
            "filter", "kalman_bias",
            "source"
        )

        val SENS_HEADER = listOf(
            "timestamp", "type",
            "ax", "ay", "az", "gx", "gy", "gz", "mx", "my", "mz",
            "raw_mag_hdg", "fused_hdg", "gyro_z_dps",
            "tilt_deg", "sea_state"
        )

        val GPS_HEADER = listOf(
            "timestamp", "type",
            "tar_hdg", "rmc_cog", "blended_hdg", "gnss_quality",
            "tar_acc_deg", "satellites", "speed_kt",
            "lat", "lon",
            "misalign_deg", "misalign_cal"
        )

        val CMD_HEADER = listOf(
            "timestamp", "type",
            "port_pct", "stbd_pct",
            "port_duty", "stbd_duty",
            "mode"
        )
    }

    // ── Config ────────────────────────────────────────────────────────────────
    /** How many SENS rows per second to write. 0 = don't log sensor rows. */
    var senseSampleHz: Int = DEFAULT_SENSE_HZ

    // ── State ─────────────────────────────────────────────────────────────────
    var isLogging: Boolean = false
        private set
    var logFilePath: String? = null
        private set

    private var writer: PrintWriter? = null
    private val timeFmt  = SimpleDateFormat("yyyy-MM-dd'T'HH:mm:ss.SSS", Locale.US)
    private val fileFmt  = SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US)

    private var senseTickCounter = 0
    private val allHeaders = (CTRL_HEADER + SENS_HEADER + GPS_HEADER + CMD_HEADER).distinct()

    // ── Lifecycle ─────────────────────────────────────────────────────────────

    fun start(): String? {
        if (isLogging) return logFilePath
        return try {
            val dir = Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_DOWNLOADS)
            dir.mkdirs()
            val file = File(dir, "AP_${fileFmt.format(Date())}.csv")
            writer = PrintWriter(FileWriter(file, false))

            // Single wide header — all possible columns
            // Rows only fill their own columns; others are empty
            writer!!.println(allHeaders.joinToString(","))
            writer!!.flush()

            isLogging   = true
            logFilePath = file.absolutePath
            Log.i(TAG, "Autopilot log started: ${file.name}")
            logFilePath
        } catch (e: Exception) {
            Log.e(TAG, "Log start error: ${e.message}")
            null
        }
    }

    fun stop() {
        if (!isLogging) return
        writer?.flush()
        writer?.close()
        writer = null
        isLogging = false
        Log.i(TAG, "Autopilot log stopped: $logFilePath")
    }

    // ── Log rows ──────────────────────────────────────────────────────────────

    /**
     * Log one autopilot control step (call from runControlStep, 5 Hz).
     */
    fun logCtrl(
        targetHdg:    Float,
        actualHdg:    Float,
        magHdg:       Float,
        error:        Float,
        deadband:     Float,
        inDeadband:   Boolean,
        pTerm:        Float,
        iTerm:        Float,
        integral:     Float,
        rawDiff:      Float,
        finalDiff:    Float,
        baseSpeed:    Int,
        portPct:      Int,
        stbdPct:      Int,
        conf:         Float,
        speedKt:      Float,
        lat:          Double,
        lon:          Double,
        seaState:     Float,
        autoDeadband: Float,
        useKalman:    Boolean,
        kalmanBias:   Float,
        source:       String
    ) {
        if (!isLogging) return
        val ts = timeFmt.format(Date())
        val filter = if (useKalman) "KF" else "CF"
        val row = buildRow(allHeaders, CTRL_HEADER, listOf(
            ts, "CTRL",
            f1(targetHdg), f1(actualHdg), f1(magHdg), f2(error),
            f2(deadband), inDeadband.toString(),
            f3(pTerm), f3(iTerm), f3(integral), f3(rawDiff), f3(finalDiff),
            baseSpeed, portPct, stbdPct,
            f2(conf), f2(speedKt),
            f6(lat), f6(lon),
            f3(seaState), f2(autoDeadband),
            filter, f4(kalmanBias),
            source
        ))
        writeLine(row)
    }

    /**
     * Log raw sensor snapshot (call from GpsManager.feedAe02Bytes on A1, decimated).
     */
    fun logSens(
        ax: Short, ay: Short, az: Short,
        gx: Short, gy: Short, gz: Short,
        mx: Short, my: Short, mz: Short,
        rawMagHdg: Float,
        fusedHdg:  Float,
        gyroZDps:  Float,
        tiltDeg:   Float,
        seaState:  Float
    ) {
        if (!isLogging || senseSampleHz <= 0) return
        // Decimate: only log every (50/senseSampleHz) ticks
        senseTickCounter++
        val logEvery = (50 / senseSampleHz.coerceIn(1, 50))
        if (senseTickCounter % logEvery != 0) return

        val ts = timeFmt.format(Date())
        val row = buildRow(allHeaders, SENS_HEADER, listOf(
            ts, "SENS",
            ax, ay, az, gx, gy, gz, mx, my, mz,
            f1(rawMagHdg), f1(fusedHdg), f3(gyroZDps),
            f1(tiltDeg), f3(seaState)
        ))
        writeLine(row)
    }

    /**
     * Log GNSS/A2 update (call from GpsManager on A2 parse, 1 Hz).
     */
    fun logGps(
        tarHdg:      Float,
        rmcCog:      Float,
        blendedHdg:  Float,
        gnssQuality: Int,
        tarAccDeg:   Float,
        satellites:  Int,
        speedKt:     Float,
        lat:         Double,
        lon:         Double,
        misalignDeg: Float,
        misalignCal: Boolean
    ) {
        if (!isLogging) return
        val ts = timeFmt.format(Date())
        val row = buildRow(allHeaders, GPS_HEADER, listOf(
            ts, "GPS",
            f1(tarHdg), f1(rmcCog), f1(blendedHdg), gnssQuality,
            f2(tarAccDeg), satellites, f2(speedKt),
            f6(lat), f6(lon),
            f2(misalignDeg), misalignCal.toString()
        ))
        writeLine(row)
    }

    /**
     * Log every motor command sent (call from sendMotors).
     */
    fun logCmd(
        portPct:   Int,
        stbdPct:   Int,
        portDuty:  Int,
        stbdDuty:  Int,
        escMode:   Boolean
    ) {
        if (!isLogging) return
        val ts = timeFmt.format(Date())
        val row = buildRow(allHeaders, CMD_HEADER, listOf(
            ts, "CMD",
            portPct, stbdPct,
            portDuty, stbdDuty,
            if (escMode) "ESC" else "BLDC"
        ))
        writeLine(row)
    }

    // ── Helpers ───────────────────────────────────────────────────────────────

    /**
     * Build a CSV row aligned to [allCols] using only [rowCols] columns.
     * Columns not in rowCols are left empty.
     */
    private fun buildRow(allCols: List<String>, rowCols: List<String>, values: List<Any>): String {
        val map = rowCols.zip(values).toMap()
        return allCols.joinToString(",") { col ->
            val v = map[col]
            when (v) {
                null -> ""
                is String -> if (v.contains(',')) "\"$v\"" else v
                else -> v.toString()
            }
        }
    }

    private fun writeLine(row: String) {
        try { writer?.println(row); writer?.flush() }
        catch (e: Exception) { Log.e(TAG, "Write error: ${e.message}") }
    }

    private fun f1(v: Float) = "%.1f".format(v)
    private fun f2(v: Float) = "%.2f".format(v)
    private fun f3(v: Float) = "%.3f".format(v)
    private fun f4(v: Float) = "%.4f".format(v)
    private fun f6(v: Double) = "%.6f".format(v)
}
