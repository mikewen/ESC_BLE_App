package com.escbleapp

import android.content.Context
import android.os.Handler
import android.os.Looper
import android.speech.tts.TextToSpeech
import android.util.Log
import java.util.Locale

/**
 * Lightweight TTS wrapper for autopilot voice feedback.
 *
 * Key design decisions:
 *  - Course/speed use a DEBOUNCE delay (300ms) so rapid joystick presses
 *    only speak the final value, not every intermediate step.
 *  - Critical alerts (engage, stop) speak immediately with no debounce.
 *  - Numbers are spoken naturally: 35 → "35", not "035".
 *  - Compass direction is appended to course: "Course 35, north-east".
 *  - QUEUE_FLUSH only for critical (stop/engage) — debounced commands use
 *    QUEUE_ADD after cancelling pending debounce, so the last value always
 *    finishes speaking.
 */
class VoicePrompt(context: Context) {

    companion object {
        private const val TAG = "VoicePrompt"
        private const val DEBOUNCE_MS   = 350L   // wait this long after last press before speaking
        const val MIN_COURSE_DELTA      = 1f      // degrees — suppress if change < this
        const val MIN_SPEED_DELTA       = 1       // percent — suppress if change < this
    }

    private var tts: TextToSpeech? = null
    private var ready = false
    private val handler = Handler(Looper.getMainLooper())

    // Debounce runnables
    private var courseRunnable: Runnable? = null
    private var speedRunnable:  Runnable? = null

    // Last announced values for throttling
    private var lastAnnouncedCourse = Float.NaN
    private var lastAnnouncedSpeed  = Int.MIN_VALUE

    init {
        tts = TextToSpeech(context) { status ->
            if (status == TextToSpeech.SUCCESS) {
                val res = tts?.setLanguage(Locale.US)
                if (res == TextToSpeech.LANG_MISSING_DATA || res == TextToSpeech.LANG_NOT_SUPPORTED)
                    tts?.setLanguage(Locale.getDefault())
                tts?.setSpeechRate(1.1f)
                ready = true
                Log.i(TAG, "TTS ready")
            } else {
                Log.e(TAG, "TTS init failed: $status")
            }
        }
    }

    /**
     * Speak immediately with QUEUE_FLUSH — interrupts everything.
     * Use for critical one-shot alerts: Engage, Stop.
     */
    fun speak(text: String) {
        if (!ready) return
        tts?.speak(text, TextToSpeech.QUEUE_FLUSH, null, "imm_${System.currentTimeMillis()}")
    }

    /**
     * Speak AFTER current speech finishes — never interrupted.
     * Use for alerts that must not be dropped (e.g. "Stop" confirmation).
     */
    fun speakQueued(text: String) {
        if (!ready) return
        tts?.speak(text, TextToSpeech.QUEUE_ADD, null, "q_${System.currentTimeMillis()}")
    }

    /**
     * Debounced course announcement — speaks DEBOUNCE_MS after the last call.
     * Cancels any pending announcement so only the final value is spoken.
     * Throttled: won't re-announce if change < MIN_COURSE_DELTA.
     */
    fun speakCourse(headingDeg: Float) {
        if (!ready) return
        // Cancel pending debounced announcement
        courseRunnable?.let { handler.removeCallbacks(it) }
        // Schedule new announcement
        val runnable = Runnable {
            if (!lastAnnouncedCourse.isNaN() &&
                angleDelta(headingDeg, lastAnnouncedCourse) < MIN_COURSE_DELTA) return@Runnable
            lastAnnouncedCourse = headingDeg
            val deg = headingDeg.toInt() % 360
            val dir = compassWord(deg)
            // Speak naturally: "Course 35" or "Course 350" — no leading zeros
            tts?.speak("Course $deg $dir", TextToSpeech.QUEUE_FLUSH, null,
                "course_${System.currentTimeMillis()}")
        }
        courseRunnable = runnable
        handler.postDelayed(runnable, DEBOUNCE_MS)
    }

    /**
     * Debounced speed announcement — speaks DEBOUNCE_MS after the last call.
     * Throttled: won't re-announce if change < MIN_SPEED_DELTA.
     */
    fun speakSpeed(speedPct: Int) {
        if (!ready) return
        speedRunnable?.let { handler.removeCallbacks(it) }
        val runnable = Runnable {
            if (lastAnnouncedSpeed != Int.MIN_VALUE &&
                Math.abs(speedPct - lastAnnouncedSpeed) < MIN_SPEED_DELTA) return@Runnable
            lastAnnouncedSpeed = speedPct
            val text = if (speedPct == 0) "Stop" else "Speed $speedPct"
            tts?.speak(text, TextToSpeech.QUEUE_FLUSH, null,
                "speed_${System.currentTimeMillis()}")
        }
        speedRunnable = runnable
        handler.postDelayed(runnable, DEBOUNCE_MS)
    }

    /** Reset throttle — next course/speed will always be announced. */
    fun resetThrottle() {
        courseRunnable?.let { handler.removeCallbacks(it) }
        speedRunnable?.let  { handler.removeCallbacks(it) }
        courseRunnable = null; speedRunnable = null
        lastAnnouncedCourse = Float.NaN
        lastAnnouncedSpeed  = Int.MIN_VALUE
    }

    fun shutdown() {
        resetThrottle()
        tts?.stop()
        tts?.shutdown()
        tts = null
        ready = false
    }

    // ── Helpers ───────────────────────────────────────────────────────────────

    private fun angleDelta(a: Float, b: Float): Float {
        var d = Math.abs(a - b) % 360f
        if (d > 180f) d = 360f - d
        return d
    }

    /** Short compass direction word for the given heading. */
    private fun compassWord(deg: Int): String = when (((deg + 22) / 45) % 8) {
        0 -> "north"
        1 -> "north east"
        2 -> "east"
        3 -> "south east"
        4 -> "south"
        5 -> "south west"
        6 -> "west"
        7 -> "north west"
        else -> ""
    }
}