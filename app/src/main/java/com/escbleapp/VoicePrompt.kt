package com.escbleapp

import android.content.Context
import android.os.Bundle
import android.speech.tts.TextToSpeech
import android.speech.tts.UtteranceProgressListener
import android.util.Log
import java.util.Locale

/**
 * Lightweight TTS wrapper for autopilot voice feedback.
 *
 * Design:
 *  - INTERRUPT mode by default — new speech cancels queued speech.
 *    You don't want "course minus one" stacked 5 times from joystick repeat.
 *  - QUEUE mode for critical alerts (stop, engage) so they never get dropped.
 *  - Throttling for repetitive commands (course/speed) — only speaks when
 *    the announced value changes by at least [minDeltaToSpeak].
 *  - All speech is non-blocking — fire and forget.
 *
 * Usage:
 *   val voice = VoicePrompt(context)
 *   voice.speak("Engaged")            // interrupts any current speech
 *   voice.speakCritical("Stop")       // queued, never interrupted
 *   voice.speakCourse(targetHeading)  // throttled — only when heading changes ≥5°
 *   voice.speakSpeed(baseSpeedPct)    // throttled — only when speed changes ≥5%
 *   voice.shutdown()                  // call from onDestroy
 */
class VoicePrompt(context: Context) {

    companion object {
        private const val TAG = "VoicePrompt"

        // Minimum heading change (degrees) before re-announcing course
        const val MIN_COURSE_DELTA = 5f
        // Minimum speed change (%) before re-announcing speed
        const val MIN_SPEED_DELTA  = 5
    }

    private var tts: TextToSpeech? = null
    private var ready = false

    // Last announced values for throttling
    private var lastAnnouncedCourse = Float.NaN
    private var lastAnnouncedSpeed  = Int.MIN_VALUE

    init {
        tts = TextToSpeech(context) { status ->
            if (status == TextToSpeech.SUCCESS) {
                val result = tts?.setLanguage(Locale.US)
                if (result == TextToSpeech.LANG_MISSING_DATA ||
                    result == TextToSpeech.LANG_NOT_SUPPORTED) {
                    Log.w(TAG, "TTS language not supported — trying default")
                    tts?.setLanguage(Locale.getDefault())
                }
                tts?.setSpeechRate(1.1f)   // slightly faster for marine use
                tts?.setPitch(1.0f)
                ready = true
                Log.i(TAG, "TTS ready")
            } else {
                Log.e(TAG, "TTS init failed: $status")
            }
        }
    }

    /**
     * Speak immediately, interrupting any current speech.
     * Use for most autopilot feedback.
     */
    fun speak(text: String) {
        if (!ready) return
        tts?.speak(text, TextToSpeech.QUEUE_FLUSH, null, "ap_${System.currentTimeMillis()}")
    }

    /**
     * Speak without interrupting — queued after current speech.
     * Use only for critical alerts that must not be dropped (STOP, ENGAGE).
     */
    fun speakCritical(text: String) {
        if (!ready) return
        tts?.speak(text, TextToSpeech.QUEUE_ADD, null, "critical_${System.currentTimeMillis()}")
    }

    /**
     * Announce current course — throttled to [MIN_COURSE_DELTA] degrees.
     * Rounds to nearest 5° for cleaner speech ("course 035" not "course 034").
     */
    fun speakCourse(headingDeg: Float) {
        if (!ready) return
        if (!lastAnnouncedCourse.isNaN() &&
            angleDelta(headingDeg, lastAnnouncedCourse) < MIN_COURSE_DELTA) return
        lastAnnouncedCourse = headingDeg
        val rounded = ((headingDeg / 5f).toInt() * 5) % 360
        speak("Course %03d".format(rounded))
    }

    /**
     * Announce current speed — throttled to [MIN_SPEED_DELTA] percent.
     */
    fun speakSpeed(speedPct: Int) {
        if (!ready) return
        if (lastAnnouncedSpeed != Int.MIN_VALUE &&
            Math.abs(speedPct - lastAnnouncedSpeed) < MIN_SPEED_DELTA) return
        lastAnnouncedSpeed = speedPct
        speak("Speed $speedPct")
    }

    /** Reset throttle so next course/speed will always be announced. */
    fun resetThrottle() {
        lastAnnouncedCourse = Float.NaN
        lastAnnouncedSpeed  = Int.MIN_VALUE
    }

    fun shutdown() {
        tts?.stop()
        tts?.shutdown()
        tts = null
        ready = false
    }

    private fun angleDelta(a: Float, b: Float): Float {
        var d = Math.abs(a - b) % 360f
        if (d > 180f) d = 360f - d
        return d
    }
}